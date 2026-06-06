import argparse
import json
import logging
import math
import os
import signal
import time
import urllib.request
from concurrent.futures import Future, ThreadPoolExecutor
from threading import Event, Lock, Thread

import yaml
from PIL import Image, ImageDraw, ImageFont

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
)
logger = logging.getLogger("agent_node")

READY_FLAG = os.environ.get("DERPBOT_READY_FLAG", "/tmp/derpbot_agent_ready")

VLM_INTERVAL_DEFAULT_S = 0.3
VLM_INTERVAL_APPROACH_S = 0.2
COMMIT_REPLAN_FRACTION = 0.25
APPROACH_STANDOFF_M = 0.5  # stop this far short of the target when range is known

# Active scan (#3 detection-frequency). The planner drives toward open space,
# which steers AWAY from cornered targets — the robot reaches proximity but the
# forward camera (90° HFOV) rarely centres the target, so the detector never
# flags it. A periodic step-stop-shoot rotation sweep makes the camera look in
# every direction, one stationary frame at a time. Stationary is REQUIRED: depth
# back-projection localises the bbox using the latest depth + odom, so the robot
# must not be turning between capture and projection or the position is wrong.
SCAN_STEPS = 6                          # 6 × 60° tiles the circle (90° HFOV → 30° overlap)
SCAN_STEP_RAD = 2.0 * math.pi / SCAN_STEPS
SCAN_SETTLE_S = 0.4                     # sim-seconds to stand still before a shot
SCAN_ROTATE_TIMEOUT_S = 4.0            # give up rotating a step (fully wedged) and shoot anyway
SCAN_ANG_SPEED = 0.7                    # rad/s sweep speed
SCAN_PERIOD_S = 20.0                   # min sim-seconds of exploration between scans
SCAN_MIN_ROT_CLEARANCE_M = 0.20        # need ≥ this rotation clearance to bother sweeping
                                       # (else the robot is in a corridor too tight to spin)

# A SMALL bbox touching the image edge is a partial / peripheral view — wall
# clutter sliced by the frame boundary, the dominant close-range FP source (#3).
# Treat it as approach-only so the precise approach re-centres a real target
# before it can publish. A LARGE edge-touching bbox is the opposite case — a
# close target filling the frame — and MUST still publish, so the edge test is
# gated on a small area (an over-broad edge test blocked legit <0.5 m detections).
BBOX_EDGE_MARGIN = 12          # in Gemma 0-1000 units
BBOX_EDGE_MIN = BBOX_EDGE_MARGIN
BBOX_EDGE_MAX = 1000 - BBOX_EDGE_MARGIN
BBOX_EDGE_MAX_AREA_FRAC = 0.25  # only edge-reject bboxes covering < this of the frame

# Approach-then-verify (#3). Detection confidence grows with proximity: a far
# target is a few blurry pixels the verifier can't confirm ("blurry red shape,
# lacks defining characteristics"). So APPROACH is decoupled from PUBLISH — the
# robot drives toward ANY detector sighting to close the distance, but only
# publishes once the verifier confirms it. A verifier reject is only taken as a
# genuine "not the target" when the crop was close enough to judge reliably
# (≤ VERIFY_TRUST_RANGE_M); a far reject means "too far to tell" → keep
# approaching. This also bounds FP-chasing: a real false positive gets
# approached at most to this range, then a close reject ends the chase.
VERIFY_TRUST_RANGE_M = 2.0

# Verifier (#10): when the detector flags target_visible=true, crop the bbox
# region + a margin, upscale if needed, and run a second skeptical VLM call
# before publishing.
VERIFY_BBOX_PAD_FRAC = 0.20    # 20 % padding around bbox before cropping
VERIFY_MIN_CROP_PX = 224       # upscale crops smaller than this on the long edge


def load_config(path: str = "config/vlm_config.yaml") -> dict:
    config_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    full_path = os.path.join(config_root, path)
    with open(full_path) as f:
        return yaml.safe_load(f)


def fetch_mission(url: str, max_retries: int = 30, retry_interval: float = 2.0) -> dict:
    for attempt in range(1, max_retries + 1):
        try:
            resp = urllib.request.urlopen(url, timeout=5)
            data = json.loads(resp.read())
            logger.info("Mission fetched: %s", data.get("description", "no description"))
            return data
        except Exception as e:
            logger.warning("Mission fetch attempt %d/%d failed: %s", attempt, max_retries, e)
            time.sleep(retry_interval)
    raise RuntimeError(f"Failed to fetch mission from {url} after {max_retries} attempts")


class AgentNode:
    def __init__(self, config: dict, frame_dir: str | None = None):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import Image as RosImage
        from nav_msgs.msg import Odometry
        from cv_bridge import CvBridge

        rclpy.init(args=[])
        self.config = config

        self.node = Node(
            "derpbot_agent",
            parameter_overrides=[self._declare_sim_time_param()],
        )

        ros_cfg = config["ros"]
        self.node.get_logger().info("Agent node started with use_sim_time=True")

        self.bridge = CvBridge()
        self._latest_image = None
        self._image_lock = Lock()
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._odom_lock = Lock()
        self._odom_received = False

        qos_be = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.node.create_subscription(
            RosImage, ros_cfg["camera_topic"], self._image_callback, qos_be)
        self.node.create_subscription(
            Odometry, ros_cfg["odom_topic"], self._odom_callback, qos_be)

        self._depth_image = None
        self._depth_lock = Lock()
        depth_topic = ros_cfg.get("depth_topic")
        if depth_topic:
            self.node.create_subscription(
                RosImage, depth_topic, self._depth_callback, qos_be)

        self._camera_K = None
        self._camera_K_lock = Lock()
        cam_info_topic = ros_cfg.get("camera_info_topic", "/derpbot_0/rgbd/camera_info")
        from sensor_msgs.msg import CameraInfo
        self.node.create_subscription(
            CameraInfo, cam_info_topic, self._camera_info_callback, qos_be)

        from agent.safety_layer import ReactiveSafetyLayer
        self.safety = ReactiveSafetyLayer(self.node, config)

        from agent.planner import Planner
        self.planner = Planner()

        from agent.memory import VisitedCells
        self.memory = VisitedCells()

        from agent.vlm_client import VLMClient
        self.vlm = VLMClient(config)

        # Verifier toggle (#10 / #11) — defaults to ENABLED. Set
        #   verifier:
        #     enabled: false
        # in the config to disable, e.g. for A/B with-vs-without runs.
        verifier_cfg = config.get("verifier") or {}
        self._verifier_enabled = bool(verifier_cfg.get("enabled", True))
        logger.info("Verifier: %s", "ENABLED" if self._verifier_enabled else "DISABLED")

        self._detection_pub = None
        self._detection_class = None
        det_topic = ros_cfg.get("detection_topic")
        if det_topic:
            from vision_msgs.msg import Detection2DArray
            self._detection_pub = self.node.create_publisher(
                Detection2DArray, det_topic, 10
            )
            self._detection_class = Detection2DArray

        self._mission = None
        self._done_event = Event()
        self._detection_counter = 0
        inf_cfg = config.get("inference", {})
        self._debug_pause_s = float(inf_cfg.get("debug_pause_s", 0.0))
        self._vlm_executor = ThreadPoolExecutor(max_workers=1)
        self._vlm_future: Future | None = None
        self._frame_dir = frame_dir
        if frame_dir:
            os.makedirs(frame_dir, exist_ok=True)
            self._frame_seq = 0

        # Active-scan state machine (see SCAN_* constants).
        self._scan_active = False
        self._scan_phase = "rotate"          # rotate → settle → query
        self._scan_shots = 0
        self._scan_accum = 0.0               # |yaw| rotated since the last shot
        self._scan_last_yaw = 0.0
        self._scan_dir = 1.0
        self._scan_settle_until = 0.0
        self._scan_rotate_deadline = 0.0
        self._last_scan_end_sim = -1e9

    @staticmethod
    def _declare_sim_time_param():
        from rclpy.parameter import Parameter
        return Parameter("use_sim_time", value=True)

    def _image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            pil_image = Image.fromarray(frame)
            with self._image_lock:
                self._latest_image = pil_image
                self._latest_image_width = pil_image.size[0]
                self._latest_image_height = pil_image.size[1]
        except Exception as e:
            logger.debug("Image callback error: %s", e)

    def _depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            with self._depth_lock:
                self._depth_image = depth
                self._depth_width = depth.shape[1]
                self._depth_height = depth.shape[0]
        except Exception as e:
            logger.debug("Depth callback error: %s", e)

    def _camera_info_callback(self, msg):
        with self._camera_K_lock:
            self._camera_K = list(msg.k)
            self._camera_info_width = msg.width
            self._camera_info_height = msg.height

    def _odom_callback(self, msg):
        with self._odom_lock:
            self._odom_x = msg.pose.pose.position.x
            self._odom_y = msg.pose.pose.position.y
            o = msg.pose.pose.orientation
            siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
            cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
            self._odom_yaw = math.atan2(siny_cosp, cosy_cosp)
            self._odom_received = True

    def _get_latest_image(self):
        with self._image_lock:
            return self._latest_image

    def _get_odom(self):
        with self._odom_lock:
            return self._odom_x, self._odom_y, self._odom_yaw

    def _build_decision_prompt(self) -> str:
        target_obj = self._mission.get("target_object", "the target") if self._mission else "the target"
        target_desc = self._mission.get("target_description", "") if self._mission else ""
        # Underscored class names confuse the VLM ("pipe_sewer_floor" → harder
        # to match than "pipe sewer floor"). Expose both so the model can fall
        # back on whichever it understands.
        target_natural = target_obj.replace("_", " ") if isinstance(target_obj, str) else str(target_obj)
        clearance = self.safety.front_clearance_m()
        clearance_str = (
            "unknown" if clearance is None or math.isinf(clearance) else f"{clearance:.2f} m"
        )
        x, y, yaw = self._get_odom()
        memory_summary = self.memory.render_prompt_summary(x, y, yaw)
        return (
            f"Target: {target_obj}  (natural language: \"{target_natural}\")\n"
            f"Description: {target_desc}\n"
            f"LiDAR front clearance: {clearance_str}\n"
            f"Memory (rays ahead, 0.5 m grid): {memory_summary}\n\n"
            "Look at the image. Decide:\n"
            "  - Is the target visible? Scan floor, corners, walls, edges. The\n"
            "    target may be small or low-contrast (e.g. a grey pipe on a grey\n"
            "    floor, a dark object in shadow). If you see ANY object that\n"
            "    plausibly matches the target, set target_visible=true and fill\n"
            "    target_bbox.\n"
            "  - Which heading (left/center/right) leads toward the target or open space?\n"
            "  - When the target is NOT visible, prefer headings marked 'unexplored' over\n"
            "    'mostly previously visited' to cover new ground.\n"
            "  - How far to drive in that heading (0.0-2.0 m). If front clearance is small,\n"
            "    pick a heading with more space and/or a short distance (≤ 0.5 m).\n"
            "Reply JSON only."
        )

    def _crop_candidate(self, bbox: list, pad_frac: float = VERIFY_BBOX_PAD_FRAC,
                          min_px: int = VERIFY_MIN_CROP_PX):
        """Crop the candidate bbox from the latest camera image, with padding,
        and upscale tiny crops on the long edge. Returns a PIL.Image or None.

        bbox is in Gemma's 0-1000 normalised space (see invariants in STATE.md).
        We crop from the raw camera image — NOT from the resized VLM-input
        image — so that small targets get the full sensor resolution before
        any further downscale in the verifier path.
        """
        with self._image_lock:
            img = self._latest_image
        if img is None or bbox is None or len(bbox) != 4:
            return None

        w, h = img.size
        x1n, y1n, x2n, y2n = bbox
        x1 = max(0.0, min(x1n, x2n)) / 1000.0
        y1 = max(0.0, min(y1n, y2n)) / 1000.0
        x2 = max(x1n, x2n) / 1000.0
        y2 = max(y1n, y2n) / 1000.0

        bw, bh = (x2 - x1), (y2 - y1)
        x1 = max(0.0, x1 - pad_frac * bw)
        y1 = max(0.0, y1 - pad_frac * bh)
        x2 = min(1.0, x2 + pad_frac * bw)
        y2 = min(1.0, y2 + pad_frac * bh)

        px1, py1 = int(x1 * w), int(y1 * h)
        px2, py2 = int(x2 * w), int(y2 * h)
        if px2 - px1 < 2 or py2 - py1 < 2:
            return None

        crop = img.crop((px1, py1, px2, py2))
        cw, ch = crop.size
        long_edge = max(cw, ch)
        if long_edge < min_px:
            scale = min_px / long_edge
            crop = crop.resize((int(cw * scale), int(ch * scale)))
        return crop

    def _save_annotated_frame(self, bbox: list, target_name: str,
                               verdict: str = "visible"):
        """Save an image with the VLM bbox drawn on it, if --save-frames is on."""
        if not self._frame_dir:
            return
        img = self._get_latest_image()
        if img is None:
            return
        w, h = img.size
        x1n, y1n, x2n, y2n = bbox
        px1 = max(0, int(min(x1n, x2n) / 1000.0 * w))
        py1 = max(0, int(min(y1n, y2n) / 1000.0 * h))
        px2 = min(w, int(max(x1n, x2n) / 1000.0 * w))
        py2 = min(h, int(max(y1n, y2n) / 1000.0 * h))

        annotated = img.copy()
        draw = ImageDraw.Draw(annotated)
        line_w = max(2, int(min(w, h) / 150))
        draw.rectangle([px1, py1, px2, py2], outline="lime", width=line_w)
        label = f"{target_name} ({verdict})"
        font_size = max(14, int(min(w, h) / 25))
        try:
            font = ImageFont.truetype(
                "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", font_size)
        except OSError:
            font = ImageFont.load_default(size=font_size)
        text_y = max(0, py1 - font_size - 4)
        draw.text((px1 + 2, text_y), label, fill="lime", font=font)

        seq = self._frame_seq
        self._frame_seq += 1
        path = os.path.join(self._frame_dir, f"frame_{seq:04d}_bbox.png")
        try:
            annotated.save(path)
            logger.info("Annotated frame: %s", os.path.basename(path))
        except Exception as e:
            logger.warning("Annotated frame save failed: %s", e)

    def _verify_detection(self, bbox: list, target_obj: str) -> tuple[bool, str]:
        """Crop + skeptical second VLM call. Returns (confirmed, reason).

        Synchronous — the cloud VLM round-trip is the same ~3 s as a normal
        query, and candidates are rare. Reason is the verifier's explanation
        (or a sentinel describing why the verifier was skipped/failed) so
        callers can log it in the structured CANDIDATE line.
        """
        if not self._verifier_enabled:
            return True, "verifier disabled"
        crop = self._crop_candidate(bbox)
        if crop is None:
            logger.warning("VERIFY: no crop available for bbox=%s — rejecting", bbox)
            return False, "no crop"
        res = self.vlm.verify_candidate(crop, target_obj)
        if res is None:
            return False, "verifier returned None"
        if not res.confirmed:
            logger.info("VERIFY REJECT: %s — %s", target_obj, res.reason[:160])
        return bool(res.confirmed), res.reason or ""

    def _project_target_from_bbox(self, bbox: list | None,
                                    vlm_image_w: int | None = None,
                                    vlm_image_h: int | None = None):
        """Returns (x_map, y_map, depth_m) for the target bbox, or None.

        Gemma 4 (cloud and e2b) emits bbox coordinates in its native 0-1000
        normalized space, independent of input image resolution. We rescale
        that 0-1000 box onto the depth image. ``vlm_image_w/h`` are accepted
        for forward-compatibility (other VLMs may use pixel coords) but are
        currently unused.
        """
        del vlm_image_w, vlm_image_h  # reserved for future non-Gemma backends
        if bbox is None or len(bbox) != 4:
            return None
        with self._depth_lock:
            depth = self._depth_image
        with self._camera_K_lock:
            K = self._camera_K
        if depth is None or K is None:
            return None

        d_h, d_w = depth.shape[:2]
        sx = d_w / 1000.0
        sy = d_h / 1000.0
        scaled = [int(bbox[0] * sx), int(bbox[1] * sy),
                  int(bbox[2] * sx), int(bbox[3] * sy)]

        from agent.depth_projection import back_project_bbox
        odom_x, odom_y, odom_yaw = self._get_odom()
        return back_project_bbox(scaled, depth, K, odom_x, odom_y, odom_yaw)

    def _publish_detection(self, target_type: str, bbox: list | None = None,
                            proj: tuple[float, float, float] | None = None,
                            vlm_image_w: int | None = None,
                            vlm_image_h: int | None = None):
        if self._detection_pub is None:
            return
        from std_msgs.msg import Header
        from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose
        from agent.depth_projection import stable_track_id

        if proj is None:
            proj = self._project_target_from_bbox(bbox, vlm_image_w, vlm_image_h)

        if proj is None:
            # Without a real (bbox + depth) projection we can't locate the
            # target. The old "fallback to robot pose" produced almost-always
            # false positives (robot pose ≠ target pose). Drop the detection
            # so we don't pollute the submission log with FPs.
            logger.info(
                "DETECTION SUPPRESSED: %s — no bbox or depth projection "
                "(bbox=%s, vlm_img=%sx%s)",
                target_type, bbox, vlm_image_w, vlm_image_h,
            )
            return

        tgt_x, tgt_y, est_depth = proj
        track = stable_track_id(target_type, tgt_x, tgt_y)
        log_extra = f"depth={est_depth:.2f}m"

        det = Detection2D()
        det.header = Header()
        det.header.frame_id = "map"
        det.id = track

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = target_type
        hyp.hypothesis.score = 0.8
        hyp.pose.pose.position.x = float(tgt_x)
        hyp.pose.pose.position.y = float(tgt_y)
        det.results.append(hyp)

        msg = self._detection_class()
        msg.header = det.header
        msg.detections.append(det)

        self._detection_pub.publish(msg)
        logger.info("DETECTION: %s id=%s at (%.2f, %.2f) %s",
                    target_type, track, tgt_x, tgt_y, log_extra)

    # Camera horizontal FOV (derpbot RGBD, from the URDF). Used to turn a bbox
    # centre into a precise bearing for the final approach.
    CAMERA_HFOV_RAD = 1.5708

    def _precise_heading_offset(self, bbox) -> float | None:
        """Bearing (rad, +ve = left/CCW) from the camera axis to the bbox
        centre, so the planner can steer straight at the target instead of the
        coarse ±30° buckets. bbox is Gemma 0-1000 space. None if unavailable."""
        if not bbox or len(bbox) != 4:
            return None
        fx = 0.5 * (bbox[0] + bbox[2]) / 1000.0      # 0 = left edge, 1 = right edge
        # Camera +x is to the right; planner convention is +left/CCW, so a
        # target on the right (fx > 0.5) needs a negative (CW) offset.
        offset = -(fx - 0.5) * self.CAMERA_HFOV_RAD
        return max(-self.CAMERA_HFOV_RAD / 2, min(self.CAMERA_HFOV_RAD / 2, offset))

    @staticmethod
    def _norm_angle(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def _debug_pause(self) -> None:
        if self._debug_pause_s > 0:
            logger.info("DEBUG PAUSE: %.1fs (detector→verifier→planner done)",
                        self._debug_pause_s)
            time.sleep(self._debug_pause_s)

    def _evaluate_candidate(self, result, sim_now: float):
        """For a result flagged target_visible+bbox: at close range depth-override
        the drive distance, run the skeptical verifier (#10), log the CANDIDATE
        line and decide the outcome. Returns ``(proj, publish_ok, close)``:
        ``proj`` is the depth projection (x, y, depth) or None; ``publish_ok``
        gates publication; ``close`` is True when the sighting is within trust
        range (used to pick precise vs coarse approach heading).

        Outcomes (see VERIFY_TRUST_RANGE_M / approach-then-verify rationale):
          - FAR (depth > trust range or no depth) → INVESTIGATE: keep visible,
            drift toward it on the VLM's own heading/distance, no publish.
          - CLOSE + verifier CONFIRM → publish_ok=True, precise approach.
          - CLOSE + verifier REJECT → genuine no: demote, resume exploration.
        Shared by the normal loop and the scan so both behave identically."""
        if not (result.target_visible and result.target_bbox):
            logger.info("CANDIDATE: skip (vis=%s bbox=%s)",
                        result.target_visible, result.target_bbox is not None)
            return None, False, False

        proj = self._project_target_from_bbox(
            result.target_bbox, result.image_width, result.image_height)
        depth_m = proj[2] if proj is not None else None
        target_obj = self._mission.get("target_object", "unknown")
        proj_str = (f"({proj[0]:.2f},{proj[1]:.2f})@{depth_m:.2f}m"
                    if proj is not None else "NONE")

        bx1, by1, bx2, by2 = result.target_bbox
        touches_edge = (min(bx1, bx2) <= BBOX_EDGE_MIN or max(bx1, bx2) >= BBOX_EDGE_MAX
                        or min(by1, by2) <= BBOX_EDGE_MIN or max(by1, by2) >= BBOX_EDGE_MAX)
        area_frac = (abs(bx2 - bx1) * abs(by2 - by1)) / 1.0e6   # bbox over 1000×1000 frame
        # peripheral sliver = touches the edge AND is small; a large frame-filling
        # bbox at the edge is a close target and is allowed through.
        edge_touch = touches_edge and area_frac < BBOX_EDGE_MAX_AREA_FRAC

        # Far sightings, OR a bbox sliced by the image edge (partial/peripheral
        # view — the dominant close-range FP): don't verify or publish. Keep the
        # VLM's own clearance-aware heading/distance so the robot drifts toward
        # it and the precise approach re-centres a real target; clutter at the
        # frame boundary never centres and so never publishes.
        if depth_m is None or depth_m > VERIFY_TRUST_RANGE_M or edge_touch:
            outcome = "INVESTIGATE-edge" if edge_touch else "INVESTIGATE"
            logger.info(
                "CANDIDATE: target=%s bbox=%s proj=%s verifier=SKIP outcome=%s",
                target_obj, result.target_bbox, proj_str, outcome)
            self._save_annotated_frame(result.target_bbox, target_obj, outcome)
            return proj, False, False   # keep target_visible=True → gentle approach

        # Close enough to trust: pin the drive distance to the measured range
        # (accurate at short range) so we stop just short of the proximity
        # radius, then run the skeptical verifier and act on it.
        from agent.planner import MAX_DISTANCE_M
        result.drive_distance_m = min(
            MAX_DISTANCE_M, max(0.0, depth_m - APPROACH_STANDOFF_M))
        logger.info("DEPTH OVERRIDE: target depth=%.2fm → commit %.2fm",
                    depth_m, result.drive_distance_m)
        verified, verify_reason = self._verify_detection(
            result.target_bbox, target_obj)
        logger.info(
            "CANDIDATE: target=%s bbox=%s proj=%s verifier=%s outcome=%s reason=%r",
            target_obj, result.target_bbox, proj_str,
            "ACCEPT" if verified else "REJECT",
            "ACCEPT" if verified else "REJECT", verify_reason[:120])
        if not verified:
            self._save_annotated_frame(result.target_bbox, target_obj, "REJECT")
            result.target_visible = False
            result.target_bbox = None
            proj = None
            result.drive_distance_m = min(result.drive_distance_m, 0.8)
            return None, False, True
        self._save_annotated_frame(result.target_bbox, target_obj, "CONFIRM")
        return proj, True, True

    # ── Active scan (step-stop-shoot rotation sweep) ────────────────────────
    # Rotation cooperates with the safety layer instead of fighting it: the
    # directional rotation veto can block the commanded turn direction near a
    # wall (the deadlock-recovery then rotates the robot toward open space,
    # possibly the *other* way). So progress is measured as ACTUAL yaw rotated
    # in any direction (`_scan_accum`), not by reaching an absolute heading.
    # The robot stops, settles and shoots every SCAN_STEP_RAD of accumulated
    # rotation — stationary while capturing so depth projection stays valid.
    def _start_scan(self, sim_now: float) -> None:
        _, _, yaw = self._get_odom()
        self._scan_active = True
        self._scan_phase = "settle"          # shoot the current heading first
        self._scan_shots = 0
        self._scan_accum = 0.0
        self._scan_last_yaw = yaw
        self._scan_dir = 1.0
        self._scan_settle_until = sim_now + SCAN_SETTLE_S
        self._scan_rotate_deadline = sim_now + SCAN_ROTATE_TIMEOUT_S
        self.safety.command(0.0, 0.0, src="scan-stop")
        logger.info("SCAN: start (%d shots × %.0f°)",
                    SCAN_STEPS, math.degrees(SCAN_STEP_RAD))

    def _end_scan(self, sim_now: float, reason: str = "complete") -> None:
        self._scan_active = False
        self._last_scan_end_sim = sim_now
        self.safety.command(0.0, 0.0, src="scan-stop")
        logger.info("SCAN: end (%s)", reason)

    def _scan_after_shot(self, sim_now: float) -> None:
        self._scan_shots += 1
        if self._scan_shots >= SCAN_STEPS:
            self._end_scan(sim_now, reason="swept full circle")
        else:
            self._scan_phase = "rotate"
            self._scan_accum = 0.0
            _, _, self._scan_last_yaw = self._get_odom()
            self._scan_rotate_deadline = sim_now + SCAN_ROTATE_TIMEOUT_S

    def _scan_tick(self, sim_now: float) -> None:
        """One iteration of the scan state machine. Owns the safety command and
        the VLM future while a scan is active."""
        x, y, yaw = self._get_odom()
        self.memory.mark(x, y)

        # A bumper back-off owns motion; pause scan timing until it clears.
        if self.safety.is_blocked():
            self._scan_rotate_deadline = sim_now + SCAN_ROTATE_TIMEOUT_S
            self._scan_last_yaw = yaw
            return

        if self._scan_phase == "rotate":
            delta = abs(self._norm_angle(yaw - self._scan_last_yaw))
            self._scan_accum += delta
            self._scan_last_yaw = yaw
            if self._scan_accum >= SCAN_STEP_RAD:
                logger.info("SCAN: step rotated %.1f° (target %.0f°), settling",
                            math.degrees(self._scan_accum),
                            math.degrees(SCAN_STEP_RAD))
                self.safety.command(0.0, 0.0, src="scan-settle")
                self._scan_phase = "settle"
                self._scan_settle_until = sim_now + SCAN_SETTLE_S
            elif sim_now >= self._scan_rotate_deadline:
                # Rotation wedged (walls all round, safety can't turn us): shoot
                # the current heading anyway, then move on.
                logger.info("SCAN: rotate timeout (accum %.0f°) — shooting current heading",
                            math.degrees(self._scan_accum))
                self.safety.command(0.0, 0.0, src="scan-settle")
                self._scan_phase = "settle"
                self._scan_settle_until = sim_now + SCAN_SETTLE_S
            else:
                self.safety.command(0.0, self._scan_dir * SCAN_ANG_SPEED, src="scan-rotate")
            return

        if self._scan_phase == "settle":
            self.safety.command(0.0, 0.0, src="scan-settle")
            if sim_now >= self._scan_settle_until and self._vlm_future is None:
                self._submit_vlm_query()
                self._scan_phase = "query"
            return

        if self._scan_phase == "query":
            self.safety.command(0.0, 0.0, src="scan-query")
            if self._vlm_future is None or not self._vlm_future.done():
                return
            try:
                result = self._vlm_future.result()
            except Exception as e:
                logger.error("SCAN VLM error: %s", e)
                result = None
            self._vlm_future = None
            if result is not None:
                logger.info("SCAN shot %d/%d: vis=%s | %s",
                            self._scan_shots + 1, SCAN_STEPS,
                            result.target_visible, result.reason[:80])
                proj, publish_ok, close = self._evaluate_candidate(result, sim_now)
                if result.target_visible:
                    # A sighting (confirmed or too-far-to-judge) ends the scan:
                    # leave the sweep to drive toward it. Publish only if the
                    # verifier confirmed. Precise bearing only when close.
                    offset = self._precise_heading_offset(result.target_bbox) if close else None
                    if publish_ok:
                        target_obj = self._mission.get("target_object", "unknown")
                        self._publish_detection(
                            target_obj, result.target_bbox, proj=proj,
                            vlm_image_w=result.image_width,
                            vlm_image_h=result.image_height)
                    self.planner.accept_decision(
                        result, yaw, x, y, sim_now, heading_offset_rad=offset)
                    self._end_scan(sim_now, reason="approaching sighting")
                    self._debug_pause()
                    return
            self._scan_after_shot(sim_now)
            return

    def _check_mission_status(self) -> bool:
        url = self.config["ros"]["mission_url"]
        try:
            resp = urllib.request.urlopen(url, timeout=2)
            data = json.loads(resp.read())
            status = data.get("status", "running")
            if status != "running":
                logger.info("Mission status: %s", status)
                return True
        except Exception:
            pass
        return False

    def _submit_vlm_query(self) -> None:
        image = self._get_latest_image()
        if image is None:
            return
        prompt = self._build_decision_prompt()
        self._vlm_future = self._vlm_executor.submit(self.vlm.query, image, prompt)

    def _commit_progress(self, sim_now: float) -> float:
        """Fraction (0..1) of distance achieved on the current commitment, or 0 if idle."""
        c = self.planner.commitment
        if c is None or c.distance_target_m <= 0.0:
            return 0.0
        x, y, _ = self._get_odom()
        traveled = math.hypot(x - c.start_x, y - c.start_y)
        return min(1.0, traveled / c.distance_target_m)

    def run(self):
        ros_cfg = self.config["ros"]
        self._mission = fetch_mission(ros_cfg["mission_url"])

        self.vlm.start()
        logger.info("VLM client started, waiting for model load...")

        time_limit = self._mission.get("time_limit_seconds", 300)
        sim_clock = self.node.get_clock()
        logger.info("Mission: %s (target=%s, limit=%ds)",
                    self._mission.get("description", ""),
                    self._mission.get("target_object", "?"),
                    time_limit)

        open(READY_FLAG, "w").close()
        logger.info("Agent ready")

        spin_thread = Thread(target=self._spin, daemon=True)
        spin_thread.start()

        last_vlm_sim_s = -1e9
        start_sim_time = None

        try:
            while not self._done_event.is_set():
                sim_now = sim_clock.now().nanoseconds / 1e9
                if start_sim_time is None:
                    start_sim_time = sim_now
                elapsed = sim_now - start_sim_time

                if elapsed >= time_limit:
                    logger.info("Time limit reached (%ds/%ds)", int(elapsed), time_limit)
                    break
                if self._check_mission_status():
                    logger.info("Mission completed!")
                    break

                # Active scan owns motion + the VLM future while running.
                if self._scan_active:
                    self._scan_tick(sim_now)
                    time.sleep(0.05)
                    continue

                # Handle VLM result
                if self._vlm_future is not None and self._vlm_future.done():
                    try:
                        result = self._vlm_future.result()
                    except Exception as e:
                        logger.error("VLM error: %s", e)
                        result = None
                    self._vlm_future = None

                    if result is not None:
                        x, y, yaw = self._get_odom()
                        # Depth-override + verifier gate (shared with the scan
                        # path). Keeps target_visible to APPROACH a sighting
                        # (confirmed or too-far-to-judge); publishes only when
                        # the verifier confirmed it.
                        proj, publish_ok, close = self._evaluate_candidate(result, sim_now)
                        offset = None
                        if result.target_visible:
                            # Re-querying (not scanning) should follow an
                            # approach, so push the scan cadence forward.
                            self._last_scan_end_sim = sim_now
                            # Precise bearing only for close sightings; far ones
                            # use the VLM's coarser, clearance-aware heading.
                            if close:
                                offset = self._precise_heading_offset(result.target_bbox)
                        self.planner.accept_decision(
                            result, yaw, x, y, sim_now, heading_offset_rad=offset)
                        if publish_ok:
                            target_obj = self._mission.get("target_object", "unknown")
                            self._publish_detection(
                                target_obj, result.target_bbox, proj=proj,
                                vlm_image_w=result.image_width,
                                vlm_image_h=result.image_height,
                            )
                        self._debug_pause()

                # Drive: planner → safety
                if self.planner.is_idle() or self.safety.is_blocked():
                    self.safety.command(0.0, 0.0, src="idle")
                else:
                    x, y, yaw = self._get_odom()
                    self.memory.mark(x, y)
                    lin, ang = self.planner.compute_command(x, y, yaw, sim_now)
                    self.safety.command(lin, ang, src="planner")

                # Periodic stop-and-scan. Forward-only queries reach the target
                # but rarely centre it (the planner steers toward open space,
                # away from cornered objects); the sweep guarantees the camera
                # looks every way. Fire purely on cadence — the query pipeline
                # keeps the planner busy commit-to-commit, so we PREEMPT the
                # current commit + any in-flight query rather than waiting for
                # an idle gap that never comes. A fresh sighting pushes
                # _last_scan_end_sim forward (see _evaluate_candidate callers),
                # so a scan never interrupts an approach.
                if (not self.planner.in_approach()
                        and not self.safety.is_blocked()
                        and (sim_now - self._last_scan_end_sim) >= SCAN_PERIOD_S
                        and self.safety.rotation_clearance_m() >= SCAN_MIN_ROT_CLEARANCE_M):
                    self.planner.cancel("scan")
                    self._vlm_future = None
                    self._start_scan(sim_now)
                    continue

                # Issue next VLM query. We pipeline:
                #   - planner idle (commitment ended) → immediate replan
                #   - mid-commit past the replan fraction → submit so the next
                #     decision arrives just as this one finishes, masking the
                #     ~3 s cloud-VLM latency instead of waiting idle for it.
                vlm_interval = (
                    VLM_INTERVAL_APPROACH_S if self.planner.in_approach() else VLM_INTERVAL_DEFAULT_S
                )
                if self._vlm_future is None:
                    should = False
                    if self.planner.is_idle():
                        should = True
                    elif self._commit_progress(sim_now) >= COMMIT_REPLAN_FRACTION:
                        should = (sim_now - last_vlm_sim_s) >= vlm_interval
                    if should:
                        self._submit_vlm_query()
                        last_vlm_sim_s = sim_now

                time.sleep(0.05)

            self.safety.stop()
            logger.info("Agent loop finished (collisions detected by safety: %d)",
                        self.safety.collision_events)

        except KeyboardInterrupt:
            logger.info("Interrupted")
        finally:
            self.safety.stop()
            self.vlm.stop()
            import rclpy
            rclpy.shutdown()

    def _spin(self):
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(self.node)
        try:
            executor.spin()
        except Exception:
            pass


def main(args=None):
    parser = argparse.ArgumentParser(description="DerpBot VLM Agent")
    parser.add_argument("--config", default="config/vlm_config.yaml", help="Path to config file")
    parser.add_argument("--save-frames", default=None, metavar="DIR",
                        help="Save annotated frames with VLM bboxes to DIR "
                             "(default: off; use '.' for cwd)")
    cli_args = parser.parse_args()

    config = load_config(cli_args.config)
    agent = AgentNode(config, frame_dir=cli_args.save_frames)

    def shutdown_handler(sig, frame):
        logger.info("Shutdown signal received")
        agent._done_event.set()

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    agent.run()


if __name__ == "__main__":
    main()
