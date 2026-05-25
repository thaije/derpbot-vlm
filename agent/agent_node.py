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
from PIL import Image

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
    def __init__(self, config: dict):
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
        self._vlm_executor = ThreadPoolExecutor(max_workers=1)
        self._vlm_future: Future | None = None

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

    def _verify_detection(self, bbox: list, target_obj: str) -> bool:
        """Crop + skeptical second VLM call. Returns True iff the verifier
        confirms the candidate. Synchronous — the cloud VLM round-trip is the
        same ~3 s as a normal query, and candidates are rare."""
        crop = self._crop_candidate(bbox)
        if crop is None:
            logger.warning("VERIFY: no crop available for bbox=%s — rejecting", bbox)
            return False
        res = self.vlm.verify_candidate(crop, target_obj)
        confirmed = bool(res and res.confirmed)
        if not confirmed:
            reason = (res.reason if res else "verifier failed") or ""
            logger.info("VERIFY REJECT: %s — %s", target_obj, reason[:160])
        return confirmed

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
                        proj = None
                        if result.target_visible and result.target_bbox:
                            proj = self._project_target_from_bbox(
                                result.target_bbox,
                                result.image_width,
                                result.image_height,
                            )
                            if proj is not None:
                                # Override VLM's guessed drive_distance_m with the
                                # measured range-to-target minus a standoff so the
                                # robot stops just short of the proximity radius.
                                _, _, depth_m = proj
                                adjusted = max(0.0, depth_m - APPROACH_STANDOFF_M)
                                # Cap at 2 m commit but allow up to that even if
                                # VLM guessed something smaller.
                                from agent.planner import MAX_DISTANCE_M
                                result.drive_distance_m = min(MAX_DISTANCE_M, adjusted)
                                logger.info(
                                    "DEPTH OVERRIDE: target depth=%.2fm → commit %.2fm",
                                    depth_m, result.drive_distance_m,
                                )
                        if result.target_visible and result.target_bbox:
                            # Skeptical second-call verifier (#10). Gates BOTH
                            # the detection publication AND the planner's
                            # approach-mode decision: if the verifier rejects,
                            # the candidate is treated as "not seen" so we
                            # don't commit cycles to driving toward an FP.
                            target_obj = self._mission.get("target_object", "unknown")
                            verified = self._verify_detection(result.target_bbox, target_obj)
                            if not verified:
                                result.target_visible = False
                                result.target_bbox = None
                                proj = None
                                # Reset the inflated drive_distance from the
                                # earlier DEPTH OVERRIDE — we no longer believe
                                # the candidate exists at that range.
                                result.drive_distance_m = min(result.drive_distance_m, 0.8)

                        self.planner.accept_decision(result, yaw, x, y, sim_now)
                        if result.target_visible:
                            target_obj = self._mission.get("target_object", "unknown")
                            self._publish_detection(
                                target_obj, result.target_bbox, proj=proj,
                                vlm_image_w=result.image_width,
                                vlm_image_h=result.image_height,
                            )

                # Drive: planner → safety
                if self.planner.is_idle() or self.safety.is_blocked():
                    self.safety.command(0.0, 0.0)
                else:
                    x, y, yaw = self._get_odom()
                    self.memory.mark(x, y)
                    lin, ang = self.planner.compute_command(x, y, yaw, sim_now)
                    self.safety.command(lin, ang)

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
    cli_args = parser.parse_args()

    config = load_config(cli_args.config)
    agent = AgentNode(config)

    def shutdown_handler(sig, frame):
        logger.info("Shutdown signal received")
        agent._done_event.set()

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    agent.run()


if __name__ == "__main__":
    main()
