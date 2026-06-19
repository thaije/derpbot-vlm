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
from typing import Optional

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

# Active scan (#3 detection-frequency). The planner drives toward open space,
# which steers AWAY from cornered targets — the robot reaches proximity but the
# forward camera (90° HFOV) rarely centres the target, so the detector never
# flags it. A periodic step-stop-shoot rotation sweep makes the camera look in
# every direction, one stationary frame at a time. Stationary is REQUIRED: depth
# back-projection localises the target using the latest depth + odom, so the robot
# must not be turning between capture and projection or the position is wrong.
# Active-scan constants now live in agent.scan_controller.
from agent.scan_controller import ScanController, ScanContext

CAMERA_HFOV_RAD = 1.5708


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
        self._latest_image_msg = None
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
        qos_reliable = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.node.create_subscription(
            RosImage, ros_cfg["camera_topic"], self._image_callback, qos_be)
        self.node.create_subscription(
            Odometry, ros_cfg["odom_topic"], self._odom_callback, qos_reliable)

        self._depth_msg = None
        self._depth_lock = Lock()
        depth_topic = ros_cfg.get("depth_topic")
        if depth_topic:
            self.node.create_subscription(
                RosImage, depth_topic, self._depth_callback, qos_reliable)

        self._camera_K = None
        self._camera_K_lock = Lock()
        cam_info_topic = ros_cfg.get("camera_info_topic", "/derpbot_0/rgbd/camera_info")
        from sensor_msgs.msg import CameraInfo
        self.node.create_subscription(
            CameraInfo, cam_info_topic, self._camera_info_callback, qos_reliable)

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

        # Active-scan state machine lives in agent.scan_controller.ScanController.
        self._scan: Optional[ScanController] = None
        self._last_confirmed_sim = -1e9

    @staticmethod
    def _declare_sim_time_param():
        from rclpy.parameter import Parameter
        return Parameter("use_sim_time", value=True)

    def _image_callback(self, msg):
        # Store the raw ROS msg only (microseconds). The cv_bridge + PIL
        # conversion is deferred to _get_latest_image(), called ~once per VLM
        # cycle, instead of running on every frame (~30 Hz). Eager per-frame
        # conversion needlessly burned the executor (#15).
        with self._image_lock:
            self._latest_image_msg = msg

    def _depth_callback(self, msg):
        # Store the raw msg only; convert lazily in _project_target_from_location
        # (rare — only when a detection is being localised). See _image_callback.
        with self._depth_lock:
            self._depth_msg = msg

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
            msg = self._latest_image_msg
        if msg is None:
            return None
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            return Image.fromarray(frame)
        except Exception as e:
            logger.debug("Image conversion error: %s", e)
            return None

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
            "    target_location.\n"
            "  - Which heading (left/center/right) leads toward the target or open space?\n"
            "  - When the target is NOT visible, prefer headings marked 'unexplored' over\n"
            "    'mostly previously visited' to cover new ground.\n"
            "  - How far to drive in that heading (0.0-2.0 m). If front clearance is small,\n"
            "    pick a heading with more space and/or a short distance (≤ 0.5 m).\n"
            "Reply JSON only."
        )

    def _save_annotated_frame(self, location: str, target_name: str,
                               verdict: str = "visible"):
        """Save an image with the location text drawn on it, if --save-frames is on."""
        if not self._frame_dir:
            return
        img = self._get_latest_image()
        if img is None:
            return
        annotated = img.copy()
        draw = ImageDraw.Draw(annotated)
        label = f"{target_name} @ {location} ({verdict})"
        font_size = max(14, int(min(img.size) / 25))
        try:
            font = ImageFont.truetype(
                "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", font_size)
        except OSError:
            font = ImageFont.load_default(size=font_size)
        draw.text((4, 4), label, fill="lime", font=font)

        seq = self._frame_seq
        self._frame_seq += 1
        path = os.path.join(self._frame_dir, f"frame_{seq:04d}_loc.png")
        try:
            annotated.save(path)
            logger.info("Annotated frame: %s", os.path.basename(path))
        except Exception as e:
            logger.warning("Annotated frame save failed: %s", e)

    def _verify_detection(self, location: str, target_obj: str) -> tuple[bool, str]:
        """Full-image skeptical second VLM call. Returns (confirmed, reason).

        Synchronous — sends the full camera image with a location hint
        instead of a bbox crop. The verifier judges the whole scene.
        """
        if not self._verifier_enabled:
            return True, "verifier disabled"
        img = self._get_latest_image()
        if img is None:
            logger.warning("VERIFY: no image available — rejecting")
            return False, "no image"
        res = self.vlm.verify_candidate(img, target_obj, location=location)
        if res is None:
            return False, "verifier returned None"
        if not res.confirmed:
            logger.info("VERIFY REJECT: %s — %s", target_obj, res.reason[:160])
        return bool(res.confirmed), res.reason or ""

    def _project_target_from_location(self, location: str | None):
        """Returns (x_map, y_map, depth_m) for the target location, or None.

        Maps the VLM location string to a bearing, reads a depth column at
        that bearing, and back-projects to the map frame.
        """
        if location is None:
            return None
        from agent.depth_projection import location_to_bearing, back_project_from_location
        bearing = location_to_bearing(location)
        if bearing is None:
            return None
        with self._depth_lock:
            depth_msg = self._depth_msg
        with self._camera_K_lock:
            K = self._camera_K
        if depth_msg is None or K is None:
            return None
        try:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            logger.debug("Depth conversion error: %s", e)
            return None
        odom_x, odom_y, odom_yaw = self._get_odom()
        return back_project_from_location(location, depth, K, odom_x, odom_y, odom_yaw)

    def _publish_detection(self, target_type: str, location: str | None = None,
                            proj: tuple[float, float, float] | None = None):
        if self._detection_pub is None:
            return
        from std_msgs.msg import Header
        from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose
        from agent.depth_projection import stable_track_id

        if proj is None and location is not None:
            proj = self._project_target_from_location(location)

        if proj is None:
            logger.info(
                "DETECTION SUPPRESSED: %s — no depth projection "
                "(location=%s)",
                target_type, location,
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

    def _heading_from_location(self, location: str | None) -> float | None:
        """Heading offset (rad, +ve = left/CCW) from the camera axis for a VLM
        location string. The depth-projection bearing convention is negative=left
        (camera x=right), but the planner uses positive=left (yaw CCW), so we
        negate the bearing. None if unavailable."""
        from agent.depth_projection import location_to_bearing
        if location is None:
            return None
        bearing = location_to_bearing(location)
        if bearing is None:
            return None
        return -bearing

    @staticmethod
    def _norm_angle(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def _sim_now(self) -> float:
        """Return current sim time in seconds."""
        return self.node.get_clock().now().nanoseconds / 1e9

    def _debug_pause(self) -> None:
        if self._debug_pause_s > 0:
            logger.info("DEBUG PAUSE: %.1fs (detector→verifier→planner done)",
                        self._debug_pause_s)
            time.sleep(self._debug_pause_s)

    def _evaluate_candidate(self, result, sim_now: float):
        """For a result flagged target_visible+location: verify on the full
        image, log the CANDIDATE line, and decide the outcome.
        Returns ``(proj, publish_ok)``: ``proj`` is the depth projection
        (x, y, depth) or None; ``publish_ok`` gates publication.

        Simplified from the bbox-based approach: no depth-override, no edge
        logic, no trust range. The VLM's own drive_distance_m stands (it sees
        LiDAR clearance). The verifier always runs on the full image with the
        location hint — no crop quality concern.
        """
        if not (result.target_visible and result.target_location):
            logger.info("CANDIDATE: skip (vis=%s loc=%s)",
                        result.target_visible, result.target_location)
            return None, False

        target_obj = self._mission.get("target_object", "unknown")
        location = result.target_location
        proj = self._project_target_from_location(location)
        depth_m = proj[2] if proj is not None else None
        proj_str = (f"({proj[0]:.2f},{proj[1]:.2f})@{depth_m:.2f}m"
                    if proj is not None else "NONE")

        verified, verify_reason = self._verify_detection(location, target_obj)
        logger.info(
            "CANDIDATE: target=%s loc=%s proj=%s verifier=%s reason=%r",
            target_obj, location, proj_str,
            "ACCEPT" if verified else "REJECT", verify_reason[:120])
        if not verified:
            self._save_annotated_frame(location, target_obj, "REJECT")
            result.target_visible = False
            result.target_location = None
            proj = None
            return None, False
        self._save_annotated_frame(location, target_obj, "CONFIRM")
        self._last_confirmed_sim = sim_now
        return proj, True

    # ── Active scan ──────────────────────────────────────────────────────
    # The scan state machine lives in agent.scan_controller.ScanController.
    # The agent provides a ScanContext with the shared dependencies.

    def _make_scan_context(self) -> ScanContext:
        return ScanContext(
            get_odom=self._get_odom,
            sim_now=self._sim_now,
            safety_command=self.safety.command,
            is_blocked=self.safety.is_blocked,
            rotation_clearance_m=self.safety.rotation_clearance_m,
            memory_mark=self.memory.mark,
            submit_vlm_query=self._submit_vlm_query,
            get_vlm_query=lambda: self._vlm_future,
            clear_vlm_future=self._clear_vlm_future,
            evaluate_candidate=self._evaluate_candidate,
            on_sighting=self._on_scan_sighting,
            end_scan=self._end_scan,
            debug_pause=self._debug_pause,
        )

    def _clear_vlm_future(self) -> None:
        self._vlm_future = None

    def _on_scan_sighting(self, result, x: float, y: float, proj, publish_ok: bool) -> None:
        from agent.depth_projection import location_to_bearing
        offset = -location_to_bearing(result.target_location) if result.target_location else None
        if publish_ok:
            target_obj = self._mission.get("target_object", "unknown")
            self._publish_detection(target_obj, location=result.target_location, proj=proj)
        self.planner.accept_decision(
            result, self._get_odom()[2], x, y, self._sim_now(),
            heading_offset_rad=offset)

    def _end_scan(self, sim_now: float, reason: str) -> None:
        if self._scan:
            self._scan.end(sim_now, reason)

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
            logger.warning("VLM: no camera image available — skipping query")
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
                if self._scan is not None and self._scan.active:
                    self._scan.tick(sim_now)
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
                        proj, publish_ok = self._evaluate_candidate(result, sim_now)
                        # Refresh sim_now after blocking VLM calls in evaluate
                        sim_now = self._sim_now()
                        offset = None
                        if result.target_visible:
                            if self._scan:
                                self._scan.last_end_sim = sim_now
                            offset = self._heading_from_location(result.target_location)
                        self.planner.accept_decision(
                            result, yaw, x, y, sim_now, heading_offset_rad=offset)
                        if publish_ok:
                            target_obj = self._mission.get("target_object", "unknown")
                            self._publish_detection(
                                target_obj, location=result.target_location, proj=proj,
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

                # Scan-on-no-detection: trigger a sweep when no detection has
                # been confirmed for SCAN_AFTER_NO_DETECT_S seconds.
                from agent.scan_controller import (
                    SCAN_AFTER_NO_DETECT_S, SCAN_MIN_ROT_CLEARANCE_M)
                if (self._scan is None or not self._scan.active):
                    should_scan = (
                        not self.planner.in_approach()
                        and not self.safety.is_blocked()
                        and (sim_now - self._last_confirmed_sim) >= SCAN_AFTER_NO_DETECT_S
                        and self.safety.rotation_clearance_m() >= SCAN_MIN_ROT_CLEARANCE_M
                    )
                    if should_scan:
                        self.planner.cancel("scan")
                        self._vlm_future = None
                        self._scan = ScanController(self._make_scan_context())
                        self._scan.start(sim_now)
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
        from rclpy.executors import SingleThreadedExecutor
        # SingleThreadedExecutor, NOT MultiThreadedExecutor (#15). The MTE
        # busy-spins in _wait_for_ready_callbacks: with several high-rate
        # subscriptions, entities are perpetually "ready but not executable"
        # (their callback group is busy on another worker), so wait_set.wait()
        # returns immediately in a tight loop, pinning a core at ~100% and
        # starving the GIL. That froze the main control loop for 10-20 s at a
        # time, during which the robot coasted on its last cmd_vel — the scan
        # "spins 180° per step" symptom. The STE blocks properly between
        # callbacks (no cross-thread group contention); callbacks are cheap now
        # that image/depth convert lazily, so one thread keeps up easily.
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)
        try:
            executor.spin()
        except Exception:
            pass


def main(args=None):
    parser = argparse.ArgumentParser(description="DerpBot VLM Agent")
    parser.add_argument("--config", default="config/vlm_config.yaml", help="Path to config file")
    parser.add_argument("--save-frames", default=None, metavar="DIR",
                        help="Save annotated frames with VLM location text to DIR "
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
