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

VLM_INTERVAL_DEFAULT_S = 1.0
VLM_INTERVAL_APPROACH_S = 0.5
COMMIT_REPLAN_FRACTION = 0.6


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
        clearance = self.safety.front_clearance_m()
        clearance_str = (
            "unknown" if clearance is None or math.isinf(clearance) else f"{clearance:.2f} m"
        )
        x, y, yaw = self._get_odom()
        memory_summary = self.memory.render_prompt_summary(x, y, yaw)
        return (
            f"Target: {target_obj}\n"
            f"Description: {target_desc}\n"
            f"LiDAR front clearance: {clearance_str}\n"
            f"Memory (rays ahead, 0.5 m grid): {memory_summary}\n\n"
            "Look at the image. Decide:\n"
            "  - Is the target visible? If yes, fill target_bbox.\n"
            "  - Which heading (left/center/right) leads toward the target or open space?\n"
            "  - When the target is NOT visible, prefer headings marked 'unexplored' over\n"
            "    'mostly previously visited' to cover new ground.\n"
            "  - How far to drive in that heading (0.0-2.0 m). If front clearance is small,\n"
            "    pick a heading with more space and/or a short distance (≤ 0.5 m).\n"
            "Reply JSON only."
        )

    def _publish_detection(self, target_type: str, bbox: list | None = None):
        if self._detection_pub is None:
            return
        from std_msgs.msg import Header
        from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose
        from agent.depth_projection import back_project_bbox, stable_track_id

        odom_x, odom_y, odom_yaw = self._get_odom()

        # Try depth-based positioning. Fall back to robot pose (counts as FP
        # if too far from ground truth — that's the Phase 4 known limitation
        # when depth or bbox is missing).
        with self._depth_lock:
            depth = self._depth_image
        with self._camera_K_lock:
            K = self._camera_K

        proj = None
        scaled_bbox = bbox
        if bbox is not None and depth is not None and K is not None:
            # The VLM sees a resized image (max 384 px). Bboxes from the VLM
            # are in that resized frame; rescale to depth-image dimensions.
            with self._image_lock:
                img_w = getattr(self, "_latest_image_width", None)
                img_h = getattr(self, "_latest_image_height", None)
            d_h, d_w = depth.shape[:2]
            if img_w and img_h and (img_w != d_w or img_h != d_h):
                sx = d_w / float(img_w)
                sy = d_h / float(img_h)
                scaled_bbox = [
                    int(bbox[0] * sx), int(bbox[1] * sy),
                    int(bbox[2] * sx), int(bbox[3] * sy),
                ]
            proj = back_project_bbox(scaled_bbox, depth, K, odom_x, odom_y, odom_yaw)

        if proj is not None:
            tgt_x, tgt_y, est_depth = proj
            track = stable_track_id(target_type, tgt_x, tgt_y)
            log_extra = f"depth={est_depth:.2f}m bbox_scaled={scaled_bbox}"
        else:
            tgt_x, tgt_y = float(odom_x), float(odom_y)
            self._detection_counter += 1
            track = f"vlm_track_{self._detection_counter}"
            log_extra = "no_depth/bbox (fallback to robot pose)"

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
                        self.planner.accept_decision(result, yaw, x, y, sim_now)
                        if result.target_visible:
                            target_obj = self._mission.get("target_object", "unknown")
                            self._publish_detection(target_obj, result.target_bbox)

                # Drive: planner → safety
                if self.planner.is_idle() or self.safety.is_blocked():
                    self.safety.command(0.0, 0.0)
                else:
                    x, y, yaw = self._get_odom()
                    self.memory.mark(x, y)
                    lin, ang = self.planner.compute_command(x, y, yaw, sim_now)
                    self.safety.command(lin, ang)

                # Issue next VLM query. Trigger immediately when:
                #   - planner just went idle (commitment ended → fresh decision needed), OR
                #   - committed and past replan fraction in approach mode.
                vlm_interval = (
                    VLM_INTERVAL_APPROACH_S if self.planner.in_approach() else VLM_INTERVAL_DEFAULT_S
                )
                if self._vlm_future is None:
                    should = False
                    if self.planner.is_idle():
                        # immediate replan on any commitment end (deadline / no_progress / reached)
                        should = True
                    elif self.planner.in_approach() and self._commit_progress(sim_now) >= COMMIT_REPLAN_FRACTION:
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
