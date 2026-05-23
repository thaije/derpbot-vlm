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

DRIVE_SPEED = 0.4
TURN_SPEED = 0.7
WALL_THRESHOLD = 0.5
WALL_FOLLOW_DIST = 0.8
SAFETY_STOP_DIST = 0.3
DRIFT_SPEED = 0.08
SIDE_THRESHOLD = 0.4
STUCK_THRESHOLD_M = 0.15
STUCK_WINDOW_S = 15.0
WEDGE_BACKUP_SPEED = -0.25
WEDGE_BACKUP_DURATION_S = 1.5


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
        from sensor_msgs.msg import Image as RosImage, LaserScan
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import Twist
        from cv_bridge import CvBridge

        rclpy.init(args=[])
        self.config = config

        self.node = Node(
            "derpbot_agent",
            parameter_overrides=[
                self._declare_sim_time_param()
            ],
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

        self._scan_ranges = None
        self._scan_lock = Lock()

        qos_best_effort = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.node.create_subscription(
            RosImage, ros_cfg["camera_topic"], self._image_callback, qos_best_effort)

        self.node.create_subscription(
            Odometry, ros_cfg["odom_topic"], self._odom_callback, qos_best_effort)

        self.node.create_subscription(
            LaserScan, ros_cfg["lidar_topic"], self._scan_callback, qos_best_effort)

        from agent.safety_layer import ReactiveSafetyLayer
        self.safety = ReactiveSafetyLayer(self.node, config)

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

        self._position_log: list[tuple[float, float, float]] = []
        self._turn_direction = 1
        self._turn_count = 0
        self._consecutive_forward = 0

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
        except Exception as e:
            logger.debug("Image callback error: %s", e)

    def _odom_callback(self, msg):
        with self._odom_lock:
            self._odom_x = msg.pose.pose.position.x
            self._odom_y = msg.pose.pose.position.y
            orientation = msg.pose.pose.orientation
            siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
            cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
            self._odom_yaw = math.atan2(siny_cosp, cosy_cosp)
            self._odom_received = True

    def _scan_callback(self, msg):
        if msg.ranges and len(msg.ranges) > 0:
            with self._scan_lock:
                self._scan_ranges = (
                    msg.ranges,
                    msg.angle_min,
                    msg.angle_increment,
                    msg.range_min,
                    msg.range_max,
                )

    def _get_latest_image(self):
        with self._image_lock:
            return self._latest_image

    def _get_odom(self):
        with self._odom_lock:
            return self._odom_x, self._odom_y, self._odom_yaw

    def _get_scan_zones(self):
        with self._scan_lock:
            if self._scan_ranges is None:
                return None
            ranges, angle_min, angle_inc, range_min, range_max = self._scan_ranges

        front_min, front_max = -0.52, 0.52
        left_min, left_max = 0.52, math.pi / 2
        right_min, right_max = -math.pi / 2, -0.52

        front_dist = range_max
        left_dist = range_max
        right_dist = range_max

        for i, r in enumerate(ranges):
            if math.isnan(r) or math.isinf(r) or r < range_min or r > range_max:
                continue
            angle = angle_min + i * angle_inc
            norm_angle = angle
            while norm_angle > math.pi:
                norm_angle -= 2 * math.pi
            while norm_angle < -math.pi:
                norm_angle += 2 * math.pi

            if front_min <= norm_angle <= front_max:
                front_dist = min(front_dist, r)
            elif left_min <= norm_angle <= left_max:
                left_dist = min(left_dist, r)
            elif right_min <= norm_angle <= right_max:
                right_dist = min(right_dist, r)

        return front_dist, left_dist, right_dist

    def _publish_cmd_vel(self, linear_x: float, angular_z: float):
        self.safety.command(linear_x, angular_z)

    def _check_stuck(self, sim_time: float) -> bool:
        x, y, _ = self._get_odom()
        self._position_log.append((sim_time, x, y))

        cutoff = sim_time - STUCK_WINDOW_S
        self._position_log = [
            (t, px, py) for t, px, py in self._position_log if t >= cutoff
        ]

        if len(self._position_log) < 5:
            return False

        positions = [(px, py) for _, px, py in self._position_log]
        min_x = min(px for px, _ in positions)
        max_x = max(px for px, _ in positions)
        min_y = min(py for _, py in positions)
        max_y = max(py for _, py in positions)
        extent = math.sqrt((max_x - min_x) ** 2 + (max_y - min_y) ** 2)

        return extent < STUCK_THRESHOLD_M

    def _build_detection_prompt(self) -> str:
        target_obj = self._mission.get("target_object", "the target") if self._mission else "the target"
        target_desc = self._mission.get("target_description", "") if self._mission else ""

        return (
            f"You are detecting objects in a robot camera image.\n"
            f"Target: {target_obj}\n"
            f"Description: {target_desc}\n\n"
            f"Is the target object visible in this image? Look carefully at all objects, "
            f"especially edges, corners, walls, and floors.\n"
            f"Reply with JSON: {{\"target_visible\": true/false, "
            f"\"reasoning\": \"brief description of what you see\"}}"
        )

    def _publish_detection(self, target_type: str):
        if self._detection_pub is None:
            return

        from std_msgs.msg import Header
        from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose

        odom_x, odom_y, _ = self._get_odom()
        self._detection_counter += 1

        det = Detection2D()
        det.header = Header()
        det.header.frame_id = "map"
        det.id = f"vlm_track_{self._detection_counter}"

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = target_type
        hyp.hypothesis.score = 0.8
        hyp.pose.pose.position.x = float(odom_x)
        hyp.pose.pose.position.y = float(odom_y)
        det.results.append(hyp)

        msg = self._detection_class()
        msg.header = det.header
        msg.detections.append(det)

        self._detection_pub.publish(msg)
        logger.info("DETECTION: %s at (%.2f, %.2f)", target_type, odom_x, odom_y)

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

        ctrl_interval = 0.1
        last_ctrl_time = time.monotonic()
        last_vlm_time = 0.0
        vlm_interval = 3.0

        start_sim_time = None
        stuck_recovery_end_sim = 0.0
        stuck_cooldown_sim = 0.0
        target_visible_count = 0
        target_visible_threshold = 1
        approach_mode_until_sim = 0.0
        backup_until_sim = 0.0

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

                wall_now = time.monotonic()

                zones = self._get_scan_zones()
                is_stuck = self._check_stuck(sim_now) if elapsed > 10.0 else False
                in_stuck_recovery = sim_now < stuck_recovery_end_sim
                in_stuck_cooldown = sim_now < stuck_cooldown_sim
                in_approach_mode = sim_now < approach_mode_until_sim

                if zones is not None:
                    front_dist, left_dist, right_dist = zones
                    in_backup = sim_now < backup_until_sim

                    if in_backup:
                        self._publish_cmd_vel(WEDGE_BACKUP_SPEED, TURN_SPEED * self._turn_direction * 0.5)
                        self._consecutive_forward = 0

                    elif in_approach_mode:
                        if front_dist < SAFETY_STOP_DIST:
                            safe_dir = -1 if left_dist < right_dist else 1
                            self._turn_direction = safe_dir
                            self._publish_cmd_vel(0.0, TURN_SPEED * safe_dir)
                        else:
                            self._publish_cmd_vel(DRIVE_SPEED, 0.0)
                            self._consecutive_forward += 1

                    elif front_dist < SAFETY_STOP_DIST:
                        if left_dist < SIDE_THRESHOLD and right_dist < SIDE_THRESHOLD:
                            safe_dir = -1 if left_dist < right_dist else 1
                            self._turn_direction = safe_dir
                            backup_until_sim = sim_now + WEDGE_BACKUP_DURATION_S
                            logger.warning("WEDGED at (%.2f,%.2f) f=%.2f L=%.2f R=%.2f, backing up + turning %s",
                                           self._odom_x, self._odom_y, front_dist, left_dist, right_dist,
                                           "right" if safe_dir < 0 else "left")
                            self._publish_cmd_vel(WEDGE_BACKUP_SPEED, TURN_SPEED * safe_dir * 0.5)
                        else:
                            safe_dir = -1 if left_dist < right_dist else 1
                            self._turn_direction = safe_dir
                            self._publish_cmd_vel(0.0, TURN_SPEED * safe_dir * 1.2)
                        self._consecutive_forward = 0

                    elif in_stuck_recovery and sim_now < stuck_recovery_end_sim:
                        self._publish_cmd_vel(0.0, TURN_SPEED * self._turn_direction)
                        last_ctrl_time = wall_now

                    elif front_dist < WALL_THRESHOLD:
                        if front_dist < 0.4:
                            safe_dir = -1 if left_dist < right_dist else 1
                            self._turn_direction = safe_dir
                            self._publish_cmd_vel(0.0, TURN_SPEED * safe_dir)
                        else:
                            turn_away = -1 if left_dist < right_dist else 1
                            self._turn_direction = turn_away
                            self._publish_cmd_vel(0.05, TURN_SPEED * turn_away)
                        self._consecutive_forward = 0

                    elif is_stuck and not in_stuck_cooldown:
                        self._turn_direction = -self._turn_direction
                        stuck_recovery_end_sim = sim_now + 2.5
                        stuck_cooldown_sim = sim_now + 12.0
                        logger.warning("STUCK at (%.2f,%.2f) after %.0fs, turning %s",
                                       self._odom_x, self._odom_y, elapsed,
                                       "right" if self._turn_direction < 0 else "left")
                        self._publish_cmd_vel(0.0, TURN_SPEED * self._turn_direction)

                    else:
                        if left_dist < WALL_FOLLOW_DIST and right_dist > WALL_FOLLOW_DIST:
                            wall_drift = DRIFT_SPEED
                        elif right_dist < WALL_FOLLOW_DIST and left_dist > WALL_FOLLOW_DIST:
                            wall_drift = -DRIFT_SPEED
                        else:
                            wall_drift = 0.0
                        self._publish_cmd_vel(DRIVE_SPEED, wall_drift)
                        self._consecutive_forward += 1

                        if self._consecutive_forward > 80 and front_dist > 2.0:
                            self._turn_direction *= -1
                            self._turn_count += 1
                            if self._turn_count % 4 == 0:
                                self._publish_cmd_vel(DRIVE_SPEED * 0.5, TURN_SPEED * self._turn_direction * 0.5)

                if self._vlm_future is not None and self._vlm_future.done():
                    try:
                        result = self._vlm_future.result()
                    except Exception as e:
                        logger.error("VLM error: %s", e)
                        result = None
                    self._vlm_future = None

                    if result:
                        logger.info("VLM: action=%s vis=%s | %s",
                                    result.action, result.target_visible, result.reasoning)

                        if result.target_visible:
                            target_visible_count += 1
                            target_obj = self._mission.get("target_object", "unknown")
                            self._publish_detection(target_obj)
                            logger.info("TARGET VISIBLE (count=%d), entering approach mode", target_visible_count)
                            approach_mode_until_sim = sim_now + 8.0
                        else:
                            target_visible_count = max(0, target_visible_count - 1)

                elif self._vlm_future is None and wall_now - last_vlm_time >= (vlm_interval if not in_approach_mode else 2.0):
                    image = self._get_latest_image()
                    if image is not None:
                        prompt = self._build_detection_prompt()
                        self._vlm_future = self._vlm_executor.submit(self.vlm.query, image, prompt)
                        last_vlm_time = wall_now

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