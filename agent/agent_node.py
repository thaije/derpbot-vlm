import argparse
import json
import logging
import os
import signal
import time
import urllib.request
from threading import Event, Lock, Thread

import yaml
from PIL import Image

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
)
logger = logging.getLogger("agent_node")

READY_FLAG = os.environ.get("DERPBOT_READY_FLAG", "/tmp/derpbot_agent_ready")


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

        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.node.create_subscription(
            RosImage,
            ros_cfg["camera_topic"],
            self._image_callback,
            qos,
        )

        self.node.create_subscription(
            Odometry,
            ros_cfg["odom_topic"],
            self._odom_callback,
            qos,
        )

        from agent.vlm_client import VLMClient
        from agent.action_executor import ActionExecutor
        from agent.safety_layer import SafetyLayer

        self.vlm = VLMClient(config)
        self.executor = ActionExecutor(self.node, config)
        self.safety = SafetyLayer(self.node, config)
        self.safety.set_action_executor(self.executor)

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
        self._action_history: list[str] = []
        self._detection_counter = 0

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
            self._odom_yaw = __import__('math').atan2(siny_cosp, cosy_cosp)

    def _get_latest_image(self):
        with self._image_lock:
            return self._latest_image

    def _get_odom(self):
        with self._odom_lock:
            return self._odom_x, self._odom_y, self._odom_yaw

    def _build_prompt(self) -> str:
        mission = self._mission or {}
        goal = mission.get("goal", "Navigate to the target object.")
        target_desc = mission.get("target_description", mission.get("description", goal))
        target_obj = mission.get("target_object", "the target")
        history = ", ".join(self._action_history[-5:]) if self._action_history else "none"

        odom_x, odom_y, odom_yaw = self._get_odom()
        import math
        heading = f"{math.degrees(odom_yaw):.0f}deg"

        return (
            f"Mission: {target_desc}\n"
            f"Target object: {target_obj}\n"
            f"Robot position: ({odom_x:.2f}, {odom_y:.2f}) heading {heading}\n"
            f"Last 5 actions: [{history}]\n"
            f"What do you do next? Respond with JSON: "
            f'{{"action": "forward|backward|left|right|stop", '
            f'"reasoning": "brief explanation", "target_visible": true|false}}'
        )

    def _publish_detection(self, target_type: str):
        if self._detection_pub is None:
            return

        from std_msgs.msg import Header
        from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose
        from geometry_msgs.msg import Pose, Point

        odom_x, odom_y, odom_yaw = self._get_odom()
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
        logger.info("Published detection: %s at (%.2f, %.2f)", target_type, odom_x, odom_y)

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
        start_sim_time = sim_clock.now().nanoseconds / 1e9

        open(READY_FLAG, "w").close()
        logger.info("Agent ready, signaling startup complete")

        inf_cfg = self.config["inference"]
        tick_interval = 1.0 / inf_cfg["rate_hz"]

        spin_thread = Thread(target=self._spin, daemon=True)
        spin_thread.start()

        try:
            while not self._done_event.is_set():
                current_sim_time = sim_clock.now().nanoseconds / 1e9
                elapsed = current_sim_time - start_sim_time

                if elapsed >= time_limit:
                    logger.info("Time limit reached (%ds/%ds)", int(elapsed), time_limit)
                    break

                if self._check_mission_status():
                    logger.info("Mission completed, exiting loop")
                    break

                image = self._get_latest_image()
                if image is None:
                    logger.debug("No camera image yet, waiting...")
                    time.sleep(0.1)
                    continue

                prompt = self._build_prompt()
                result = self.vlm.query(image, prompt)

                if result:
                    logger.info(
                        "Action: %s | visible: %s | reasoning: %s",
                        result.action, result.target_visible, result.reasoning,
                    )
                    self._action_history.append(result.action)
                    self.executor.execute(result.action, duration_s=tick_interval)

                    if result.target_visible:
                        target_obj = self._mission.get("target_object", "unknown")
                        self._publish_detection(target_obj)
                else:
                    logger.warning("No VLM result, stopping")
                    self.executor.stop()

            self.executor.stop()
            logger.info("Agent loop finished")

        except KeyboardInterrupt:
            logger.info("Interrupted")
        finally:
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