import argparse
import json
import logging
import os
import signal
import sys
import time
import urllib.request
from threading import Event, Thread

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
        from sensor_msgs.msg import Image as RosImage
        from cv_bridge import CvBridge

        rclpy.init(args=[], context=rclpy.default_context(self._get_rcl_args()))
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
        self._image_lock = threading.Lock()

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

        from agent.vlm_client import VLMClient
        from agent.action_executor import ActionExecutor
        from agent.safety_layer import SafetyLayer

        self.vlm = VLMClient(config)
        self.executor = ActionExecutor(self.node, config)
        self.safety = SafetyLayer(self.node, config)
        self.safety.set_action_executor(self.executor)

        self._mission = None
        self._done_event = Event()
        self._action_history: list[str] = []

    @staticmethod
    def _get_rcl_args():
        return []

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

    def _get_latest_image(self):
        with self._image_lock:
            return self._latest_image

    def _build_prompt(self) -> str:
        mission = self._mission or {}
        goal = mission.get("goal", "Navigate to the target object.")
        target_desc = mission.get("target_description", mission.get("description", goal))
        history = ", ".join(self._action_history[-3:]) if self._action_history else "none"
        return (
            f"Mission: {target_desc}\n"
            f"Last 3 actions: [{history}]\n"
            f"What do you do next? Respond with JSON: "
            f'{{"action": "forward|backward|left|right|stop", '
            f'"reasoning": "brief explanation", "target_visible": true|false}}'
        )

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