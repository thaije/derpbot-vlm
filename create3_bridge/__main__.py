"""CLI entry point for the Create 3 bridge agent (#25).

Usage:
    python3.12 -m create3_bridge --target fire_extinguisher --teleop-only --debug-bus 8770
    python3.12 -m create3_bridge --target fire_extinguisher --ros-domain 0

The Create 3 connects via ROS 2 (/cmd_vel, /imu, /odom, /hazard_detection).
The camera comes from the Android phone in camera-only mode (same relay app,
--camera-only flag in deploy.sh).
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import signal
import sys
from dataclasses import dataclass

from robot_agent.base_agent import BaseAgentConfig
from rvr_bridge.server import PhoneRelay
from .agent import Create3Agent
from .transport import Create3Transport


@dataclass
class Create3Config(BaseAgentConfig):
    ws_host: str = "::"  # dual-stack for adb reverse (phone camera relay)
    ws_port: int = 8765
    ros_domain_id: int = 0
    drive_speed_mps: float = 0.4
    rotate_speed_rad: float = 0.8


def main() -> None:
    parser = argparse.ArgumentParser(description="Create 3 bridge agent — ROS 2 + phone camera (#25)")
    parser.add_argument("--target", required=True, help="Target object to find")
    parser.add_argument("--target-description", default="", help="Natural-language description")
    parser.add_argument("--ollama-url", default="http://localhost:11434", help="Ollama API URL")
    parser.add_argument("--model", default="gemma4:31b-cloud", help="Ollama model name")
    parser.add_argument("--api-key", default=None, help="Ollama API key (for cloud models)")
    parser.add_argument("--ws-host", default="::", help="WebSocket listen host for phone camera relay")
    parser.add_argument("--ws-port", type=int, default=8765, help="WebSocket listen port for phone camera relay")
    parser.add_argument("--ros-domain", type=int, default=0, help="ROS 2 domain ID (must match Create 3)")
    parser.add_argument("--drive-speed", type=float, default=0.4, help="Linear drive speed (m/s)")
    parser.add_argument("--rotate-speed", type=float, default=0.8, help="Rotation speed (rad/s)")
    parser.add_argument("--log-file", default=None, help="JSONL log file path")
    parser.add_argument("--debug-bus", type=int, default=None, metavar="PORT",
                        help="Start debug bus WS server on this port (for panel)")
    parser.add_argument("--teleop-only", action="store_true", default=True,
                        help="Start in teleop mode — autonomous loop paused, panel owns all movement. Default: True.")
    parser.add_argument("--no-teleop-only", dest="teleop_only", action="store_false",
                        help="Start in autonomous mode instead of teleop.")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    )

    config = Create3Config(
        ollama_url=args.ollama_url,
        model=args.model,
        api_key=args.api_key,
        target=args.target,
        target_description=args.target_description,
        ws_host=args.ws_host,
        ws_port=args.ws_port,
        ros_domain_id=args.ros_domain,
        drive_speed_mps=args.drive_speed,
        rotate_speed_rad=args.rotate_speed,
        log_file=args.log_file,
        debug_bus_port=args.debug_bus,
        teleop_only=args.teleop_only,
    )

    relay = PhoneRelay(host=config.ws_host, port=config.ws_port)
    transport = Create3Transport(
        relay,
        ros_domain_id=config.ros_domain_id,
        drive_speed_mps=config.drive_speed_mps,
        rotate_speed_rad=config.rotate_speed_rad,
    )
    agent = Create3Agent(config, transport)

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    def _signal_handler():
        logging.info("Interrupt received, stopping agent...")
        agent.stop()

    loop.add_signal_handler(signal.SIGINT, _signal_handler)
    loop.add_signal_handler(signal.SIGTERM, _signal_handler)

    try:
        loop.run_until_complete(agent.run())
    except KeyboardInterrupt:
        pass
    finally:
        agent.stop()
        loop.close()


if __name__ == "__main__":
    main()