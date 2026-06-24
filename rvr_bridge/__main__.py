"""CLI entry point for the RVR bridge agent (#21, refactored #25).

Usage:
    python3.12 -m rvr_bridge --target fire_extinguisher --ollama-url http://localhost:11434
    python3.12 -m rvr_bridge --target fire_extinguisher --restart-app  # hands-off: adb reverse + app restart
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import signal
import sys
from dataclasses import dataclass

from robot_agent.base_agent import BaseAgentConfig
from .agent import RvrAgent
from .drive_test import restart_app, _pick_device, _ensure_server_url
from .server import PhoneRelay
from .transport import RvrTransport


@dataclass
class BridgeConfig(BaseAgentConfig):
    ws_host: str = "::"  # dual-stack: adb reverse tunnel + OkHttp uses IPv6 ::1
    ws_port: int = 8765
    drive_speed_byte: int = 64
    speed_mps: float = 0.35
    max_drive_ms: int = 4000
    bump_threshold_factor: float = 2.5


def main() -> None:
    parser = argparse.ArgumentParser(description="RVR bridge agent — phone-as-BLE-shell (#21)")
    parser.add_argument("--target", required=True, help="Target object to find")
    parser.add_argument("--target-description", default="", help="Natural-language description")
    parser.add_argument("--ollama-url", default="http://localhost:11434", help="Ollama API URL")
    parser.add_argument("--model", default="gemma4:31b-cloud", help="Ollama model name")
    parser.add_argument("--api-key", default=None, help="Ollama API key (for cloud models)")
    parser.add_argument("--ws-host", default="::", help="WebSocket listen host (default :: dual-stack for adb reverse)")
    parser.add_argument("--ws-port", type=int, default=8765, help="WebSocket listen port")
    parser.add_argument("--drive-speed", type=int, default=64, help="RVR drive speed byte (0-255)")
    parser.add_argument("--speed-mps", type=float, default=0.35, help="Speed in m/s for distance→duration")
    parser.add_argument("--bump-threshold", type=float, default=2.5, help="IMU bump detection threshold factor")
    parser.add_argument("--log-file", default=None, help="JSONL log file path")
    parser.add_argument("--debug-bus", type=int, default=None, metavar="PORT",
                        help="Start debug bus WS server on this port (for panel; #24)")
    parser.add_argument("--teleop-only", action="store_true", default=True,
                        help="Start in teleop mode — autonomous loop paused, panel owns all movement (#24). Default: True.")
    parser.add_argument("--no-teleop-only", dest="teleop_only", action="store_false",
                        help="Start in autonomous mode instead of teleop.")
    parser.add_argument("--restart-app", action="store_true",
                        help="Set up adb reverse + restart phone app via ADB (no phone interaction)")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    )

    if args.restart_app:
        device = _pick_device()
        if device is None:
            logging.error("No ADB device found. Is the phone reachable?")
            sys.exit(1)
        logging.info("Using ADB device: %s — restarting app + adb reverse", device)
        restart_app(device)

    config = BridgeConfig(
        ollama_url=args.ollama_url,
        model=args.model,
        api_key=args.api_key,
        target=args.target,
        target_description=args.target_description,
        ws_host=args.ws_host,
        ws_port=args.ws_port,
        drive_speed_byte=args.drive_speed,
        speed_mps=args.speed_mps,
        bump_threshold_factor=args.bump_threshold,
        log_file=args.log_file,
        debug_bus_port=args.debug_bus,
        teleop_only=args.teleop_only,
    )

    relay = PhoneRelay(host=config.ws_host, port=config.ws_port)
    transport = RvrTransport(
        relay,
        drive_speed_byte=config.drive_speed_byte,
        speed_mps=config.speed_mps,
        max_drive_ms=config.max_drive_ms,
        bump_threshold_factor=config.bump_threshold_factor,
    )
    agent = RvrAgent(config, transport)

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