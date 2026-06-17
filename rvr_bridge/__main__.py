"""CLI entry point for the RVR bridge agent (#21).

Usage:
    python3.12 -m rvr_bridge --target fire_extinguisher --ollama-url http://localhost:11434
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import signal
import sys

from .agent import BridgeConfig, RvrAgent


def main() -> None:
    parser = argparse.ArgumentParser(description="RVR bridge agent — phone-as-BLE-shell (#21)")
    parser.add_argument("--target", required=True, help="Target object to find")
    parser.add_argument("--target-description", default="", help="Natural-language description")
    parser.add_argument("--ollama-url", default="http://localhost:11434", help="Ollama API URL")
    parser.add_argument("--model", default="gemma4:31b-cloud", help="Ollama model name")
    parser.add_argument("--api-key", default=None, help="Ollama API key (for cloud models)")
    parser.add_argument("--ws-host", default="0.0.0.0", help="WebSocket listen host")
    parser.add_argument("--ws-port", type=int, default=8765, help="WebSocket listen port")
    parser.add_argument("--drive-speed", type=int, default=64, help="RVR drive speed byte (0-255)")
    parser.add_argument("--speed-mps", type=float, default=0.35, help="Speed in m/s for distance→duration")
    parser.add_argument("--bump-threshold", type=float, default=2.5, help="IMU bump detection threshold factor")
    parser.add_argument("--log-file", default=None, help="JSONL log file path")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    )

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
    )

    agent = RvrAgent(config)

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