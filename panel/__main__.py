"""CLI entry point for the command panel (#24).

    python3.12 -m panel --agent-url ws://localhost:8770 --bind 0.0.0.0:8080
"""

from __future__ import annotations

import argparse
import asyncio
import logging

from .proxy import PanelProxy

log = logging.getLogger("panel")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="DerpBot command panel — debug observation + teleop (#24)")
    parser.add_argument("--agent-url", default="ws://localhost:8770",
                        help="Debug bus WebSocket URL of the running agent")
    parser.add_argument("--bind", default="0.0.0.0:8080",
                        help="HTTP+WS bind address (host:port)")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    )

    host, _, port = args.bind.rpartition(":")
    if not host:
        host = "0.0.0.0"
    port_i = int(port) if port else 8080

    proxy = PanelProxy(agent_url=args.agent_url, http_host=host, http_port=port_i)

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(proxy.run())
    except KeyboardInterrupt:
        pass
    finally:
        loop.run_until_complete(proxy.stop())
        loop.close()


if __name__ == "__main__":
    main()