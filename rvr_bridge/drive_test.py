"""Closed-loop smoke test for the phone-as-BLE-shell (#21).

End-to-end, zero phone interaction:
  1. Restart the Android relay app over WiFi ADB (re-scan BLE, re-report state).
  2. Start the WebSocket server on the computer.
  3. Wait for the phone to connect and BLE to reach `ready`.
  4. Send a `drive` command, hold for N seconds, then `stop`.
  5. Optionally `sleep` the RVR and tear down.

Usage:
    python3.12 -m rvr_bridge.drive_test --duration 1.0
    python3.12 -m rvr_bridge.drive_test --speed 64 --heading 90 --duration 2.0
    python3.12 -m rvr_bridge.drive_test --no-restart   # phone app already running
    python3.12 -m rvr_bridge.drive_test --raw-motors   # use raw_motors instead of drive
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import shlex
import subprocess
import sys
import time

from .protocol import (
    DriveMessage,
    RawMotorsMessage,
    ResetYawMessage,
    SleepMessage,
    StopMessage,
    WakeMessage,
)
from .server import PhoneRelay

logger = logging.getLogger("drive_test")

PACKAGE = "com.derpbot.app"
ACTIVITY = f"{PACKAGE}/{PACKAGE}.RelayActivity"
DRIVE_FLAGS_FORWARD = 0x00


def _adb(args: list[str], device: str | None, timeout: float = 15.0) -> str:
    cmd = ["adb"]
    if device:
        cmd += ["-s", device]
    cmd += args
    logger.info("$ %s", " ".join(shlex.quote(c) for c in cmd))
    return subprocess.check_output(cmd, text=True, timeout=timeout).strip()


def _pick_device() -> str | None:
    out = subprocess.check_output(["adb", "devices"], text=True)
    devices = [
        ln.split()[0]
        for ln in out.splitlines()[1:]
        if ln.strip() and "\tdevice" in ln
    ]
    if not devices:
        return None
    # Prefer a WiFi (ip:port) device for hands-off operation.
    for d in devices:
        if ":" in d:
            return d
    return devices[0]


def restart_app(device: str | None) -> None:
    """Wake screen, dismiss keyguard, set up adb reverse, force-stop + relaunch.

    Without an awake+unlocked screen, Samsung's "Freecess" app-freezer caches the
    activity moments after launch and the OkHttp WebSocket is torn down before BLE
    ever reaches `ready`. Keep the screen on while we work.

    ``adb reverse tcp:8765 tcp:8765`` tunnels the phone's 127.0.0.1:8765 to the
    laptop's 127.0.0.1:8765 over the ADB connection. Direct WiFi TCP from the
    phone's OkHttp to the laptop is unreliable on some networks (ECONNABORTED /
    SocketTimeoutException even though ping + nc work) — the ADB reverse tunnel
    sidesteps the phone's WiFi TCP stack entirely.
    """
    # Long screen-off timeout so the phone doesn't sleep mid-drive.
    try:
        _adb(["shell", "settings", "put", "system", "screen_off_timeout",
              "2147483647"], device, timeout=10)
    except subprocess.CalledProcessError:
        pass
    _adb(["shell", "input", "keyevent", "KEYCODE_WAKEUP"], device, timeout=10)
    _adb(["shell", "wm", "dismiss-keyguard"], device, timeout=10)
    # ADB reverse: phone's 127.0.0.1:8765 → laptop's 127.0.0.1:8765.
    # This works for the app process (launched by `am start`) even though
    # `run-as <pkg> nc` can't access the tunnel — the app process's network
    # namespace is set up by Android's Zygote, not by `run-as`.
    # Direct WiFi TCP from OkHttp is unreliable on some networks
    # (ECONNABORTED/SocketTimeoutException even though ping+nc work).
    _adb(["reverse", "tcp:8765", "tcp:8765"], device, timeout=10)
    # Ensure the app's server URL points at the tunnel endpoint.
    _ensure_server_url(device, "ws://127.0.0.1:8765")
    _adb(["shell", "am", "force-stop", PACKAGE], device)
    _adb(["shell", "am", "start", "-n", ACTIVITY], device)


def _ensure_server_url(device: str | None, url: str) -> None:
    """Write the server URL pref if it doesn't already match."""
    # Read current value.
    try:
        current = _adb(
            ["shell", "run-as", PACKAGE, "cat",
             f"/data/data/{PACKAGE}/shared_prefs/rvr_relay.xml"],
            device, timeout=10,
        )
    except subprocess.CalledProcessError:
        current = ""
    if url in current:
        return  # Already correct.
    xml = (
        '<?xml version="1.0" encoding="utf-8" standalone="yes" ?>'
        f'<map><string name="server_url">{url}</string></map>'
    )
    adb_cmd = f"run-as {PACKAGE} sh -c \"cat > /data/data/{PACKAGE}/shared_prefs/rvr_relay.xml\""
    cmd = ["adb"] + (["-s", device] if device else []) + ["shell", adb_cmd]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, text=True)
    proc.communicate(input=xml, timeout=10)


async def drive_once(
    *,
    relay: PhoneRelay,
    speed: int,
    heading: int,
    duration_s: float,
    raw_motors: bool,
    sleep_after: bool,
    ready_timeout: float,
) -> int:
    """Returns 0 on success, non-zero on failure."""
    logger.info("Waiting for phone to connect on ws://%s:%d ...",
                relay.host, relay.port)

    # 1. phone WebSocket connect
    t0 = time.time()
    while not relay.phone_connected:
        if time.time() - t0 > ready_timeout:
            logger.error("Phone did not connect in %.0fs", ready_timeout)
            return 2
        await asyncio.sleep(0.25)

    logger.info("Phone connected. Waiting for BLE ready "
                "(state stream starts on app restart)...")

    # 2. BLE ready
    t0 = time.time()
    while relay.ble_state != "ready":
        if time.time() - t0 > ready_timeout:
            logger.error("BLE did not reach 'ready' in %.0fs "
                         "(last state=%s)", ready_timeout, relay.ble_state)
            return 3
        await asyncio.sleep(0.2)

    logger.info("BLE ready. Waking RVR + zeroing heading.")
    await relay.send(WakeMessage())
    await asyncio.sleep(1.0)
    await relay.send(ResetYawMessage())
    await asyncio.sleep(0.5)

    # 3. drive
    if raw_motors:
        logger.info("RAW MOTORS forward for %.2fs (l=r=%d)", duration_s, speed)
        await relay.send(RawMotorsMessage(l_mode=1, l_speed=speed,
                                          r_mode=1, r_speed=speed))
    else:
        logger.info("DRIVE forward for %.2fs (speed=%d heading=%d)",
                    duration_s, speed, heading)
        await relay.send(DriveMessage(speed=speed, heading=heading,
                                      flags=DRIVE_FLAGS_FORWARD))

    await asyncio.sleep(duration_s)

    # 4. stop
    await relay.send(StopMessage(heading=heading))
    logger.info("STOP sent.")

    # 5. sleep (optional)
    if sleep_after:
        await asyncio.sleep(0.5)
        await relay.send(SleepMessage())
        logger.info("SLEEP sent.")

    return 0


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--duration", type=float, default=1.0,
                   help="Drive duration in seconds (default 1.0)")
    p.add_argument("--speed", type=int, default=64,
                   help="Drive speed byte 0-255 (default 64)")
    p.add_argument("--heading", type=int, default=0, help="Drive heading 0-359")
    p.add_argument("--raw-motors", action="store_true",
                   help="Use raw_motors (both wheels) instead of drive+heading")
    p.add_argument("--ws-host", default="::",
                   help="WebSocket bind host (default :: dual-stack for adb reverse)")
    p.add_argument("--ws-port", type=int, default=8765)
    p.add_argument("--no-restart", action="store_true",
                   help="Don't restart the phone app (assume it's running)")
    p.add_argument("--no-sleep", action="store_true",
                   help="Don't send SLEEP after stop (keep RVR awake)")
    p.add_argument("--ready-timeout", type=float, default=40.0,
                   help="Seconds to wait for phone/BLE ready (default 40)")
    p.add_argument("--device", default=None,
                   help="ADB device serial (default: auto-pick WiFi device)")
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    )

    device = args.device or _pick_device()
    if device is None:
        logger.error("No ADB device found. Is the phone reachable?")
        sys.exit(1)
    logger.info("Using ADB device: %s", device)

    relay = PhoneRelay(host=args.ws_host, port=args.ws_port)

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    async def _run() -> int:
        await relay.start()
        logger.info("WebSocket server up on ws://%s:%d", args.ws_host, args.ws_port)

        if not args.no_restart:
            logger.info("Restarting phone relay app via ADB (no phone interaction)...")
            # Run restart_app in a thread so the event loop stays free to
            # accept the phone's WebSocket connection. Blocking the loop with
            # synchronous subprocess calls prevents the WS handshake from
            # completing, causing OkHttp to time out.
            try:
                await loop.run_in_executor(None, restart_app, device)
            except subprocess.CalledProcessError as e:
                logger.error("ADB restart failed: %s", e)
                await relay.stop()
                return 1

        return await drive_once(
            relay=relay,
            speed=args.speed,
            heading=args.heading,
            duration_s=args.duration,
            raw_motors=args.raw_motors,
            sleep_after=not args.no_sleep,
            ready_timeout=args.ready_timeout,
        )

    try:
        rc = loop.run_until_complete(_run())
    finally:
        loop.run_until_complete(relay.stop())
        loop.close()

    sys.exit(rc)


if __name__ == "__main__":
    main()