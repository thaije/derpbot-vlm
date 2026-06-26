#!/usr/bin/env bash
# Start the RVR agent (teleop mode), command panel, and Android phone app.
#
# Usage: ./start_rvr.sh [--no-teleop-only]
#
# Requires: ADB device connected (USB or WiFi), Ollama running on localhost:11434.
# Press Ctrl+C to stop everything (tmux sessions are killed on exit).
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT"

VENV="$ROOT/.venv/bin/python"
PYFLAGS=""
TELEOP_ONLY="--teleop-only"
DEBUG_BUS_PORT=8770
PANEL_HTTP_PORT=8081
# Phone WiFi ADB endpoint. Override with PHONE_WIFI_ADDR=<ip:port>.
PHONE_WIFI_ADDR="${PHONE_WIFI_ADDR:-192.168.2.7:5555}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-teleop-only) TELEOP_ONLY=""; shift ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
done

cleanup() {
  tmux kill-session -t rvr 2>/dev/null || true
  tmux kill-session -t panel 2>/dev/null || true
}
trap cleanup INT TERM EXIT

# Kill any existing sessions
cleanup

# --- Connect ADB: prefer WiFi, fall back to USB-enabling wireless adb. ----
# The phone's WebSocket relay reaches the laptop through an `adb reverse`
# tunnel on port 8765. We need *some* ADB link first; we prefer WiFi for
# hands-free operation, but if that's not listening yet, we boot it via USB.
adb_connect_wifi() {
  adb connect "$PHONE_WIFI_ADDR" >/dev/null 2>&1
  # Confirm the device actually came up as "device" (not "offline").
  adb devices | awk -v d="$PHONE_WIFI_ADDR" '$1==d && $2=="device"{found=1} END{exit !found}'
}

if adb_connect_wifi; then
  echo ">>> ADB: connected to phone over WiFi ($PHONE_WIFI_ADDR)"
else
  echo ">>> ADB: WiFi device not reachable ($PHONE_WIFI_ADDR)."
  # Look for a wired USB device and use it to enable `adb tcpip`.
  usb_dev=$(adb devices | awk 'NF && $1 !~ /:/ && $2=="device"{print $1; exit}')
  if [[ -n "$usb_dev" ]]; then
    echo ">>> ADB: enabling wireless adb via USB device $usb_dev..."
    adb -s "$usb_dev" tcpip 5555 >/dev/null 2>&1
    # Give the phone's adbd a moment to start the TCP listener.
    for _ in 1 2 3 4 5 6; do
      sleep 2
      adb_connect_wifi && break
    done
  fi
  if adb_connect_wifi; then
    echo ">>> ADB: WiFi link up via USB bootstrap ($PHONE_WIFI_ADDR)"
  else
    notify-send "RVR start_rvr.sh" \
      "Phone not reachable over WiFi ADB ($PHONE_WIFI_ADDR).\\nConnect it via USB, enable Wireless debugging, then rerun." \
      2>/dev/null || true
    echo "ERROR: Could not reach phone over WiFi ADB ($PHONE_WIFI_ADDR)." >&2
    echo "       Connect the phone via USB, enable Developer options →" >&2
    echo "       Wireless debugging (or run \`adb tcpip 5555\` once), then rerun." >&2
    exit 1
  fi
fi

echo ">>> Starting RVR agent (teleop mode, debug bus :$DEBUG_BUS_PORT)..."
tmux new -s rvr -d "$VENV -m rvr_bridge --target teleop $TELEOP_ONLY --debug-bus $DEBUG_BUS_PORT"

echo ">>> Starting command panel (HTTP :$PANEL_HTTP_PORT, WS :$((PANEL_HTTP_PORT+1)))..."
tmux new -s panel -d "$VENV -m panel --agent-url ws://127.0.0.1:$DEBUG_BUS_PORT --bind 0.0.0.0:$PANEL_HTTP_PORT"

echo ">>> Restarting Android phone app (adb reverse + relaunch)..."
$VENV -c "
from rvr_bridge.drive_test import restart_app, _pick_device, _ensure_server_url
import logging, sys
logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(name)s] %(levelname)s: %(message)s')
d = _pick_device()
if d is None:
    print('WARNING: No ADB device found — phone app not started.', file=sys.stderr)
    sys.exit(0)
# Prefer a WiFi device so the reverse tunnel survives USB unplug.
_ensure_server_url(d, 'ws://127.0.0.1:8765')
restart_app(d)
print(f'Phone app restarted on {d}')
"

echo ""
echo ">>> All systems up."
echo "    Panel:  http://localhost:$((PANEL_HTTP_PORT+1))"
echo "    Logs:   tmux capture-pane -t rvr -p -S -50"
echo "    Stop:   Ctrl+C"
echo ""

# Block until Ctrl+C
echo ">>> Waiting... (Ctrl+C to stop)"
tail -f /dev/null