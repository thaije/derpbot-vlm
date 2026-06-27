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
mkdir -p "$ROOT/runs"
tmux new -s rvr -d "$VENV -m rvr_bridge --target teleop $TELEOP_ONLY --debug-bus $DEBUG_BUS_PORT 2>&1 | tee $ROOT/runs/rvr_console.log"

echo ">>> Starting command panel (HTTP :$PANEL_HTTP_PORT, WS :$((PANEL_HTTP_PORT+1)))..."
tmux new -s panel -d "$VENV -m panel --agent-url ws://127.0.0.1:$DEBUG_BUS_PORT --bind 0.0.0.0:$PANEL_HTTP_PORT"

echo ">>> Restarting Android phone app (adb reverse + relaunch)..."
$VENV -c "
import sys, os, subprocess, time
sys.path.insert(0, '$ROOT')
from rvr_bridge.drive_test import restart_app, _pick_device, _ensure_server_url
import logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(name)s] %(levelname)s: %(message)s')
d = _pick_device()
if d is None:
    print('WARNING: No ADB device found — phone app not started.', file=sys.stderr)
    sys.exit(0)

# --- APK staleness check (#28) -------------------------------------------
# Compare the newest .kt mtime under android/app/src/main/kotlin/ against the
# mtime of the APK installed on the phone. If source is newer, rebuild +
# reinstall via deploy.sh so code changes (e.g. BT auto-enable) actually land
# on the phone — start_rvr.sh previously only relaunched the existing APK.
APK_REMOTE = '/data/app/com.derpbot.app*/base.apk'
KT_ROOT = '$ROOT/android/app/src/main/kotlin'

def newest_kt_mtime():
    latest = 0
    for root, _, files in os.walk(KT_ROOT):
        for f in files:
            if f.endswith('.kt'):
                m = os.path.getmtime(os.path.join(root, f))
                if m > latest:
                    latest = m
    return latest

def installed_apk_mtime(device):
    # Resolve the APK path via 'pm path', then stat it on the phone.
    try:
        paths = subprocess.check_output(
            ['adb'] + (['-s', device] if device else []) + ['shell', 'pm', 'path', 'com.derpbot.app'],
            text=True, timeout=10).strip().splitlines()
        apk = next((p.split(':', 1)[1] for p in paths if p.startswith('package:')), None)
        if not apk:
            return 0.0
        out = subprocess.check_output(
            ['adb'] + (['-s', device] if device else []) + ['shell', 'stat', '-c', '%Y', apk],
            text=True, timeout=10).strip()
        return float(out) if out else 0.0
    except Exception:
        return 0.0

src_mtime = newest_kt_mtime()
apk_mtime = installed_apk_mtime(d)
if src_mtime > apk_mtime and apk_mtime > 0:
    print(f'>>> Kotlin source ({time.ctime(src_mtime)}) newer than installed APK ({time.ctime(apk_mtime)}); redeploying...')
    rc = subprocess.call(['bash', '$ROOT/android/deploy.sh', 'ws://127.0.0.1:8765'])
    if rc != 0:
        print('WARNING: deploy.sh failed — continuing with the existing APK', file=sys.stderr)
    # Re-pick device — deploy.sh may have reconnected ADB.
    d = _pick_device() or d
elif apk_mtime == 0:
    print('>>> Could not determine installed APK mtime — skipping deploy. Run android/deploy.sh manually if the app is outdated.')

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