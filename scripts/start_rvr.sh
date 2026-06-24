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
trap cleanup EXIT INT TERM

# Kill any existing sessions
cleanup

echo ">>> Starting RVR agent (teleop mode, debug bus :$DEBUG_BUS_PORT)..."
tmux new -s rvr -d "$VENV -m rvr_bridge $TELEOP_ONLY --debug-bus $DEBUG_BUS_PORT"

echo ">>> Starting command panel (HTTP :$PANEL_HTTP_PORT, WS :$((PANEL_HTTP_PORT+1)))..."
tmux new -s panel -d "$VENV -m panel --agent-url ws://localhost:$DEBUG_BUS_PORT --bind 0.0.0.0:$PANEL_HTTP_PORT"

echo ">>> Restarting Android phone app (adb reverse + relaunch)..."
$VENV -c "
from rvr_bridge.drive_test import restart_app, _pick_device, _ensure_server_url
import logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(name)s] %(levelname)s: %(message)s')
d = _pick_device()
if d is None:
    print('WARNING: No ADB device found — phone app not started.')
else:
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

# Wait for Ctrl+C
echo ">>> Waiting... (Ctrl+C to stop)"
wait