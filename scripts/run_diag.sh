#!/usr/bin/env bash
# Diagnostic run harness: start sim + agent headless, capture FULL agent stdout
# (VLM/CANDIDATE/VERIFY lines) to a log, wait for mission end, print funnel.
#
# Usage: scripts/run_diag.sh <seed> [config] [speed] [scenario]
set -o pipefail

SEED="${1:-1}"
CONFIG="${2:-config/vlm_config_cloud.yaml}"
SPEED="${3:-1}"
SCENARIO="${4:-config/scenarios/basement_find/easy.yaml}"

SANDBOX_DIR="$HOME/Projects/robot-sandbox"
REPO_ROOT="$HOME/Projects/derpbot-vlm"
READY_FLAG="/tmp/derpbot_agent_ready"
TS="$(date +%Y%m%dT%H%M%S)"
LOGDIR="$REPO_ROOT/logs"
mkdir -p "$LOGDIR"
AGENT_LOG="$LOGDIR/agent_seed${SEED}_${TS}.log"

echo "[diag] seed=$SEED config=$CONFIG speed=$SPEED log=$AGENT_LOG"

"$REPO_ROOT/scripts/cleanup.sh" >/dev/null 2>&1 || true
rm -f "$READY_FLAG"
sleep 2

# Start sim
tmux kill-session -t sim 2>/dev/null || true
tmux new-session -d -s sim -x 220 -y 50
tmux send-keys -t sim "cd $SANDBOX_DIR && ./scripts/run_scenario.sh $SCENARIO --headless --seed $SEED --speed $SPEED 2>&1 | tee /tmp/sim_seed${SEED}.log" Enter

echo "[diag] waiting for sim ready..."
ready=0
for i in $(seq 1 90); do
    if grep -q "Simulation ready" /tmp/sim_seed${SEED}.log 2>/dev/null; then
        ready=1; echo "[diag] sim ready after ${i}s"; break
    fi
    sleep 1
done
if [ "$ready" -ne 1 ]; then
    echo "[diag] ERROR sim not ready, tail:"; tail -20 /tmp/sim_seed${SEED}.log; exit 1
fi
sleep 3

# Start agent (background process, full log)
cd "$REPO_ROOT"
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$REPO_ROOT:/opt/ros/jazzy/lib/python3.12/site-packages"
export DERPBOT_READY_FLAG="$READY_FLAG"
.venv/bin/python3.12 -m agent.agent_node --config "$CONFIG" > "$AGENT_LOG" 2>&1 &
AGENT_PID=$!
echo "[diag] agent pid=$AGENT_PID"

echo "[diag] waiting for agent ready..."
for i in $(seq 1 120); do
    [ -f "$READY_FLAG" ] && { echo "[diag] agent ready after ${i}s"; break; }
    if ! kill -0 "$AGENT_PID" 2>/dev/null; then echo "[diag] agent died early"; tail -30 "$AGENT_LOG"; exit 1; fi
    sleep 1
done

# Wait for agent to finish (it self-terminates at time limit or mission complete)
echo "[diag] mission running; waiting for agent exit..."
WAIT_MAX=$(( 400 * 100 / (SPEED*100) ))  # generous
SECS=0
while kill -0 "$AGENT_PID" 2>/dev/null; do
    sleep 5; SECS=$((SECS+5))
    if [ "$SECS" -gt 500 ]; then echo "[diag] timeout, killing agent"; kill -INT "$AGENT_PID" 2>/dev/null; sleep 3; break; fi
done
echo "[diag] agent exited after ~${SECS}s wall"
sleep 3

# Cleanup sim
"$REPO_ROOT/scripts/cleanup.sh" >/dev/null 2>&1 || true
tmux kill-session -t sim 2>/dev/null || true

echo ""
echo "===== FUNNEL (seed $SEED) ====="
echo "Target:        $(grep -m1 'Mission fetched' "$AGENT_LOG" | sed 's/.*Mission fetched: //' | cut -c1-80)"
echo "VLM queries:        $(grep -c 'VLM: vis=' "$AGENT_LOG")"
echo "  vis=True flags:   $(grep -c 'VLM: vis=True' "$AGENT_LOG")"
echo "Candidates logged:  $(grep -c 'CANDIDATE:' "$AGENT_LOG")"
echo "  verifier ACCEPT:  $(grep -c 'CANDIDATE:.*verifier=ACCEPT' "$AGENT_LOG")"
echo "  verifier REJECT:  $(grep -c 'CANDIDATE:.*verifier=REJECT' "$AGENT_LOG")"
echo "  proj=NONE:        $(grep -c 'CANDIDATE:.*proj=NONE' "$AGENT_LOG")"
echo "Detections PUBd:    $(grep -c 'DETECTION: ' "$AGENT_LOG")"
echo "Detections SUPPRd:  $(grep -c 'DETECTION SUPPRESSED' "$AGENT_LOG")"
echo "Verify rejects:     $(grep -c 'VERIFY REJECT' "$AGENT_LOG")"
echo "AGENT_LOG=$AGENT_LOG"
