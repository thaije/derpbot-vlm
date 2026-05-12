#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

SCENARIO="${1:-}"
SEED="${2:-}"
EXTRA_ARGS=()

if [ -z "$SCENARIO" ]; then
    echo "Usage: $0 <scenario_yaml> [--seed N] [--headless] [--timeout S]"
    echo "  scenario_yaml: path in robot-sandbox config (e.g. config/scenarios/basement_find/easy.yaml)"
    echo "  --seed N: pin layout for reproducibility"
    echo "  --headless: run without GUI"
    echo "  --timeout S: override scenario timeout"
    exit 1
fi

shift
while [ $# -gt 0 ]; do
    case "$1" in
        --seed) SEED="$2"; shift 2 ;;
        --headless) EXTRA_ARGS+=(--headless); shift ;;
        --timeout) EXTRA_ARGS+=(--timeout "$2"); shift 2 ;;
        *) EXTRA_ARGS+=("$1"); shift ;;
    esac
done

SANDBOX_DIR="${SANDBOX_DIR:-$HOME/Projects/robot-sandbox}"
READY_FLAG="${DERPBOT_READY_FLAG:-/tmp/derpbot_agent_ready}"

cleanup() {
    echo "Shutting down..."
    rm -f "$READY_FLAG"
    tmux kill-session -t derpbot 2>/dev/null || true
    sleep 2
}
trap cleanup EXIT

tmux new-session -d -s derpbot -x 200 -y 50

echo "Starting simulation..."
tmux send-keys -t derpbot "cd $SANDBOX_DIR && ./scripts/run_scenario.sh $SCENARIO ${SEED:+--seed $SEED} ${EXTRA_ARGS[*]}" Enter

echo "Waiting for simulation to be ready..."
for i in $(seq 1 60); do
    if tmux capture-pane -t derpbot -p -S -20 | grep -q "Simulation ready"; then
        echo "Simulation ready"
        break
    fi
    sleep 1
done

echo "Starting VLM agent..."
tmux split-window -t derpbot
tmux send-keys -t derpbot "cd $REPO_ROOT && source .venv/bin/activate && python3.12 agent/agent_node.py" Enter

echo "Waiting for agent to signal ready..."
for i in $(seq 1 120); do
    if [ -f "$READY_FLAG" ]; then
        echo "Agent ready!"
        break
    fi
    sleep 1
done

echo "Agent running. Press Ctrl+C to stop."
tmux attach -t derpbot || true