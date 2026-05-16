#!/usr/bin/env bash
set -euo pipefail

FDS_PORT="${FDS_PORT:-11811}"

pkill -INT -f "agent_node" 2>/dev/null || true
sleep 3

echo "Cleaning up tmux sessions..."
for sess in derpbot agent sim; do
    tmux kill-session -t "$sess" 2>/dev/null || true
done

echo "Killing remaining processes..."
for _round in 1 2; do
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -f "parameter_bridge" 2>/dev/null || true
    pkill -9 -f "scenario_runner" 2>/dev/null || true
    pkill -9 -f "run_scenario" 2>/dev/null || true
    pkill -9 -f "agent_node" 2>/dev/null || true
    pkill -9 -f "ekf_node" 2>/dev/null || true
    pkill -9 -f "robot_state_publisher" 2>/dev/null || true
    pkill -9 -f "mission_server" 2>/dev/null || true
    pkill -9 -f "fastdds discovery" 2>/dev/null || true
    pkill -9 -f "spawn_robot" 2>/dev/null || true
    sleep 1
done

echo "Freeing ports..."
fuser -k 7400/tcp 2>/dev/null || true
fuser -k "$FDS_PORT"/udp 2>/dev/null || true

echo "cleanup done"