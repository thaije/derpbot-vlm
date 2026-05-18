# STATE — derpbot-vlm

Hybrid agent: LiDAR wall-following for navigation, VLM for target detection. No Nav2, no SLAM, no frontier explorer.
Load this every session. What's next lives in [`ROADMAP.md`](ROADMAP.md); history lives in GitHub issues + commits.

---

## Current performance

**Seed 1, basement_find/easy** (fire_extinguisher target):

| Run | Overall | Exploration | Min dist | Collisions | Detections (TP/FP) |
|-----|---------|-------------|----------|------------|---------------------|
| baseline (VLM-nav) | 16/100 | 99% | 5.58m | 1 | 0/0 |
| LiDAR+VLM hybrid | 4/100 | 99.5% | 1.88m | 16-23 | 0/2 |

**Target:** Complete `basement_find/easy` with success=true on ≥ 3/5 seeds. Proximity ≤ 1m + valid detection.

---

## Architecture

```
Camera → VLM detection only (3s interval, 2s in approach mode)
LiDAR → reactive wall-following (front/left/right zones)
            ↓
  agent_node: hybrid control loop (10Hz cmd_vel)
   - LiDAR: wall-following, safety stop, stuck recovery
   - VLM: target detection (target_visible=true → approach + publish)
   - Approach mode: drive forward when target visible
```

**Navigation (LiDAR-based, reactive):**
- Front zone (<0.3m): safety stop + turn
- Front zone (<0.5m): wall-avoidance turn
- Side zones (<0.8m): drift toward/away from wall
- Stuck detection: position unchanged for 10s → forced turn
- Drive speed: 0.4 m/s forward, 0.7 rad/s turn

**VLM (Ollama, gemma4:e2b):** Detection-only mode. Query every 3s (2s in approach).
- Output: `{"target_visible": bool, "reasoning": str}`
- action field unused (navigation is LiDAR-driven)

**Detection publishing** (when target_visible=true):
- Topic: `/derpbot_0/detections` (vision_msgs/Detection2DArray)
- Content: class_id=target_object, position=robot_odom
- Validation requires: correct type, within 1.5m of ground truth, line-of-sight

**Approach mode** (triggered by target_visible=true):
- Drive forward for 8 sim-seconds
- VLM query interval drops to 2s
- Re-enter if target remains visible

---

## Invariants (will bite again — keep in context)

### Runtime / ROS 2
- **Python interpreter: always `python3.12`.** `python3` may resolve to another venv.
- **`PYTHONPATH` must include both project root AND `/opt/ros/jazzy/lib/python3.12/site-packages`.** Otherwise rclpy import fails.
- **`use_sim_time=True` required.** rclpy node must use sim time or messages are silently dropped as future-dated.
- **Only one sim run at a time.** Hardware cannot sustain two Gazebo/ROS 2 stacks simultaneously.
- **Sim speed affects VLM frequency.** At 3x speed, 300s sim = 100s wall time, only ~15 VLM queries.

### VLM / Ollama
- **VLM detects target ~4% of queries.** gemma4:e2b rarely says target_visible=true even when target is in frame.
- **Detection position must match ground truth within 1.5m.** Robot odom position used; detection at wrong position counts as FP.
- **Line-of-sight required.** Detection through walls counts as FP_LOS, not TP.

### Safety / Navigation
- **LiDAR safety stop at <0.3m.** Pure rotation during safety; no forward movement.
- **Wall-following at 0.5-0.8m.** Reduces collisions from 42 to 16-23.
- **Stuck detection: 10s window, 0.15m threshold.** Forced turn + cooldown prevents oscillation.

---

## How to run

```bash
# Clean up any old processes
./scripts/cleanup.sh

# Start sim + agent manually
cd ~/Projects/robot-sandbox && ./scripts/run_scenario.sh config/scenarios/basement_find/easy.yaml --headless --seed 1

# In separate terminal (after sim ready):
cd ~/Projects/derpbot-vlm
source /opt/ros/jazzy/setup.bash
export PYTHONPATH=/home/plip/Projects/derpbot-vlm:/opt/ros/jazzy/lib/python3.12/site-packages
export DERPBOT_READY_FLAG=/tmp/derpbot_agent_ready
rm -f /tmp/derpbot_agent_ready
.venv/bin/python3.12 -m agent.agent_node

# Run tests
PYTHONPATH=. .venv/bin/python3.12 -m pytest tests/ -v -p no:launch_testing
```

Results: `~/Projects/robot-sandbox/results/` via `validate_submission.py`.

---

## Issue tracker

Everything with a lifecycle lives in GitHub issues, not this doc.
- **Active / next work:** [`ROADMAP.md`](ROADMAP.md) — short TOC with links.
- **Before proposing a change, check closed dead-ends:** `gh issue list --state closed --label dead-end --search <topic>`