# STATE — derpbot-vlm

VLM-steered robot agent: camera → VLM → action → cmd_vel. No Nav2, no SLAM, no frontier explorer.
Load this every session. What's next lives in [`ROADMAP.md`](ROADMAP.md); history lives in GitHub issues + commits.

Issue tracker: [#15](https://github.com/thaije/derpbot-vlm/issues/15) (agent spec) and [robot-sandbox #13](https://github.com/thaije/robot-sandbox/issues/13) (proximity-goal scenario).

---

## Current performance

Not yet benchmarked. Development phase.

**Target:** Complete `basement_find/easy` with success=true on ≥ 3/5 seeds. Safety layer prevents all collisions.

---

## Architecture

```
Camera (10 Hz) → [rate limiter 1 Hz] → VLM subprocess
                                           ↓
                                 action + reasoning text
                                           ↓
                              action executor → /cmd_vel
                                           ↑
LiDAR → safety layer (stop if obstacle < 0.3 m, overrides cmd_vel)
```

**VLM (local-first):**
1. Phi-3-Vision (4.2B, ~4–5 GB VRAM) — primary, fully local, no API cost
2. LLaVA-1.5-7B Q4 — fallback if Phi-3 unavailable

**Action → velocity mapping** (from `config/vlm_config.yaml`):

| Action | linear.x | angular.z |
|--------|----------|-----------|
| forward | 0.3 | 0 |
| backward | -0.2 | 0 |
| left | 0 | 0.5 |
| right | 0 | -0.5 |
| stop | 0 | 0 |

Each action runs for 1 s, then VLM re-queried.

**VLM prompt (each tick):**
- System: mission brief from `GET /mission` (target description + proximity_radius)
- User: current camera image + "Last 3 actions: [...]. What do you do next?"
- Output: JSON `{"action": "forward|backward|left|right|stop", "reasoning": "...", "target_visible": bool}`

**Termination:** agent polls `GET /mission` status. Sim ends on SUCCESS (proximity achieved) or TIME_LIMIT.

**ROS 2 topics** (all namespaced `/derpbot_0/`):
- Camera: `/derpbot_0/rgbd/image` (sensor_msgs/Image, BEST_EFFORT QoS)
- LiDAR: `/derpbot_0/scan` (sensor_msgs/LaserScan, BEST_EFFORT QoS)
- cmd_vel: `/derpbot_0/cmd_vel` (geometry_msgs/Twist)
- Mission API: `GET http://localhost:7400/mission`

**Repo structure:**
```
agent/
  agent_node.py       — main loop, ROS2 node, mission polling
  vlm_client.py       — VLM subprocess or API wrapper
  action_executor.py  — action → cmd_vel publisher
  safety_layer.py     — LiDAR collision stop
scripts/
  start_stack.sh      — launch sim (via robot-sandbox) + agent
config/
  vlm_config.yaml     — model choice, inference params, action speeds
docs/
  STATE.md, ROADMAP.md
```

---

## Invariants (will bite again — keep in context)

Anything in committed config/code is omitted. Only things a fresh agent would rediscover the hard way.

### Runtime / ROS 2
- **Python interpreter: always `python3.12`.** `python3` may resolve to another venv.
- **`use_sim_time=True` required.** rclpy node must use sim time or messages are silently dropped as future-dated.
- **IMU is BEST_EFFORT QoS.** Subscribe with `ReliabilityPolicy.BEST_EFFORT` or receive nothing.
- **Only one sim run at a time.** Hardware cannot sustain two Gazebo/ROS 2 stacks simultaneously. Always run seeds sequentially.

### Safety layer
- **Safety layer must be in a separate thread/process.** VLM inference blocks; safety must remain responsive at all times.
- **Safety override is non-negotiable.** If min range < 0.3 m in the forward arc (±30°), publish zero velocity regardless of VLM output.

---

## How to run

```bash
# Start sim + agent
./scripts/start_stack.sh config/scenarios/basement_find/easy.yaml --seed 42 --headless

# Manual agent only (requires running sim)
python3.12 agent/agent_node.py

# Run tests
python3.12 -m pytest tests/
```

Results: `~/Projects/robot-sandbox/results/` via `validate_submission.py`.

---

## Issue tracker

Everything with a lifecycle lives in GitHub issues, not this doc.
- **Active / next work:** [`ROADMAP.md`](ROADMAP.md) — short TOC with links.
- **Before proposing a change, check closed dead-ends:** `gh issue list --state closed --label dead-end --search <topic>`
- **Backlog / known bugs:** `gh issue list --state open --label backlog`