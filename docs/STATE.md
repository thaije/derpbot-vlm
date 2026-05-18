# STATE — derpbot-vlm

VLM-steered robot agent: camera → VLM → action → cmd_vel. No Nav2, no SLAM, no frontier explorer.
Load this every session. What's next lives in [`ROADMAP.md`](ROADMAP.md); history lives in GitHub issues + commits.

---

## Current performance

**Seed 42, basement_find/easy** (best of 2 runs):

| Model | Overall | Success | Safety | Exploration | Collisions | Min dist | Detections |
|-------|---------|---------|--------|-------------|------------|----------|------------|
| qwen2.5vl:3b | 20/100 | 0 | 100 | 74% | 0 | 4.35m | 0 |
| gemma4:e2b | 16/100 | 0 | 80 | 99% | 1 | 4.35m | 0 |

**Target:** Complete `basement_find/easy` with success=true on ≥ 3/5 seeds. Safety layer prevents all collisions.

---

## Architecture

```
Camera (640x480 RGB) → resize 384px → JPEG quality 70
                                         ↓
                               Ollama HTTP API (gemma4:e2b)
                                         ↓
                               Pydantic-validated JSON: action + reasoning + target_visible
                                         ↓
                              action_executor → /cmd_vel (5x burst)
                                         ↑
Odom (/derpbot_0/odom) → agent_node ─┤
LiDAR (/derpbot_0/scan) → safety_layer → override cmd_vel to zero if <0.3m in ±30° arc
Camera → VLM query (~5s warm) → action → execute 0.25s burst → next VLM query
```

**VLM (Ollama, gemma4:e2b):** 4-5s warm inference, 7.2GB VRAM. Structured output via Ollama `format` parameter + Pydantic schema — no regex parsing needed.
- Output schema: `NavigationAction(action: Literal[5 choices], reasoning: str, target_visible: bool)`
- Fallback: `qwen3.5:2b` (10-17s, verbose), `qwen2.5vl:3b` (left-bias, hallucinates target_visible=True).

**Action → velocity mapping** (from `config/vlm_config.yaml`):

| Action | linear.x | angular.z |
|--------|----------|-----------|
| forward | 0.3 | 0 |
| backward | -0.2 | 0 |
| left | 0 | 0.5 |
| right | 0 | -0.5 |
| stop | 0 | 0 |

**VLM prompt (each tick):**
- System: navigation rules, anti-hallucination, action definitions
- User: mission description + target object + robot position/heading + last 5 actions + camera image
- Output: Pydantic-validated `NavigationAction` with `Literal["forward","backward","left","right","stop"]`, constrained by Ollama `format` parameter

**Detection publishing** (when target_visible=true):
- Topic: `/derpbot_0/detections` (vision_msgs/Detection2DArray)
- Content: class_id=target_object, position=robot_odom, tracking_id=vlm_track_N

**Termination:** agent polls `GET /mission` status. Sim ends on SUCCESS, FAIL, or TIME_LIMIT(300s).

**ROS 2 topics** (all namespaced `/derpbot_0/`):
- Camera: `/derpbot_0/rgbd/image` (sensor_msgs/Image, BEST_EFFORT QoS)
- LiDAR: `/derpbot_0/scan` (sensor_msgs/LaserScan, BEST_EFFORT QoS)
- Odom: `/derpbot_0/odom` (nav_msgs/Odometry, BEST_EFFORT QoS)
- cmd_vel: `/derpbot_0/cmd_vel` (geometry_msgs/Twist)
- Detections: `/derpbot_0/detections` (vision_msgs/Detection2DArray)
- Mission API: `GET http://localhost:7400/mission`

**Repo structure:**
```
agent/
  agent_node.py       — main loop: mission, camera, VLM query, action, detection publish
  vlm_client.py       — Ollama HTTP API client, Pydantic structured output, image resize
  action_executor.py  — action → Twist burst publisher (5x over 0.25s)
  safety_layer.py     — LiDAR forward arc collision stop
scripts/
  start_stack.sh      — launch sim (via robot-sandbox) + agent
  cleanup.sh          — kill all processes, free ports
config/
  vlm_config.yaml     — model choice (gemma4:e2b), inference params, action speeds
tests/
  test_vlm_client.py  — unit tests for NavigationAction schema and VLMResult
docs/
  STATE.md, ROADMAP.md
```

---

## Invariants (will bite again — keep in context)

### Runtime / ROS 2
- **Python interpreter: always `python3.12`.** `python3` may resolve to another venv.
- **`use_sim_time=True` required.** rclpy node must use sim time or messages are silently dropped as future-dated.
- **IMU is BEST_EFFORT QoS.** Subscribe with `ReliabilityPolicy.BEST_EFFORT` or receive nothing.
- **Only one sim run at a time.** Hardware cannot sustain two Gazebo/ROS 2 stacks simultaneously. Always run seeds sequentially.

### VLM / Ollama
- **Structured output via Ollama `format` parameter.** Passes Pydantic `model_json_schema()` to guarantee valid JSON with correct types. No regex/escape parsing needed.
- **Ollama server version matters.** qwen3.5 VL models require server ≥0.24. Current server 0.30.0.
- **Image resize to 384px max dim, JPEG quality 70.** Reduces inference from ~21s to ~5s.
- **`keep_alive=-1` keeps model in VRAM between queries.** Without it, cold start is 15-130s.
- **`max_retries` retained as safety net.** Even with structured output, model can return empty on edge cases. Retries log the failure for debugging.

### Safety layer
- **cmd_vel persists after execute() returns.** This causes collisions — see #7.
- **Safety layer must override cmd_vel at all times, not just set a flag.** Current design sets flag + publishes zero once, but main loop may re-publish forward.

---

## How to run

```bash
# Clean up any old processes
./scripts/cleanup.sh

# Start sim + agent (automated)
./scripts/start_stack.sh config/scenarios/basement_find/easy.yaml --seed 42 --headless

# Manual: start sim, then agent
cd ~/Projects/robot-sandbox && ./scripts/run_scenario.sh config/scenarios/basement_find/easy.yaml --headless --seed 42
source /opt/ros/jazzy/setup.bash && PYTHONPATH=... DERPBOT_READY_FLAG=/tmp/derpbot_agent_ready .venv/bin/python3.12 -m agent.agent_node

# Run tests
PYTHONPATH=. .venv/bin/python3.12 -m pytest tests/ -v -p no:launch_testing
```

Results: `~/Projects/robot-sandbox/results/` via `validate_submission.py`.

---

## Issue tracker

Everything with a lifecycle lives in GitHub issues, not this doc.
- **Active / next work:** [`ROADMAP.md`](ROADMAP.md) — short TOC with links.
- **Before proposing a change, check closed dead-ends:** `gh issue list --state closed --label dead-end --search <topic>`