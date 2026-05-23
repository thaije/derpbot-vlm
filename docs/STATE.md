# STATE — derpbot-vlm

Hybrid agent: LiDAR wall-following for navigation, VLM for target detection. No Nav2, no SLAM, no frontier explorer.
Load this every session. What's next lives in [`ROADMAP.md`](ROADMAP.md); history lives in GitHub issues + commits.

---

## Current performance

**basement_find/easy, cloud VLM (`gemma4:31b-cloud`), after Phase 1 of #9 (bumper-aware safety layer):**

| Seed | Target | Score | Exploration | Min dist | Collisions | Near-miss | TP/FP |
|------|--------|-------|-------------|----------|------------|-----------|--------|
| 1 | fire_extinguisher | 4.0 | 99.1% | 5.58m | 11 | 14 | 0/0 |
| 2 | pipe_sewer_floor | 4.0 | 99.1% | 1.30m | **23** | 17 | 0/0 |

**Pre-Phase-1 baseline (#7):** seed 1 → 13 collisions / min 3.44m; seed 2 → 11 collisions / min 1.31m.

**Phase 1 outcome:** bumper sub + back-off + LiDAR forward veto wired up; collision regression on seed 2 (11→23) because the legacy wall-follow re-commands forward immediately after back-off completes, causing hit-back-hit loops on the same wall (7–17 contacts/wall observed). Architecture is the right scaffolding for Phase 2 (VLM steering retires wall-follow), where back-off pays off cleanly.

**Target:** Complete `basement_find/easy` with success=true on ≥ 3/5 seeds. Proximity ≤ 1m + valid detection.

**Cloud VLM (#8) findings:** detection rate target-dependent — fire_extinguisher detects ~14% of queries; pipe_sewer_floor ~0%. All detections still FP (position = robot odom).

---

## Architecture

```
Camera → VLM detection only (3s interval, 2s in approach mode)
LiDAR → reactive wall-following (front/left/right zones)
            ↓
  agent_node: hybrid control loop (10Hz desired)
   - LiDAR: wall-following, safety stop, stuck recovery
   - VLM: target detection (target_visible=true → approach + publish)
   - Approach mode: drive forward when target visible
            ↓
  ReactiveSafetyLayer (20Hz, owns /cmd_vel):
   - command(lin, ang) API from agent_node
   - LiDAR forward veto: front<0.25m → zero linear, slide via rotation
   - Bumper back-off: non-ground contact → 1.5s reverse + rotate toward open side
```

**Navigation (LiDAR-based, reactive):**
- Front zone (<0.3m): safety stop + turn (agent_node)
- Front zone (<0.5m): wall-avoidance turn (agent_node)
- Front zone (<0.25m): forward veto (safety_layer backstop)
- Side zones (<0.8m): drift toward/away from wall
- Stuck detection: position unchanged for 10s → forced turn
- Bumper contact (`/derpbot_0/bumper_contact`): back-off 1.5s (-0.2 m/s + 0.6 rad/s)
- Drive speed: 0.4 m/s forward, 0.7 rad/s turn

**VLM (Ollama, gemma4:e2b):** Detection-only mode. Query every 3s (2s in approach).
- Output: `{"target_visible": bool, "reasoning": str}`
- action field unused (navigation is LiDAR-driven)
- Supports local (`gemma4:e2b`) and cloud (`gemma4:31b-cloud`) backends
- Cloud backend: `backend: ollama-cloud` in config, `keep_alive=0`, no local model load
- Response parser handles: strict JSON, code-fenced JSON, partial JSON dicts, free text

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
- **Cloud VLM detects target more often** (~3.5x) but all FPs — position is robot odom, not object location.
- **Detection position must match ground truth within 1.5m.** Robot odom position used; detection at wrong position counts as FP.
- **Line-of-sight required.** Detection through walls counts as FP_LOS, not TP.
- **Cloud models may return free text instead of JSON.** Parser handles both via `_parse_vlm_response`.
- **`ollama signin` required before cloud models.** Run once; auth persists.

### Safety / Navigation
- **`ReactiveSafetyLayer` owns `/cmd_vel`.** agent_node calls `safety.command(lin, ang)`; safety publishes the filtered twist at 20 Hz. Direct publishes from agent_node would race with the timer.
- **Bumper ground-plane filter is string-match only** (`"ground_plane" in str(c.collision1/2)`). A per-contact-normal filter (`|n.z| > 0.9`) over-rejects legitimate wall hits — `normals[]` is empty or non-horizontal on real wall contacts. Mirror `metrics/collision_count.py`.
- **LiDAR safety stop at <0.3m (agent_node), forward veto at <0.25m (safety_layer).** Pure rotation during safety; no forward movement.
- **Wall-following at 0.5-0.8m.** Reduces collisions from 42 to ~12.
- **Safety/wedge turn direction must come from LiDAR side-clearance**, not stored `_turn_direction`. Otherwise robot turns into walls. Wedge (front<0.3 AND both sides<0.4) → backup+rotate for 1.5 sim-s.
- **Stuck detection: 10s window, 0.15m threshold.** Forced turn + cooldown prevents oscillation.
- **Bumper back-off + wall-follow fight each other.** After 1.5s back-off, wall-follow re-commands forward into the same wall (7+ contacts/wall observed). Will be resolved by Phase 2 of #9 (retire wall-follow for VLM steering); meanwhile a "clearing rotation" after back-off was tried but worsened collisions (21 vs 11 on seed 1) and was reverted.

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

# Run with local VLM (default):
.venv/bin/python3.12 -m agent.agent_node --config config/vlm_config.yaml

# Run with cloud VLM (gemma4:31b-cloud, requires `ollama signin`):
.venv/bin/python3.12 -m agent.agent_node --config config/vlm_config_cloud.yaml

# Run tests
PYTHONPATH=. .venv/bin/python3.12 -m pytest tests/ -v -p no:launch_testing
```

Results: `~/Projects/robot-sandbox/results/` via `validate_submission.py`.

---

## Issue tracker

Everything with a lifecycle lives in GitHub issues, not this doc.
- **Active / next work:** [`ROADMAP.md`](ROADMAP.md) — short TOC with links.
- **Before proposing a change, check closed dead-ends:** `gh issue list --state closed --label dead-end --search <topic>`