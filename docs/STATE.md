# STATE — derpbot-vlm

VLM-steered agent: VLM picks (heading, distance) per query, planner executes commitments, reactive safety layer owns cmd_vel. No Nav2, no SLAM, no wall-follow.
Load this every session. What's next lives in [`ROADMAP.md`](ROADMAP.md); history lives in GitHub issues + commits.

---

## Current performance

**basement_find/easy, cloud VLM (`gemma4:31b-cloud`), after Phase 2 of #9 (VLM steering + planner, wall-follow retired):**

| Seed | Target | Score | Exploration | Min dist | Collisions | Near-miss | TP/FP |
|------|--------|-------|-------------|----------|------------|-----------|--------|
| 1 | fire_extinguisher | **12.0** | 99.5% | 4.95m | **4** | 5 | 0/5 |
| 2 | pipe_sewer_floor | **9.2** | 94.9% | **1.91m** | **6** | 3 | 0/0 |

**Pre-Phase-2 baselines:** Phase 1 seed 1 score 4.0 / 11 collisions; Phase 1 seed 2 score 4.0 / 23 collisions.

**Phase 2 outcome:** score improved 4→9-12, collisions dropped 11-23 → 4-6, exploration stayed ≥ 95%. Robot now navigates via VLM-chosen (heading ∈ {left,center,right}, distance ∈ [0,2] m) commitments rather than wall-follow. Min-dist criterion (<1.5 m) **not met** — seed 1 stalls at 4.95 m, seed 2 reaches 1.91 m. Robot is slow per cycle (~6-10 s commitment, ~3 s VLM latency) so it doesn't make many approach attempts. Phase 3 (visited-cells memory) and Phase 4 (depth-based detection positioning) are the next bottlenecks.

**Target:** Complete `basement_find/easy` with success=true on ≥ 3/5 seeds. Proximity ≤ 1m + valid detection.

---

## Architecture

```
Camera+LiDAR(front) → VLM (cloud, ~1s interval, 0.5s in approach)
            ↓
  NavigationDecision: {target_visible, target_bbox, heading∈{L,C,R},
                       drive_distance_m∈[0,2], reason}
            ↓
  Planner: yaw_target = current_yaw + heading_offset (±30° / 0)
           commitment lifecycle: rotate-to-align → drive-to-distance → end
           timeouts: 10 s normal, 6 s approach; ends on distance/deadline
            ↓
  ReactiveSafetyLayer (20 Hz, owns /cmd_vel):
   - command(lin, ang) API
   - LiDAR forward veto: front<0.25 m → zero linear, slide toward open side
   - Bumper back-off: non-ground contact → 1.5 s reverse + rotate
```

**Planner constants** (`agent/planner.py`):
- `DRIVE_SPEED = 0.4 m/s`, `ROTATE_SPEED_MAX = 0.8 rad/s`
- `YAW_TOLERANCE_RAD = 8°` — switch from rotate to drive
- `DEFAULT_COMMIT_TIMEOUT_S = 10.0`, `APPROACH_COMMIT_TIMEOUT_S = 6.0`
- `APPROACH_DIST_M = 1.5` — target_visible + dist≤1.5 → approach mode
- `MAX_DISTANCE_M = 2.0` (per issue #9 design)

**VLM (Ollama, cloud `gemma4:31b-cloud`):** Decision schema
- Output: `{target_visible, target_bbox, heading, drive_distance_m, reason}`
- Prompt includes LiDAR front clearance so VLM picks shorter distances near walls
- Response parser: strict JSON → code-fenced → embedded → heuristic free-text fallback
- VLM cycle (agent_node): submit when planner idle (immediate replan) OR in approach mode at ≥60% commit progress and `vlm_interval_s` elapsed (1.0 default, 0.5 approach)

**Detection publishing** (when target_visible=true):
- Topic: `/derpbot_0/detections` (vision_msgs/Detection2DArray)
- Content: class_id=target_object, position=robot_odom (Phase 4 will fix to depth-back-projected target position)
- Validation requires: correct type, within 1.5 m of ground truth, line-of-sight

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
- **`ReactiveSafetyLayer` owns `/cmd_vel`.** Upstream callers (planner) use `safety.command(lin, ang)`; safety publishes the filtered twist at 20 Hz from its own ROS timer. Direct publishes elsewhere would race with the timer.
- **Bumper ground-plane filter is string-match only** (`"ground_plane" in str(c.collision1/2)`). A per-contact-normal filter (`|n.z| > 0.9`) over-rejects legitimate wall hits — `normals[]` is empty or non-horizontal on real wall contacts. Mirror `metrics/collision_count.py`.
- **Forward veto at <0.25 m with auto-slide.** When forward is vetoed AND upstream angular is ~0, safety injects ±0.7 rad/s rotation toward the more open side. Without this auto-slide, robot deadlocks facing walls because the planner stays in "drive forward" until deadline (~10 s/cycle wasted).
- **Planner / wall-follow split: pick one or the other.** Pre-Phase-2 had both fighting (bumper back-off vs wall-follow re-commanding forward → 11→23 collisions on seed 2). Phase 2 retires wall-follow; only the planner drives, safety filters.
- **Planner uses absolute yaw_target = current_yaw + heading_offset.** Cumulative VLM commits in one direction accumulate yaw absolutely — robot can spin past 180° if VLM repeatedly says "left".
- **Tried but reverted in Phase 2:** post-back-off "clearing rotation" (worsened seed 1 to 21 collisions); planner no-progress watchdog with 1 s sim window (fired prematurely against simple rotation, slowed exploration); zero auto-slide in safety (robot deadlocked at walls within 80 s sim).

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