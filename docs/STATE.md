# STATE — derpbot-vlm

VLM-steered agent: VLM picks (heading, distance) per query, planner executes commitments, reactive safety layer owns cmd_vel. No Nav2, no SLAM, no wall-follow.
Load this every session. What's next lives in [`ROADMAP.md`](ROADMAP.md); history lives in GitHub issues + commits.

---

## Current performance

**basement_find/easy, cloud VLM (`gemma4:31b-cloud`), after the detection-rate iteration (image resolution + bbox-convention fix + suppressed FP fallback):**

| Seed | Target | Score | Exploration | Min dist | Collisions | Detections | Notes |
|------|--------|-------|-------------|----------|------------|------------|-------|
| 1 | fire_extinguisher | 6.8 | 96.6% | 2.37 m | 8 | 4 (all FP) | VLM saw target 4×; positions 7–13 m off GT |
| 2 | pipe_sewer_floor | 9.2 | 83.5% | 2.34 m | 9 | 3 (all FP) | **First-ever VLM detection of the floor pipe** — model called it "white cylindrical pipe lying on the ground", but world position ~12 m off GT |

**Headline change vs prior iteration:** detection rate is non-zero on both seeds; pipe_sewer_floor is now visually recognised (was 0× ever). All published detections are still FP — the model identifies the target *visual class* but locates a similar-looking shape (wall edge, floor seam) elsewhere in the scene. Robot-pose fallback (which guaranteed FPs) is removed.

**Earlier reference points (trend):** Latency-mitigation seed 1 score 4.0 / 13 col / min 3.09 m / 0 det; seed 2 score 4.0 / 21 col / min 0.494 m (proximity reached) / 0 det. Phase 3+P4 seed 1 score 13.2 / 4 col; Phase 3+P4 seed 2 score 10.4 / 6 col.

**Outstanding gap:** detection *position* accuracy. The VLM gets the visual class right but hallucinates similar shapes far from the actual target. Likely levers: depth-pattern consistency check (a pipe should have a horizontal depth band; a wall edge has a depth step), multi-frame world-position consensus across DIFFERENT robot poses (not just sightings of the same hallucinated spot), or stricter requirement that approach-distance match GT proximity radius before publishing.

**Target:** Complete `basement_find/easy` with success=true on ≥ 3/5 seeds. Proximity ≤ 1m + valid detection.

---

## Architecture

```
Camera+LiDAR(front)+VisitedCells(memory) → VLM (cloud, ~1 s, 0.5 s in approach)
            ↓
  NavigationDecision: {target_visible, target_bbox, heading∈{L,C,R},
                       drive_distance_m∈[0,2], reason}
            ↓
  Planner: yaw_target = current_yaw + heading_offset (±30° / 0)
           commitment lifecycle: rotate-to-align → drive-to-distance → end
           timeouts: 10 s normal, 6 s approach
            ↓
  ReactiveSafetyLayer (20 Hz, owns /cmd_vel):
   - command(lin, ang) API
   - LiDAR forward veto: front<0.25 m → zero linear, slide toward open side
   - Bumper back-off: non-ground contact → 1.5 s reverse + rotate

  On target_visible → _publish_detection(target, bbox):
   - depth_projection.back_project_bbox(bbox, depth, K, x, y, yaw)
     using /derpbot_0/rgbd/{depth_image,camera_info}.
   - depth_projection.stable_track_id(class, x_map, y_map) — same physical
     object across sightings shares an id.
   - Falls back to robot pose if bbox/depth/K missing.
```

**Planner constants** (`agent/planner.py`):
- `DRIVE_SPEED = 0.4 m/s`, `ROTATE_SPEED_MAX = 0.8 rad/s`
- `YAW_TOLERANCE_RAD = 8°` — switch from rotate to drive
- `DEFAULT_COMMIT_TIMEOUT_S = 10.0`, `APPROACH_COMMIT_TIMEOUT_S = 6.0`
- `APPROACH_DIST_M = 1.5` — target_visible + dist≤1.5 → approach mode
- `MAX_DISTANCE_M = 2.0` (per issue #9 design)

**agent_node loop constants** (`agent/agent_node.py`):
- `VLM_INTERVAL_DEFAULT_S = 0.3`, `VLM_INTERVAL_APPROACH_S = 0.2` — min gap between submits
- `COMMIT_REPLAN_FRACTION = 0.25` — submit next VLM query when commit 25% done so the result arrives near commit end (masks cloud latency)
- `APPROACH_STANDOFF_M = 0.5` — when bbox+depth available, overrides VLM's drive_distance_m to `clamp(depth - 0.5, 0, MAX_DISTANCE_M)`

**VLM (Ollama, cloud `gemma4:31b-cloud`):** Decision schema
- Output: `{target_visible, target_bbox, heading, drive_distance_m, reason}`
- `VLMResult` also carries `image_width/image_height` — the dimensions of the
  image actually sent (after the `MAX_IMAGE_DIM` resize), so callers can
  rescale `target_bbox` into other frames.
- Prompt includes LiDAR front clearance so VLM picks shorter distances near walls
- Target name shown twice: raw (`pipe_sewer_floor`) AND natural (`pipe sewer floor`) so VLM doesn't trip on the underscored class name
- Detection-side prompt: "scan floor, corners, walls, edges; targets may be small / low-contrast / partly hidden; MUST include bbox when target_visible=true"
- Image sent at up to **768 px** max dim, JPEG quality 90 (was 384 / q70). Camera is 640×480 so in practice no upscale happens.
- Response parser: strict JSON → code-fenced → embedded → heuristic free-text fallback
- VLM cycle (agent_node): submit when planner idle (immediate replan) OR mid-commit past 25% progress and `vlm_interval_s` elapsed (0.3 default, 0.2 approach)
- `options.temperature = 0.3`

**Detection publishing** (when target_visible=true):
- Topic: `/derpbot_0/detections` (vision_msgs/Detection2DArray)
- Bbox interpretation: **Gemma 4 0-1000 normalised coords** (`agent_node._project_target_from_bbox` rescales 0-1000 → depth dims directly; does NOT use the input-image pixel dims, which was the prior bug).
- Position: depth-back-projected from VLM bbox center via camera intrinsics
  (`agent/depth_projection.py`).
- **Suppressed when projection fails.** No more robot-pose fallback — it was
  a guaranteed FP (robot pose ≠ target pose).
- Stable id: `f"{class}_{round(x,0.5)}_{round(y,0.5)}"`. Multiple sightings of
  the same physical object share an id, so duplicates don't tank precision.
- Validation requires: correct type, within 1.5 m of ground truth, line-of-sight.

---

## Invariants (will bite again — keep in context)

### Runtime / ROS 2
- **Python interpreter: always `python3.12`.** `python3` may resolve to another venv.
- **`PYTHONPATH` must include both project root AND `/opt/ros/jazzy/lib/python3.12/site-packages`.** Otherwise rclpy import fails.
- **`use_sim_time=True` required.** rclpy node must use sim time or messages are silently dropped as future-dated.
- **Only one sim run at a time.** Hardware cannot sustain two Gazebo/ROS 2 stacks simultaneously.
- **Sim speed affects VLM frequency.** At 3x speed, 300s sim = 100s wall time, only ~15 VLM queries.

### VLM / Ollama
- **Cloud VLM detects target more often than local** (~3.5×) — fire_extinguisher reliably, pipe_sewer_floor was 0× ever until this iteration (now seen, but at wrong world position).
- **Detection position must match ground truth within 1.5 m.** Bbox + depth back-projects into the map frame; published only when projection succeeds.
- **Gemma 4 emits bbox coords in 0-1000 normalised space, regardless of input image size.** Order is `[x1, y1, x2, y2]` per our prompt. Rescale 0-1000 → depth dims directly (`agent_node._project_target_from_bbox`). Treating these as input-image pixels was a silent bug that clamped most bboxes off-image.
- **Robot-pose fallback for detection position is removed.** When bbox/depth/K is missing we suppress the detection rather than publishing at robot pose — the fallback was an FP machine (robot pose ≠ target pose).
- **`MAX_IMAGE_DIM = 768`, JPEG quality 90.** Bumped from 384/q70 for small-object recall. DerpBot camera is 640×480 so this means "no downscale, less compression" today; only matters if we ever raise camera res.
- **Mentioning the coord system in the prompt ("Gemma convention", "0-1000 normalized") suppressed detections to zero** on a 300 s run (tried 2026-05-25). Keep the prompt domain-language only; let the model use its native scale.
- **`temperature=0.1` is too conservative for VLM-driven exploration.** Seed 2 trial: 3.7 m traveled, 10 queries in 300 s, 0 detections. Keep at 0.3.
- **Line-of-sight required.** Detection through walls counts as FP_LOS, not TP.
- **Cloud models may return free text instead of JSON.** Parser handles strict / fenced / embedded / heuristic via `_parse_vlm_response`.
- **`ollama signin` required before cloud models.** Run once; auth persists.
- **Detection currently FP-biased.** The VLM correctly identifies the target's visual class but localises a similar-looking shape elsewhere (wall edge, floor seam) in the scene. Multi-frame voting on the *same hallucinated spot* doesn't help — observed three sightings collapsing to the same wrong world position.

### Safety / Navigation
- **`ReactiveSafetyLayer` owns `/cmd_vel`.** Upstream callers (planner) use `safety.command(lin, ang)`; safety publishes the filtered twist at 20 Hz from its own ROS timer. Direct publishes elsewhere would race with the timer.
- **Bumper ground-plane filter is string-match only** (`"ground_plane" in str(c.collision1/2)`). A per-contact-normal filter (`|n.z| > 0.9`) over-rejects legitimate wall hits — `normals[]` is empty or non-horizontal on real wall contacts. Mirror `metrics/collision_count.py`.
- **Forward veto at <0.25 m with auto-slide.** When forward is vetoed AND upstream angular is ~0, safety injects ±0.7 rad/s rotation toward the more open side. Without this auto-slide, robot deadlocks facing walls because the planner stays in "drive forward" until deadline (~10 s/cycle wasted).
- **Planner / wall-follow split: pick one or the other.** Pre-Phase-2 had both fighting (bumper back-off vs wall-follow re-commanding forward → 11→23 collisions on seed 2). Phase 2 retires wall-follow; only the planner drives, safety filters.
- **Planner uses absolute yaw_target = current_yaw + heading_offset.** Cumulative VLM commits in one direction accumulate yaw absolutely — robot can spin past 180° if VLM repeatedly says "left".
- **Tried but reverted in Phase 2:** post-back-off "clearing rotation" (worsened seed 1 to 21 collisions); planner no-progress watchdog with 1 s sim window (fired prematurely against simple rotation, slowed exploration); zero auto-slide in safety (robot deadlocked at walls within 80 s sim).
- **Tried but reverted in latency-mitigation iteration:** graduated linear slowdown scaling `desired_lin` by front clearance between `[min_range_m, 0.6 m]`. Cut total motion, robot spent longer in the high-risk band, seed 2 regressed from min 0.49 m to 2.34 m with 34 near-misses.

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