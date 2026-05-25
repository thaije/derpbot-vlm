# STATE — derpbot-vlm

VLM-steered agent: VLM picks (heading, distance) per query, planner executes commitments, reactive safety layer owns cmd_vel. Each candidate detection passes through a skeptical second VLM call (verifier, #10) before it is published or before the planner enters approach mode. No Nav2, no SLAM, no wall-follow.
Load this every session. What's next lives in [`ROADMAP.md`](ROADMAP.md); history lives in GitHub issues + commits.

---

## Current performance

**basement_find/easy, cloud VLM (`gemma4:31b-cloud`), after the verifier landed (#10) — n=1 per seed on this batch:**

| Seed | Target | Score | Exploration | Min dist | Collisions | Detections / FP | Notes |
|------|--------|-------|-------------|----------|------------|-----------------|-------|
| 1 | fire_extinguisher | 9.2 | 83.0 % | 5.58 m | 6 | 0 / 0 | Cloud detector returned `target_visible=false` on every one of 8 queries — no candidate reached the verifier |
| 2 | pipe_sewer_floor | 9.2 | 98.0 % | **0.59 m** | 6 | 0 / 0 | Robot passed within 0.6 m of GT but detector still returned no candidate |

**Synthetic verifier smoke test (3 hand-crafted crops):** verifier correctly rejected stylised red bar, blank grey square, and grey horizontal rectangle against `fire_extinguisher` and `pipe_sewer_floor` prompts, listing concrete mismatching features ("rectangular shape, lack of nozzle/handle, lack of pressure gauge"; "no cylindrical shape, no pipe characteristics"). Wiring + parser + prompt all work.

**Headline change vs prior iteration:** verifier landed; FP count dropped to 0/0 across both seeds — but the detector also produced 0 candidates this run, so the FP drop is not attributable to the verifier yet. Cloud VLM was visibly slow today (13–30 s per query vs prior ~3 s), starving the runs of candidates. Re-run on a normal-latency day before drawing conclusions.

**Trend over recent iterations (n=1 per seed):**

| Iteration | Seed 1 score | Seed 2 score | Seed 1 detections | Seed 2 detections |
|---|---|---|---|---|
| Latency mitigation (0d30927) | 4.0 | 4.0 | 0 | 0 |
| Detection rate (4100be2) | 6.8 | 9.2 | 4 / 4 FP | 3 / 3 FP |
| Verifier (this) | 9.2 | 9.2 | 0 / 0 | 0 / 0 |

**Outstanding work:** evaluate verifier on a run where the detector actually produces candidates. Synthetic test shows the verifier is strict (possibly too strict on small / low-detail crops); if real-run candidates all get rejected we'll have to soften the prompt or trade strictness for recall. Until then, treat the verifier as plumbing that is provably correct end-to-end but not yet measured against real FPs.

**Target:** Complete `basement_find/easy` with success=true on ≥ 3/5 seeds. Proximity ≤ 1 m + valid detection.

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

  On target_visible + bbox:
   - _verify_detection(bbox, target):
       crop raw camera image (bbox + 20 % pad, upscale ≥ 224 px) →
       VLMClient.verify_candidate(crop, target) → skeptical 2nd call →
       confirmed ? proceed : demote target_visible=False, no publish
   - If confirmed → _publish_detection(target, bbox):
       depth_projection.back_project_bbox(bbox, depth, K, x, y, yaw)
       using /derpbot_0/rgbd/{depth_image,camera_info};
       suppressed when projection fails (no robot-pose fallback);
       stable_track_id(class, x_map, y_map) so repeat sightings share an id.
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
- **Verifier gate (#10).** Every candidate is cropped from the raw camera
  frame (bbox + 20 % padding, upscaled to ≥ 224 px on the long edge) and sent
  through a second VLM call (`VLMClient.verify_candidate`) with a skeptical
  system prompt that defaults to REJECT and requires the model to list
  matching AND mismatching features. Only `confirmed=True` candidates are
  published; rejected candidates also have `target_visible` demoted to false
  so the planner doesn't enter approach mode chasing an FP. Verifier uses
  `temperature=0.1`. Cost: +1 cloud round-trip per candidate (~3 s).
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
- **Detection currently FP-biased.** The VLM correctly identifies the target's visual class but localises a similar-looking shape elsewhere (wall edge, floor seam) in the scene. Multi-frame voting on the *same hallucinated spot* doesn't help — observed three sightings collapsing to the same wrong world position. Verifier (#10) is the current mitigation.
- **Verifier prompt asymmetry is intentional.** Detector prompt rewards aggressive scanning ("see anything that fits"); verifier prompt rewards skepticism ("default to reject, list counter-evidence"). Mentioning calibration / framing terminology to the model breaks it (see "Gemma convention" failure above); keep both prompts in plain domain language.
- **Verifier failure path defaults to REJECT.** If the verifier call errors or returns unparseable JSON, the candidate is rejected (safer for FP than letting it through). This means cloud outages will block detections entirely — accept that trade-off until / unless we run a local-VLM fallback.
- **Verifier blocks the agent main loop for one cloud round-trip per candidate** (~3 s on a healthy cloud, 30+ s on a bad day). Safety layer keeps publishing at 20 Hz from its own ROS timer during the stall, so collisions are still filtered. If verifier latency dominates a mission budget, move it to the same ThreadPoolExecutor as the detector query.

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