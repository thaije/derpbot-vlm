# STATE — derpbot-vlm

VLM-steered agent: VLM picks (heading, distance) per query, planner executes commitments, reactive safety layer owns cmd_vel. Each candidate detection passes through a skeptical second VLM call (verifier, #10) before it is published or before the planner enters approach mode. No Nav2, no SLAM, no wall-follow.
Load this every session. What's next lives in [`ROADMAP.md`](ROADMAP.md); history lives in GitHub issues + commits.

---

## Current performance

**Default model `gemma4:31b-cloud`, speed 1.** The #3 detection rework (sim-aware
verifier + active scan + precise approach + approach-then-verify + area-aware
edge guard; commits ef1bf72, 08b158b, ddd506b) **fixed the real bottleneck — the
verifier was rejecting the sim's low-poly target models, not detector recall.**

**Seed → target** (deterministic, `rng(seed+5555).choice(pool)`): 1=fire_extinguisher,
2=pipe_sewer_floor, 3=drill, 4=drink_can_on_box, 5=fire_extinguisher.

**Demonstrated:** full success (proximity <1 m + TP, 0 FP) on individual runs of
seed 1 (fire_extinguisher, 0.71 m, score 75.7) and seed 3 (drill, 0.80 m, score
58.3) — the first confirmations of the stylized sim targets in project history.

**Not yet reliable.** A clean 5-seed sweep (n=1) is high-variance: detection fires
only when the robot gets a close, centred, confirmed look (~30-50 % of runs/seed),
and red-cylinder distractors are occasionally confirmed as the target (FP). One
sweep before the area-edge fix landed 0/5 success + 2 FP; the area fix re-enabled
the close-range publishes the broad edge guard had been blocking. **Target ≥3/5
success is NOT met — per-run reliability is the open problem.** Re-sweep the final
code (`scripts/sweep.sh "1 2 3 4 5"`) before trusting any delta.

**Why variance dominates (the residual limits):**
- Cornered targets (fire_extinguisher, pipe) sit where the robot reaches them only
  in spots too tight to rotate (<0.4 m to walls) → can't scan there; must spot them
  from open space with line-of-sight to the corner, which is luck-of-the-path.
- The sim target is "a red/low-poly shape"; so are some distractors and wall pipes
  → the verifier confirms the wrong one sometimes (precision ceiling at low res).
- ~20-30 VLM queries per 300 s run (cloud latency) is a thin sampling of the scene.

**Target:** Complete `basement_find/easy` with success=true on ≥ 3/5 seeds
(proximity ≤ 1 m + valid detection). Next levers in `ROADMAP.md` item 1.

### Evaluation metrics (priority order — use for model comparison)

Compare models on these, in this order. Use lexicographic ranking — a model only loses on the next metric if it ties on the previous one.

| # | Metric | Source | Direction |
|---|---|---|---|
| 1 | **`success`** = `proximity_success AND target_detected` | result JSON `raw_metrics` | higher (binary) |
| 2 | **`time_to_success`** (s) | `raw_metrics.task_completion_time` if success; else 300 (= timeout) | lower |
| 3a | **`tp_count`** | `len(submission_log)` rows where `outcome == "TP"` | higher |
| 3b | **`time_to_first_tp`** (s) | min `timestamp` over TP rows in `submission_log` | lower |
| 3c | **`vlm_flag_rate`** | agent log: `vis=True` count / total VLM queries | higher = more responsive detector |
| 4 | **`route_efficiency`** | `straight_line_distance / proximity_path_length` (proximity_reached only) | higher (1.0 = perfect) |

**De-emphasised — safety-layer health, not model quality.** `collision_count`, `near_miss_count` reflect safety-cushion integrity, not VLM choice. Keep reporting them so we notice safety regressions, but they should not drive a model winner decision.

**`overall_score` is misleading for model comparison.** The benchmark scorecard weights collisions and exploration heavily, so a do-nothing-safely model can outscore a do-the-mission model. Use the metrics table above instead. Score is fine as a sanity-check single number.

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
   - geometry veto: reaction-aware linear cap + directional rotation veto
     + deadlock recovery (pivot-open / rear-gated reverse) — see Invariants
   - Bumper back-off: non-ground contact → 1.5 s capped-reverse + turn

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
- `APPROACH_STANDOFF_M = 0.5` — when a CLOSE bbox+depth is available, overrides VLM's drive_distance_m to `clamp(depth - 0.5, 0, MAX_DISTANCE_M)`
- `VERIFY_TRUST_RANGE_M = 2.0` — sightings ≤ this are verified + published (crop/projection reliable); farther ones are approach-only (no verify, no publish, see #3)

**Active scan** (`agent/agent_node.py`, #3) — step-stop-shoot rotation sweep so the camera looks every way (the planner steers toward open space, away from cornered targets):
- `SCAN_STEPS = 6` × `SCAN_STEP_RAD = 60°` (90° HFOV → 30° overlap); `SCAN_PERIOD_S = 20` cadence; `SCAN_MIN_ROT_CLEARANCE_M = 0.20` gate
- Robot STOPS, settles (`SCAN_SETTLE_S = 0.4`), shoots one frame per heading — stationary is required for a valid depth projection. Progress = actual yaw rotated (`_scan_accum`), so it cooperates with the safety rotation veto; `SCAN_ROTATE_TIMEOUT_S = 4.0` shoots anyway if wedged.
- Triggers on cadence by PREEMPTING the current commit + in-flight query (the query pipeline never leaves an idle gap). A sighting pushes `_last_scan_end_sim` forward so a scan never interrupts an approach.

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

**Detection publishing** — approach-then-verify (#3, supersedes #10's verify-everything):
- **Far sighting (depth > `VERIFY_TRUST_RANGE_M` or no depth):** do NOT verify or
  publish — the crop is a few pixels and the projection is imprecise / often
  wall-occluded. Keep `target_visible` on the VLM's own clearance-aware
  heading/distance so the robot drifts toward it and resolves it once close.
- **Close sighting (depth ≤ 2 m):** pin drive distance to the measured range,
  then run the **verifier gate (#10)** — crop the raw frame (bbox + 20 % pad,
  upscaled ≥ 224 px) → second skeptical VLM call (`verify_candidate`,
  `temperature=0.1`). `confirmed` → publish; reject → demote `target_visible`,
  resume exploration. Only close, confirmed candidates publish (accurate
  position + the verifier judges a clear crop, not a blur).
- **Verifier is SIM-AWARE (#3).** Sim targets are LOW-POLY 3D models; the old
  "demand a real extinguisher's hose/gauge" prompt rejected every one. Prompt
  now confirms a DISCRETE object of the right form (low detail expected) and
  rejects FLAT/repeating surfaces (brick walls, seams) — accepts the stylized
  target without reopening brick FPs.
- **Precise final approach (#3).** When close + target_visible, the planner
  heading offset is the exact bearing to the bbox centre (90° HFOV), not the
  ±30° bucket, so the robot can centre the target and reach < 1 m.
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
- **Default cloud model: `gemma4:31b-cloud`** (only model producing a TP/success in the post-#12 re-benchmark; see Current performance). qwen3-vl preserved at `config/vlm_config_cloud_qwen3vl.yaml`. Other tested models live under `config/vlm_config_cloud_<name>.yaml`. Pick model winners on success → tp_count, never overall_score.
- **Cloud VLM detects target more often than local** (~3.5×) — fire_extinguisher reliably, pipe_sewer_floor was 0× ever until #9 (now seen, but at wrong world position).
- **Detection position must match ground truth within 1.5 m.** Bbox + depth back-projects into the map frame; published only when projection succeeds.
- **Gemma 4 emits bbox coords in 0-1000 normalised space, regardless of input image size.** Order is `[x1, y1, x2, y2]` per our prompt. Rescale 0-1000 → depth dims directly (`agent_node._project_target_from_bbox`). Treating these as input-image pixels was a silent bug that clamped most bboxes off-image.
- **Robot-pose fallback for detection position is removed.** When bbox/depth/K is missing we suppress the detection rather than publishing at robot pose — the fallback was an FP machine (robot pose ≠ target pose).
- **`MAX_IMAGE_DIM = 768`, JPEG quality 90.** Bumped from 384/q70 for small-object recall. DerpBot camera is 640×480 so this means "no downscale, less compression" today; only matters if we ever raise camera res.
- **Mentioning the coord system in the prompt ("Gemma convention", "0-1000 normalized") suppressed detections to zero** on a 300 s run (tried 2026-05-25). Keep the prompt domain-language only; let the model use its native scale.
- **`temperature=0.1` is too conservative for VLM-driven exploration.** Seed 2 trial: 3.7 m traveled, 10 queries in 300 s, 0 detections. Keep at 0.3.
- **Line-of-sight required.** Detection through walls counts as FP_LOS, not TP.
- **Cloud models may return free text instead of JSON.** Parser handles strict / fenced / embedded / heuristic via `_parse_vlm_response`.
- **`ollama signin` required before cloud models.** Run once; auth persists.
- **The verifier MUST be sim-aware (#3).** Sim targets are LOW-POLY 3D models with no real-world detail (hose, gauge, label). A strict "looks like a real fire extinguisher" verifier rejected EVERY true target ("a stylized 3D model lacking defining characteristics") even with the robot centred at 1.4 m — this, not the detector, was the binding constraint. Prompt now confirms a *discrete object of the right overall form* and rejects *flat/repeating surfaces* (brick walls, seams). That accepts the stylized target while still rejecting red brick — the right discriminator (the basement walls ARE red brick, the original false-positive source).
- **Detection confidence scales with proximity (#3).** Both classification and depth-projection are unreliable for a few-pixel far object — so verify + publish ONLY within `VERIFY_TRUST_RANGE_M` (2 m). Far sightings are approach-only: drift toward them on the VLM's own clearance-aware heading; do NOT force-drive to the projected point (it is often behind a wall → ramming, 11 collisions on one seed). `stable_track_id` collapses the approach's repeat sightings into one detection so the early imprecise ones don't tank precision.
- **Active scan is how the robot looks at cornered targets (#3).** The planner steers toward open space, away from cornered objects, so the forward camera rarely centres them (baseline reached 0.16 m yet detected nothing). The periodic in-place sweep fixes this — but only where there's room to rotate; a robot at <0.4 m from walls (the closest approach to a cornered target) physically cannot spin, so it must spot the target from open space with line-of-sight to the corner. This is the residual reliability limit.
- **Verifier prompt asymmetry is intentional.** Detector prompt rewards aggressive scanning ("see anything that fits"); verifier prompt rewards skepticism ("default to reject, list counter-evidence"). Mentioning calibration / framing terminology to the model breaks it (see "Gemma convention" failure above); keep both prompts in plain domain language.
- **Verifier failure path defaults to REJECT.** If the verifier call errors or returns unparseable JSON, the candidate is rejected (safer for FP than letting it through). This means cloud outages will block detections entirely — accept that trade-off until / unless we run a local-VLM fallback.
- **Verifier blocks the agent main loop for one cloud round-trip per candidate** (~3 s on a healthy cloud, 30+ s on a bad day). Safety layer keeps publishing at 20 Hz from its own ROS timer during the stall, so collisions are still filtered. If verifier latency dominates a mission budget, move it to the same ThreadPoolExecutor as the detector query.

### Safety / Navigation
- **`ReactiveSafetyLayer` owns `/cmd_vel`.** Upstream callers (planner, teleop) use `safety.command(lin, ang)`; safety publishes the filtered twist at 20 Hz from its own ROS timer. Direct publishes elsewhere would race with the timer.
- **Safety is geometry-aware (#12).** Robot is modelled as a rectangle 0.30 × 0.26 m (body + wheel hubs) with cushion 0.10 m. Checks run on every published tick:
  - **Forward / rearward translation**: minimum (x − FRONT) over scan points whose |y| < SIDE + cushion. Linear speed capped so the robot stops within (clear − cushion) accounting for **reaction latency THEN braking**: `v·t_react + v²/(2a) ≤ d` → cap `√(a²t² + 2a·d) − a·t` (a = 2.0 m/s², t_react = 0.15 s = 10 Hz LiDAR + control tick). Old `√(2a·d)` assumed instant braking and overran the cushion at speed.
  - **Rotation (directional veto)**: when an obstacle is inside the cushion, rotation is allowed only in the direction that does NOT bring the perimeter closer (probe nearest point's distance to the rectangle under a small test rotation). Rotating *away* is the wedge escape and stays allowed. Binary "block all rotation when close" wedged the robot into sustained contact.
  - **Deadlock recovery**: when upstream wants to move (forward OR rotate) but every component was vetoed, pivot toward the open side if the directional veto permits, else back out via `_wedge_reverse_speed` (rear-clearance gated). Replaces the old forward-only auto-slide; covers the planner's pure-rotation align phase and teleop.
- **Bumper back-off: reverse capped, turn unconditional.** Reverse is gated by rear clearance (never reverse into a rear obstacle); the recovery turn is left unconditional — gating it on corner clearance removed the escape and froze the robot against thin divider walls (35 bumper events in one episode).
- **LiDAR blind-zone is handled.** `range_min` is 0.15 m on the DerpBot LiDAR and equals the robot's front extent. When the wall is pressed against the bumper, those rays return invalid → the wall vanishes from the scan. `_scan_cb` now treats `0 < r < range_min` as an obstacle at `range_min` so the safety check still vetoes that direction.
- **The earlier opt-in depth-camera veto was removed.** The scenario no longer contains objects below the LiDAR plane (z ≈ 0.12 m), so floor-obstacle handling isn't needed.
- **Lateral "glide" was tried for side-clips and reverted (#12).** Added a steering nudge away from obstacles in a side cushion band (SIDE..SIDE+0.08 m) while moving forward, to slide past thin walls. On qwen3-vl it did NOT reduce collisions (seed 2 flat at 5) and tripled near-misses (4→26) — the robot hugged walls without preventing the actual contacts (front-corner box hit + thin-wall clip, not parallel side-clips). Deterministic stress stayed 0, so harmless there, but no VLM benefit + less margin → reverted. The real side-clip fix is planner-level (don't re-approach the same gap).
- **Cushion 0.10 is the practical sweet spot.** 0.15 m made things worse — the bigger no-go zone made the planner thrash. Latency is now charged to the velocity cap (reaction term), so the cushion is a pure geometric margin, not a braking-lag fudge.
- **Deterministic stress proves the safety layer (#12).** `scripts/safety_stress.py` drives a fixed command stream (no VLM) through the safety layer. Both `continuous` and `phased` patterns yield **0 collisions / 0 bumper events** across seeds → collisions are impossible for smooth/teleop inputs. Use it as the safety regression test; VLM runs are too high-variance to measure safety deltas (same seed gave 8 vs 17 collisions on gemma4).
- **Residual VLM contacts are thin-divider-wall side-clips, a blind-zone structural limit.** With the directional veto the wedge-freeze is gone (qwen3-vl s2 exploration 92→99.7 %), but ~5 contacts remain: the robot side-clips thin `wall_divide_*` while maneuvering in tight quarters the VLM keeps re-approaching. A 2D *forward* veto can't catch a side-clip, and the robot's **sides (0.13 m) sit inside `range_min` (0.15 m)** so an obstacle beside the robot is invisible. User constraint: do NOT lower `range_min` (real hardware is 0.15 m). Pushing this to zero needs planner-level obstacle memory/avoidance, not reactive safety. Robot also contacts the TARGET on approach-mode overshoot — physically correct given the proximity goal.
- **Things tried and reverted in #12 first pass** (kept here so we don't repeat): depth-camera forward veto (helped 3 cells, hurt 2), LiDAR threshold 0.25→0.45 m (same), forward arc 30°→45° (made gemini s1 worse). All reverted.
- **Bumper ground-plane filter is string-match only** (`"ground_plane" in str(c.collision1/2)`). A per-contact-normal filter (`|n.z| > 0.9`) over-rejects legitimate wall hits — `normals[]` is empty or non-horizontal on real wall contacts. Mirror `metrics/collision_count.py`.
- **Agent `_collision_events` counter vs scenario `collision_count` metric give different counts.** Agent debounces by sim-time gap of 0.5 s between bumper events (resets timer each msg); scenario metric counts rising-edge events with wall-clock 0.5 s gap and CLUSTERS them. Treat the scenario metric as ground truth; agent log is for tracing back-off behaviour, not collision counting.
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

# Run with cloud VLM (default qwen3-vl:235b-cloud, requires `ollama signin`):
.venv/bin/python3.12 -m agent.agent_node --config config/vlm_config_cloud.yaml
# Other cloud models tested in #11 live alongside: vlm_config_cloud_gemma4.yaml,
# _kimik26.yaml, _mistrallarge3.yaml, _gemini3flashpreview.yaml. Disable the
# verifier with a *_noverify.yaml variant (sets verifier.enabled: false).
#   --save-frames DIR   save annotated frames (bbox overlay) every detection

# Detection-debugging harness (#13): manual teleop + live production VLM I/O.
# Drive with w/a/s/d, 'v' = single decision+verifier query w/ full prompt+raw
# response printed, 'e' = auto observe-only queries. Saves frames + transcript
# to --out-dir. Bbox-annotated frames saved as frame_NNNN_bbox.png alongside
# the raw frame. Open `ros2 run rqt_image_view rqt_image_view` for the camera.
.venv/bin/python3.12 -m agent.debug_node --config config/vlm_config_cloud.yaml
#   --no-safety  raw control (safety passthrough)
#   --target X   skip mission fetch
#   --out-dir .  where to write frames + transcript (default: current dir)

# Automated single-seed run + FULL agent-log capture + funnel summary (#3):
#   scripts/run_diag.sh <seed> [config] [speed]    # log → logs/agent_seed<N>_*.log
# Multi-seed sweep with a result table (target, prox, minD, TP/FP, exp):
#   scripts/sweep.sh "1 2 3 4 5" [config]
# NOTE: do NOT edit agent/*.py while a sweep runs — each seed re-imports the
# code at launch, so a mid-edit file crashes the next run.

# Run tests
PYTHONPATH=. .venv/bin/python3.12 -m pytest tests/ -v -p no:launch_testing

# Safety regression test (deterministic; no VLM). Start the sim as above, then:
#   .venv/bin/python3.12 scripts/safety_stress.py \
#       --config config/vlm_config_cloud.yaml --pattern phased   # or continuous
# Expect collision_count == 0 for both patterns.
```

Results: `~/Projects/robot-sandbox/results/` via `validate_submission.py`.

---

## Issue tracker

Everything with a lifecycle lives in GitHub issues, not this doc.
- **Active / next work:** [`ROADMAP.md`](ROADMAP.md) — short TOC with links.
- **Before proposing a change, check closed dead-ends:** `gh issue list --state closed --label dead-end --search <topic>`