# STATE — derpbot-vlm

VLM-steered agent: VLM picks (heading, distance, location) per query, planner executes commitments, reactive safety layer owns cmd_vel (bumper back-off only; geometry veto disabled #14). Each candidate detection passes through a skeptical second VLM call (verifier on the full image + location text) before it is published. No Nav2, no SLAM, no wall-follow.
Load this every session. What's next lives in [`ROADMAP.md`](ROADMAP.md); history lives in GitHub issues + commits.

---

## Current performance

**Default model `gemma4:31b-cloud`, speed 1.** The #14 simplification (location
text + full-image verifier + VLM-owns-distance + bumper-only safety; bbox /
depth-override / edge-guard / trust-range / geometry-veto removed) plus the #15
executor fix gives **3/5 success on `basement_find/easy`** (2026-06-13 sweep),
up from 1/5 pre-#14. Target ≥ 3/5 **met**.

| seed | target | success | t→success | tp | fp | col | score |
|---|---|---|---|---|---|---|---|
| 1 | fire_extinguisher | ✅ | 119 s | 1 | 4 | 0 | 81.2 |
| 2 | pipe_sewer_floor | ❌ | — | 0 | 0 | 0 | 20.0 |
| 3 | drill | ✅ | 65 s | 1 | 1 | 0 | 89.9 |
| 4 | drink_can_on_box | ❌ | — | 0 | 2 | 0 | 20.0 |
| 5 | fire_extinguisher | ✅ | 191 s | 1 | 3 | 2 | 58.5 |

Two open weaknesses (→ detection-reliability work): (a) **misses on flat/small
targets** — seed 2 pipe 0/31 vis flags (blind), seed 4 can flagged 11× but only
mislocalised FPs confirmed; (b) **FP scatter** — same object projects to drifting
map positions as the robot moves (tall depth-column median tracks the wall behind
a cornered object, not the object), spawning extra track ids (10 FPs across the run).

**Seed → target** (deterministic, `rng(seed+5555).choice(pool)`): 1=fire_extinguisher,
2=pipe_sewer_floor, 3=drill, 4=drink_can_on_box, 5=fire_extinguisher.

**Target:** Complete `basement_find/easy` with success=true on ≥ 3/5 seeds
(proximity ≤ 1 m + valid detection) — **met 3/5**.

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

**De-emphasised — bumper-only safety health, not model quality.** `collision_count`, `near_miss_count` may increase without the geometry veto. Keep reporting them.

**`overall_score` is misleading for model comparison.** The benchmark scorecard weights collisions and exploration heavily, so a do-nothing-safely model can outscore a do-the-mission model. Use the metrics table above instead.

---

## Architecture

```
Camera+LiDAR(front)+VisitedCells(memory) → VLM (cloud, ~1 s, 0.5 s in approach)
            ↓
  NavigationDecision: {target_visible, target_location, heading∈{L,C,R},
                       drive_distance_m∈[0,2], reason}
            ↓
  Planner: yaw_target = current_yaw + heading_offset (±30° / bearing from location)
           commitment lifecycle: rotate-to-align → drive-to-distance → end
           timeouts: 10 s normal, 6 s approach
            ↓
  ReactiveSafetyLayer (20 Hz, owns /cmd_vel):
   - Bumper back-off ONLY (#14): on contact → 1.5 s capped-reverse + turn
   - Geometry veto DISABLED (config: safety.geometry_veto: false)
   - Passthrough mode for debug harness (--no-safety)

  On target_visible + location:
   - _verify_detection(location, target):
       full camera image + location text →
       VLMClient.verify_candidate(full_image, target, location) → skeptical 2nd call →
       confirmed ? proceed : demote target_visible, no publish
   - If confirmed → _publish_detection(target, location):
       depth_projection.back_project_from_location(location, depth, K, x, y, yaw)
       using /derpbot_0/rgbd/{depth_image,camera_info};
       suppressed when projection fails;
       stable_track_id(class, x_map, y_map) so repeat sightings share an id.
```

**Planner constants** (`agent/planner.py`):
- `DRIVE_SPEED = 0.4 m/s`, `ROTATE_SPEED_MAX = 0.8 rad/s`
- `YAW_TOLERANCE_RAD = 8°` — switch from rotate to drive
- `DEFAULT_COMMIT_TIMEOUT_S = 10.0`, `APPROACH_COMMIT_TIMEOUT_S = 6.0`
- `APPROACH_DIST_M = 1.5` — target_visible + dist≤1.5 → approach mode
- `MAX_DISTANCE_M = 2.0`

**agent_node loop constants** (`agent/agent_node.py`):
- `VLM_INTERVAL_DEFAULT_S = 0.3`, `VLM_INTERVAL_APPROACH_S = 0.2` — min gap between submits
- `COMMIT_REPLAN_FRACTION = 0.25`
- `SCAN_AFTER_NO_DETECT_S = 30.0` — scan when no detection confirmed for this many sim-seconds (replaces periodic SCAN_PERIOD_S)

**Active scan** (`agent/agent_node.py`) — step-stop-shoot rotation sweep:
- `SCAN_STEPS = 6` × `SCAN_STEP_RAD = 60°` (90° HFOV → 30° overlap); `SCAN_MIN_ROT_CLEARANCE_M = 0.20` gate
- Triggers when: not approaching, not blocked, rotation clearance OK, and no confirmed detection for ≥ `SCAN_AFTER_NO_DETECT_S`
- `SCAN_SETTLE_S = 0.4`, `SCAN_ROTATE_TIMEOUT_S = 4.0`, `SCAN_ANG_SPEED = 0.7 rad/s`

**VLM (Ollama, cloud `gemma4:31b-cloud`):** Decision schema
- Output: `{target_visible, target_location, heading, drive_distance_m, reason}`
- `target_location` ∈ {far left, left, center-left, center, center-right, right, far right} — horizontal position in the image
- Prompt includes LiDAR front clearance so VLM picks shorter distances near walls
- Target name shown twice: raw (`pipe_sewer_floor`) AND natural (`pipe sewer floor`)
- Detection prompt: "MUST fill target_location when target_visible=true"
- Image sent at up to **768 px** max dim, JPEG quality 90
- Response parser: strict JSON → code-fenced → embedded → heuristic free-text fallback
- `options.temperature = 0.3`

**Verifier** (#14, replaces #10/#3 bbox-crop approach):
- Full camera image + location text (not a bbox crop). The verifier sees the whole scene and judges the indicated location.
- Sim-aware prompt: confirms discrete object of the right form, rejects flat/repeating surfaces.
- `temperature = 0.1`

**Detection publishing** (#14):
- Location → bearing → depth column median → back-project to map frame.
- `back_project_from_location(location, depth, K, x, y, yaw)` maps location text to a bearing, reads a depth column patch at that bearing, back-projects.
- No depth-override; no APPROACH_STANDOFF_M; VLM's own `drive_distance_m` stands.
- No edge-touch logic (no bbox to touch an edge). No VERIFY_TRUST_RANGE_M — verifier always runs on full image.
- Stable id: `f"{class}_{round(x,0.5)}_{round(y,0.5)}"`.

---

## Invariants (will bite again — keep in context)

### Runtime / ROS 2
- **Python interpreter: always `python3.12`.** `python3` may resolve to another venv.
- **`PYTHONPATH` must include both project root AND `/opt/ros/jazzy/lib/python3.12/site-packages`.** Otherwise rclpy import fails.
- **`use_sim_time=True` required.** rclpy node must use sim time or messages are silently dropped as future-dated.
- **Only one sim run at a time.** Hardware cannot sustain two Gazebo/ROS 2 stacks simultaneously.
- **Sim speed affects VLM frequency.** At 3x speed, 300s sim = 100s wall time, only ~15 VLM queries.
- **QoS must match publisher reliability.** EKF odom, depth, camera_info publish RELIABLE; subscribing BEST_EFFORT silently receives nothing in FastDDS (#15). Use RELIABLE for non-image topics; image can stay BEST_EFFORT.
- **Use `SingleThreadedExecutor`, NOT `MultiThreadedExecutor` (#15).** The MTE busy-spins in `_wait_for_ready_callbacks` (entities perpetually "ready but not executable" across worker threads) → pins a core at 100%, starves the GIL, freezes the main control loop 10-20s; robot coasts on last cmd_vel (scan "spins 180°/step"). STE blocks properly between callbacks. Keep ROS callbacks cheap so one thread keeps up.
- **Image/depth callbacks store the raw msg only; convert lazily on demand (#15).** cv_bridge+PIL on every frame (~30 Hz) needlessly loads the executor; conversion is only needed ~once per VLM cycle. `_get_latest_image()` / `_project_target_from_location()` do the conversion.

### VLM / Ollama
- **Prompts + schema + inference params live in `shared/`** (`prompts/*.txt`, `vlm_schema.json`), loaded by `vlm_client.py` (both sim agent and `rvr_bridge`). ONE source of truth — edit there, never re-inline. Loaded verbatim (byte-identical).
- **Default cloud model: `gemma4:31b-cloud`**
- **`target_location` replaces `target_bbox`.** The VLM emits a semantic position string (left/center/right etc.) instead of pixel coordinates. This is mapped to a bearing fraction for depth-projection and planner heading.
- **Location bearings** are in `depth_projection.py`: far left=-0.75, left=-0.42, center-left=-0.21, center=0, center-right=0.21, right=0.42, far right=0.75 rad (positive=left/CCW).
- **Verifier uses the full image, not a bbox crop.** The `location` parameter tells the verifier where to focus. No more crop misalignment, no edge-touch gating.
- **No depth-override.** The VLM's `drive_distance_m` is used directly (it sees LiDAR clearance in the prompt). Removed `APPROACH_STANDOFF_M` which created micro-commitments (e.g. 0.23m at 0.73m depth).
- **`MAX_IMAGE_DIM = 768`, JPEG quality 90.**
- **`temperature=0.3` for detection, `temperature=0.1` for verification.**
- **Verifier failure path defaults to REJECT.**
- **Cloud models may return free text instead of JSON.** Parser handles strict / fenced / embedded / heuristic.
- **`ollama signin` required before cloud models.** Run once; auth persists.
- **Scan triggers on time-since-last-confirmed-detection**, not a fixed period. `SCAN_AFTER_NO_DETECT_S = 30.0`. A confirmed detection resets `_last_confirmed_sim`.

### Safety / Navigation
- **`ReactiveSafetyLayer` owns `/cmd_vel`.** Upstream callers (planner, teleop) use `safety.command(lin, ang)`; safety publishes at 20 Hz.
- **Geometry veto is DISABLED by default (#14).** Config key `safety.geometry_veto: false`. The VLM sees LiDAR clearance in the prompt and chooses its own distances/headers. The old directional veto caused oscillation near walls (flipping commands mid-rotation). Bumper back-off remains active.
- **Bumper back-off: reverse capped, turn unconditional.** Reverse is gated by rear clearance; the recovery turn is left unconditional. Duration 1.5 s.
- **LiDAR blind-zone is handled.** `range_min = 0.15 m`; rays < `range_min` are treated as obstacles at `range_min`.
- **Passthrough mode** (debug only): `--no-safety` skips ALL filtering including bumper.

### Real-robot (RVR+ / Android, #21 phone-as-BLE-shell)
- **Phone is a thin relay**: camera + IMU streamed over WebSocket to computer; motor commands received from computer over WebSocket. All intelligence (VLM, planner, safety) runs in Python (`rvr_bridge/`). No APK rebuild for logic changes.
- **WebSocket protocol** (`rvr_bridge/protocol.py`): JSON messages with `type` field. Phone→computer: `frame`, `imu`, `battery`, `ble_state`. Computer→phone: `capture_frame`, `drive`, `raw_motors`, `stop`, `wake`, `sleep`, `reset_yaw`, `get_battery`. Port 8765.
- **`:rvr` module unchanged**: clean-room Kotlin port of Sphero v2 BLE protocol, verified vs `spherov2.py`.
- **v2 wire protocol**: SOP `0x8D`/EOP `0xD8`/escape `0xAB`; checksum `0xFF-(sum&0xFF)`; target byte `(1<<4)|proc` (PRIMARY=`0x11`, SECONDARY=`0x12`). Power DID=19, Drive DID=22. No anti-DoS handshake.
- **BLE GATT**: Sphero v2 API service `00010001-574f-...`, single write+notify char `00010002-574f-...`.
- **IMU bump detect**: `BumpDetector` (accelerometer RMS spike over baseline) replaces sim bumper topic for real-robot safety. Threshold configurable (`--bump-threshold`).
- **WiFi deploy** (no USB cable needed after initial setup):
  1. First time over USB: `adb tcpip 5555` then `adb connect <phone-ip>:5555`. After that WiFi ADB persists across reboots.
  2. `cd android && ./deploy.sh [ws://<laptop-ip>:8765]` — builds APK, patches default server URL, installs over WiFi ADB, launches app.
  3. If `adb install` fails with signature mismatch: `adb uninstall com.derpbot.app` then redeploy.
  4. VPN caveat: NordVPN blocks inbound local traffic by default. Enable "Allow local network access" (or disable VPN) for phone↔laptop WebSocket.
- **Android SDK** at `$HOME/Android` (cmdline-tools, platform-36, build-tools-36). JDK 21 required. `ANDROID_HOME` and `PATH` set in `deploy.sh`.