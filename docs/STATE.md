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
   - **Geometry veto removed (code cleanup, #14).** The `geometry_veto` config key, `set_geometry_veto()` / `is_veto_active()` methods, and ~150 lines of directional-clearance/rotation-veto/wedge-escape code were removed from `safety_layer.py` (559→407 LOC). The VLM sees LiDAR clearance in the prompt and chooses its own distances/headers. Bumper back-off remains active.
   - **Scan logic extracted to `agent/scan_controller.py`** (#14). `ScanController` holds the step-stop-shoot state machine; `agent_node.py` provides a `ScanContext` with the shared dependencies (794→675 LOC).
- **Bumper back-off: reverse capped, turn unconditional.** Reverse is gated by rear clearance; the recovery turn is left unconditional. Duration 1.5 s.
- **LiDAR blind-zone is handled.** `range_min = 0.15 m`; rays < `range_min` are treated as obstacles at `range_min`.
- **Passthrough mode** (debug only): `--no-safety` skips ALL filtering including bumper.

### Real-robot architecture (multi-backend, #25)
- **`robot_agent/` package** holds transport-agnostic domain logic: `BaseRealAgent` (VLM loop, teleop state machine, bump recovery, panel hooks), `RobotTransport` ABC (capture_frame/move_linear/rotate/teleop_step/halt/set_status/get_battery/get_pose), `DebugBus` (generic, backend-agnostic). All prompts/schema/verifier stay in `shared/` + `agent/vlm_client.py`; transports never re-inline.
- **Two transports**: `RvrTransport` (rvr_bridge/, Sphero RVR via phone BLE — speed byte + heading deg, IMU bump detect, dead-reckoned heading) and `Create3Transport` (create3_bridge/, iRobot Create 3 via ROS 2 — /cmd_vel Twist, /imu yaw, /odom path-length, /hazard_detection, /cmd_lightring LED, /cmd_audio beep).
- **`BaseRealAgent._desired_heading` is None when the transport has real heading** (Create 3 /imu). RVR keeps a dead-reckoned int counter. `_apply_heading_delta` is a no-op for Create 3 — the transport tracks yaw itself.
- **Debug bus `hello.capabilities`** drives panel UI: `wake_sleep`/`reset_yaw`/`torch` (RVR) vs `led_ring`/`audio`/`dock`/`hazard_display` (Create 3). Panel shows/hides button groups by capability. `rvr {cmd}` renamed to `robot {cmd}` (rvr kept for back-compat).
- **Panel is backend-agnostic** — one panel process, one HTML file. Backend selected at agent launch; panel adapts via `hello.backend` + `hello.capabilities`.

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
- **Closed-loop dev (no phone interaction) · `rvr_bridge/drive_test.py`:**
  - `python3.12 -m rvr_bridge.drive_test --duration 1.0` — restarts app via ADB, waits for BLE ready, drives, stops, sleeps. Zero phone touching.
  - **`adb reverse tcp:8765 tcp:8765`** tunnels phone's `127.0.0.1:8765` → laptop's `127.0.0.1:8765` via the ADB connection. Works for the app process (launched by `am start`) even though `run-as <pkg> nc` can't access the tunnel. Direct WiFi TCP from OkHttp is unreliable on some networks (`ECONNABORTED`/`SocketTimeoutException` even though ping+nc work) — the ADB reverse tunnel sidesteps the phone's WiFi TCP stack entirely.
  - **Server must bind to `::` (dual-stack), not `127.0.0.1`.** Android OkHttp connects via IPv6 `::1` through the adb reverse tunnel. A server on `127.0.0.1` (IPv4 only) won't accept the IPv6 connection. `::` accepts both.
  - **`restart_app()` must run in `run_in_executor`**, not synchronously. Blocking the event loop with subprocess calls prevents the WS server from completing the handshake → OkHttp times out.
  - **Samsung Freecess app-freezer** caches the activity moments after launch if the screen is off, killing the WebSocket before BLE reaches `ready`. **Fixed (#26):** the relay runs in a foreground service (`RvrRelayService`, type `connectedDevice`) which is freezer-exempt — the screen can turn off mid-run once BLE is connected (verified: 45s screen-off, RVR stays connected). The screen-on hacks (`max screen-off timeout`, `wm dismiss-keyguard`) were removed from `deploy.sh` and `drive_test.py.restart_app()`. **Scan caveat:** Samsung throttles *unfiltered* BLE scans with the screen off even with a foreground service running (Samsung power management overrides Android's foreground-service scan exemption), so `restart_app()` still wakes the screen (`KEYCODE_WAKEUP`) for the ~2s scan phase; the screen times out naturally after. The activity is a thin UI: dim dark bg + animated logo (`assets/logo-animated.svg` in a WebView so CSS keyframes run as-is), tap to reveal Connect/STOP/Disconnect/log, 10 s idle → fade back to logo-only. Activity holds no BLE/camera/relay state — `RvrRelayService` owns it.
  - **`server.py` `send()` uses `self._ws.state == WsState.OPEN`** (websockets v16 removed the `.closed` attribute; use `.state`).

### Real-robot (Create 3 / ROS 2, #25)
- **Create 3 firmware: ROS 2 Iron + FastDDS**, Jazzy-compatible per iRobot dev. `irobot_create_msgs` available in Jazzy apt repo (verified: LightringLeds, AudioNoteVector, HazardDetectionVector, Dock/Undock/DriveDistance/RotateAngle actions).
- **Transport**: `Create3Transport` runs an rclpy node in a background thread with `SingleThreadedExecutor` (STATE.md #15 — MTE busy-spins) and `use_sim_time=False`. Bridges rclpy↔asyncio; sensor data cached in thread-safe vars, commands published directly.
- **Topics**: `/cmd_vel` (Twist), `/cmd_lightring` (LightringLeds), `/cmd_audio` (AudioNoteVector) pub; `/imu`, `/odom`, `/battery_state`, `/hazard_detection` (HazardDetectionVector on Jazzy, not Stamped) sub. Sensor QoS = BEST_EFFORT (Create 3 sensor publishers).
- **Camera**: same Android phone in camera-only mode (`deploy.sh --camera-only`, `EXTRA_CAMERA_ONLY` intent extra). Phone skips BLE scan/connect; drive/motor/BLE commands no-op'd; `ble_state="disabled"`. Camera + IMU + phone battery still stream via PhoneRelay. Phone IMU redundant — bump detection uses `/hazard_detection`.
- **Drive**: `move_linear` monitors `/odom` path-length; `rotate` monitors `/imu` yaw (±8° tolerance, same as `agent/planner.py`); `teleop_step` publishes direct `Twist(linear.x=lin*0.4, angular.z=turn*1.5)`. No dead-reckoned heading — `has_real_heading=True`.
- **Hazard**: `/hazard_detection` → `HazardEvent(kind=bump|cliff|stall|wheel_drop|object_proximity)` → shared `_handle_bump_recovery` (halt, reverse 1.5 s, rotate 90° via /imu). Panel shows hazard kind in log.
- **LED autonomy**: green=autonomous, blue=teleop, red=bumped, yellow=searching/driving, off=idle. Panel colour picker for manual override.
- **Audio**: `/cmd_audio` note sequences — found=C-E-G ascending, bump=descending buzz, error=harsh low. Parity with RVR phone beep.
- **Run**: `python3.12 -m create3_bridge --target X --ros-domain 0 --debug-bus 8770` + `python3.12 -m panel --agent-url ws://localhost:8770 --bind 0.0.0.0:8080`.
- **`ROS_AUTOMATIC_DISCOVERY=false` required** for WiFi discovery with the Create 3. The default multicast discovery doesn't find the robot on 192.168.2.0/24; unicast discovery works reliably after ~3s. Set as an env var default in `Create3Transport._Create3Node.__init__`.
- **`/cmd_audio` QoS = RELIABLE** (Create 3 `ui_mgr` subscription is RELIABLE; mismatch silently drops messages). All sensor topics + `/cmd_lightring` = BEST_EFFORT.
- **Create 3 firmware stops accepting `/cmd_vel` on bump** — the transport's `move_linear`/`rotate` check `_hazard_pending` and abort early so the agent's bump recovery can take over.

### Run artifacts (VLM frames + decision log)
- **Per-run directory**: `runs/<YYYYMMDD_HHMMSS>/` (git-ignored). Created automatically by `BaseRealAgent.__init__` on every agent start. Contains `frames/` (VLM input JPEGs) + `decisions.jsonl` (one JSON line per event with epoch timestamp `t`).
- **`decisions.jsonl` events**: `agent_start` (target, model, run_dir), `agent_stop`, `decision` (vis, heading, turn_angle_deg, dist, loc, confirmed, reason, prompt, vlm_latency_ms, frame), `verifier` (confirmed, matches, mismatches, reason, latency_ms), `vlm_error` (stage=decision|verifier|scan), `arrived`, `hazard` (kind, magnitude), `mode` (teleop bool), `toggle` (which, value), `set_target` (old, new, description), `estop`, `manual_query`, `ble_state` (state), `phone_connect`, `phone_disconnect`, `scan_start` (reason, steps, step_deg), `scan_decision` (step, vis, turn, dist, loc, reason, vlm_latency_ms, frame), `scan_end` (reason), `forced_drive` (dist, reason).
- **`runs/_current`** is a symlink to the latest run dir. The panel proxy serves frames from `runs/_current/frames/` via `/frames/<name>` — no path update needed between runs.
- **CLI overrides**: `--run-dir <path>` sets a custom run dir; `--log-file <path>` overrides the JSONL path (default: `<run_dir>/decisions.jsonl`).
- **Console log**: `scripts/start_rvr.sh` tees agent stdout to `runs/rvr_console.log` for post-run inspection.
- **Old runs are NOT auto-pruned** — delete manually when no longer needed.

### Scan sweep + loop detection (real-robot)
- **360° scan sweep** (`BaseRealAgent._scan_sweep`): 6 × 60° step-stop-shoot. At each step: wait_standstill → capture_frame → VLM query → verifier if target visible. Breaks early on confirmed sighting. Uses `transport.rotate()` (transport-agnostic). Logged as `scan_start`/`scan_decision`/`scan_end` events.
- **Initial scan**: first autonomous loop iteration triggers a scan sweep before any normal VLM decision. Gives the VLM full environmental context.
- **Loop detection**: `_consecutive_turns` counter increments on each turn-only decision without vis=True; resets on any drive or confirmed sighting. At `LOOP_TURNS_TRIGGER=3`, triggers a scan sweep. If scan finds nothing, forces a forward drive using the last scan frame's VLM decision.
- **Post-scan forced drive**: if the scan completes without finding the target, the agent drives forward by the distance suggested by the last scan frame's VLM decision (ignoring its turn angle). This breaks the robot out of the oscillation zone.
- **Decision history in prompt**: each entry shows only the first sentence of the reason (max 50 chars), not the full truncated reason — so the VLM can parse past actions clearly.
- **Separate process** (`python3.12 -m panel`): connects to agent's debug bus (WS `:8770`) and serves browsers (dual-port: WS on `bind_port`, HTTP on `bind_port+1`). websockets v16 doesn't reliably serve HTTP (transport aborted before flush) → stdlib `http.server` in a thread handles static files; WS port is injected into `index.html` as `window.__WS_PORT__`.
- **`--debug-bus PORT`** on `rvr_bridge.__main__` and `create3_bridge.__main__` starts `DebugBus` (WS server in agent's asyncio loop). Default off — zero cost when absent. `--teleop-only` starts with autonomous loop paused (panel owns all movement); E-STOP (`manual_stop()`) zeroes velocities but does NOT exit teleop mode (only the `toggle {which:"teleop"}` command does).
- **Wire protocol**: binary WS = JPEG frames (prefix `0x01`); JSON msgs: `hello` (with `backend` + `capabilities`), `frame_meta`, `state`, `decision`, `verifier`, `imu`, `bump` (with `kind`), `ble`, `battery` (bus→panel→browser); `teleop {x,y}`, `stop`, `manual_query`, `toggle`, `set_target`, `robot {cmd}`, `led {r,g,b}` (browser→panel→bus). Teleop is normalized `[-1,1]`; transport maps to backend-specific drive. Bump detector stays armed during teleop.
- **Panel holds zero domain logic** — all prompts/schema/verifier stay in `shared/` + `agent/vlm_client.py`; adapters never re-inline. Sim backend is a follow-up (not yet built).
- **Teleop in-place turn: `driveWithHeading(speed=0, heading=...)` with the heading target incremented every tick (`agent.py` `TELEOP_TURN_DEG_PER_TICK`), not `raw_motors`.** Raw motors is open-loop (no torque compensation) — 64/255 (fine for forward rolling) wasn't enough torque to overcome pivot-turn wheel-scrub friction, so wheels spun without rotating the chassis. `driveWithHeading` is closed-loop; firmware handles torque. (Tried + reverted: toggling `set_stabilization` off so raw_motors wouldn't fight the yaw-hold loop — fixed the symptom looked like, but torque was still insufficient. Dead end for in-place turns specifically.)
- **Agent's BLE-ready wait actively polls (`GetBleStateMessage`/`get_ble_state`), not passive.** The phone only pushes `ble_state` on a transition (`onStateChange`); if BLE was already up before this agent process (re)started, no transition fires and a passive wait hangs forever. Phone responds to `get_ble_state` with `RvrBleConnection.currentState` on demand.
- **Camera feed for the panel uses CameraX `ImageAnalysis` (continuous, ~5 fps), not `ImageCapture.takePicture()`.** `takePicture()` is a full photo shutter (AE/AF settle) — ~1.1-1.25s round-trip, far short of 2 fps. `ImageAnalysis` + `STRATEGY_KEEP_ONLY_LATEST` keeps a cached latest frame; `captureFrame()` returns it instantly. Trade-off: native analysis resolution (640 long edge) is lower than the old full-photo-downscaled-to-768px.