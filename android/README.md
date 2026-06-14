# derpbot-rvr — RVR+ / Phone / Cloud VLM real-robot agent

Phase 1 of the real-robot port (issue [#19]): an Android app that drives a
Sphero RVR+ to find an object, steered by the same cloud VLM the sim agent uses.
No ROS, no LiDAR, no depth sensor — distance is judged by a bbox-size proxy.

## Why this isn't using "the Sphero Android SDK"

There is **no maintained official Android SDK for the RVR**. The only Sphero
Android SDK (`sphero-inc/Sphero-Android-SDK`) was archived in 2019 and targets
the original Sphero *ball*, not the RVR. Sphero officially supports RVR only via
Raspberry Pi / micro:bit / Arduino.

So the BLE layer is a **clean-room Kotlin port of the documented Sphero v2 wire
protocol**, cross-checked byte-for-byte against the reference
[`spherov2.py`](https://github.com/artificial-intelligence-class/spherov2.py).
That lives in the `:rvr` module and is fully unit-tested on a plain JVM — no
robot required to know the bytes are correct.

## Modules

| Module | What | Runnable without hardware? |
|--------|------|----------------------------|
| `:rvr` | Pure-Kotlin v2 protocol: `Packet` (build/parse/escape/checksum), `RvrCommands` (wake/drive/resetYaw/battery), `PacketCollector`. | **Yes** — `./gradlew :rvr:test` |
| `:app` | Android: `RvrBleConnection` (GATT transport), `MainActivity` (Step-1 bring-up harness). Camera + VLM + control loop land here for Steps 2-4. | No — needs phone + RVR+ |

## Build order against issue #19

- **Step 1 — BLE connect + drive: DONE (pending hardware verification).**
  `:rvr` protocol verified; `:app` `RvrBleConnection` + a button harness that
  connects, wakes, drives a 1 s burst, stops, and reads battery.
- **Step 2 — Camera (CameraX ImageAnalysis → JPEG):** TODO, in `:app`.
- **Step 3 — Cloud VLM client:** TODO. Reuse the prompt/schema from
  `agent/vlm_client.py` (Ollama cloud `gemma4:31b-cloud`); decision schema is
  `{target_visible, target_location, heading, drive_distance_m, reason}`.
- **Step 4 — Control loop (VLM → drive), bbox-size "arrived" proxy:** TODO.
- **Step 5 — Safety (IMU bump heuristic + STOP):** STOP button exists; bump TODO.
- **Step 6 — Logging:** TODO.

## Hardware bring-up checklist (first time on a real RVR+)

1. Open `android/` in Android Studio; let it generate the Gradle wrapper, or run
   `gradle wrapper` once. Set the Android SDK path in `local.properties`.
2. `./gradlew :rvr:test` — protocol unit tests should pass.
3. Install `:app` on the phone, power on the RVR+, tap **Connect**.
4. **Confirm the BLE UUIDs** in `RvrBleConnection` against `nRF Connect` if the
   characteristic isn't found — Sphero firmware revs have shipped variants
   (alternatives are noted in `spherov2.py`).
5. Tap **Wake**, then **Drive forward** — the robot should roll ~1 s and stop.

[#19]: https://github.com/thaije/derpbot-vlm/issues/19
