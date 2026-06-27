# ROADMAP — derpbot-vlm

Table of contents for the issue tracker. Each task gets a short entry here with a link to the GitHub issue where the full plan and discussion live. Do not copy issue content into this file.

Current state lives in [`STATE.md`](STATE.md). History lives in closed issues + commits.

---

## Ground rules (apply to all upcoming work)

- **No hardcoded class names, scenario-specific logic, or oracle mode.** Generic VLM-driven agent — see `CLAUDE.md`.
- **No dead-end retries.** `gh issue list --state closed --label dead-end` before proposing a change in an area with prior attempts.
- **Safety layer is non-negotiable.** VLM output never overrides LiDAR collision stop.

---

## Next


### 1. VLM tool calling · [#27](https://github.com/thaije/derpbot-vlm/issues/27)
Expose `navigate()`, `report_detection()`, `toggle_flashlight()` as Ollama tools; model decides
when to call each. Replaces `target_visible` boolean with a `report_detection` tool call, letting
the VLM report detections independently of navigation actions, and turn the phone flashlight on/off
autonomously (e.g. when the scene is too dark to see). Config-gated, incremental — existing
JSON-schema path stays as the A/B baseline. Verifier stays a separate agent-triggered call;
flashlight tool is capability-gated (torch backends only). Phase 0 spike gates the whole effort on
`gemma4:31b-cloud` reliably emitting `tool_calls` with images attached.

---

## Later

Titles only. Expand when a task is promoted to "Next".

- **Judge role: VLM arrival-gate** · [#26](https://github.com/thaije/derpbot-vlm/issues/26) — VLM Judge (reference image + candidate frame → match verdict) automates the `confirm_target` human gate in `BaseRealAgent`. Lightweight 5-run calibration, no harness. Enables unattended real-robot eval (#23).
- **Real-robot structured scenario eval** · [#23](https://github.com/thaije/derpbot-vlm/issues/23) — First real-world benchmark. One-shot + few-shot modes, 5 objects × 3 runs × 2 modes = 30 trials. Reveals whether sim failure modes transfer to real world. Builds on #21 (closed); needs a working real-robot backend (Create 3 via #25 or RVR) and the Judge (#26) for unattended few-shot.

- **Create 3 backend** · [#25](https://github.com/thaije/derpbot-vlm/issues/25)
Second real-robot transport. iRobot Create 3 (ROS 2 Iron firmware, FastDDS, Jazzy-compatible) via
`/cmd_vel` + `/imu`/`/odom`/`/hazard_detection`/`/battery_state`; same Android phone in camera-only
mode. Refactor `RvrAgent` → `BaseRealAgent` + `RobotTransport` ABC; RVR + Create 3 become transports.
One panel, backend selected at launch. Feature parity: LED ring, audio, hazard display, odom-based
heading. Prereq verification (topic inventory, `irobot_create_msgs` on Jazzy, Iron↔Jazzy DDS interop)
before code; RVR regression gate after Phase 1 refactor.
- **Detection reliability** · [#18](https://github.com/thaije/derpbot-vlm/issues/18) — misses on flat/small targets + FP scatter. Done when ≥ 4/5 success and ≤ 1 FP/seed. Starting hypotheses in the issue.
- **Benchmark submission** · [#4](https://github.com/thaije/derpbot-vlm/issues/4) — `validate_submission.py` + result JSONs.
- **Medium/hard tier scenarios** · [#5](https://github.com/thaije/derpbot-vlm/issues/5) — once easy ≥ 3/5.
- **Qwen-RobotNav eval** · [#22](https://github.com/thaije/derpbot-vlm/issues/22) — VLA foundation model (Qwen3-VL-based, 5 nav domains SOTA) as detection/nav backbone. Path A: drop-in perception swap vs `gemma4:31b-cloud`. Path B: full VLA action replacement (safety-gated). **Blocked on public weights/API release** — non-actionable until then; promote to "Next" when unblocked.

---

## Open backlog

Known issues not currently prioritised. Full details in the linked issues.

Run `gh issue list --state open --label backlog` for the live list.

---

## Workflow

- **Starting a task:** read `STATE.md`, `ROADMAP.md`, and the task's issue. Check closed dead-ends in the same area.
- **During a task:** log findings and decisions as comments on the issue, not in this doc.
- **New finding:** `gh issue create` with `bug` / `dead-end` / `backlog` label. Cross-link related issues.
- **Completing a task:** close the issue with a final comment (outcome + commit SHA). Delete the task entry from "Next" here. Update `STATE.md` only if a new *invariant* came out of it.
- **Commits:** reference the issue, e.g. `feat(agent): VLM client (#1)`. GitHub auto-links.