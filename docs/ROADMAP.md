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

### 1. Raise detection reliability (simplified pipeline, re-benchmark pending) · open
**#14 simplified the detect→verify→act pipeline:** removed bbox, replaced with
location text + full-image verifier + column-based depth projection + VLM-owns-distance +
bumper-only safety. The old bottlenecks (bbox inaccuracy, depth-override micro-commitments,
edge-touch FP gating, safety-geometry oscillation) are removed. Awaiting re-benchmark.
Next: sweep 5 seeds, compare with pre-#14 baseline.

### 2. Fix scan rotation overshoot · [#15](https://github.com/thaije/derpbot-vlm/issues/15)
Robot rotates ~180° per scan step instead of 60° because `_scan_accum` stays at 0°
— the main loop blocks on cloud VLM while the robot keeps spinning uncontrolled.
With geometry veto disabled, rotation is unfiltered. Needs: stop the robot between
scan steps, robust accum tracking, or yaw-target-based rotation.

### 3. ~~Fix agent hangs idle — zero VLM output~~ · [#17](https://github.com/thaije/derpbot-vlm/issues/17) ✓
Fixed: HTTP timeout propagated to ollama client; scan state machine handles failed VLM submissions.

### 4. Benchmark suite on more seeds · [#3](https://github.com/thaije/derpbot-vlm/issues/3)
Done for 5 seeds with the #12 safety stack (gemma4 1/5 success). Target success=true ≥ 3/5 unmet — gated by item 1 (detection frequency). Re-run after a detection improvement lands.

---

## Later

Titles only. Expand when a task is promoted to "Next".

- **VLM tool calling** — Define `navigate()` and `report_detection()` as Ollama tools; model decides when to call each. Replaces `target_visible` boolean field with separate detection tool call. Lets VLM report detections independently of navigation actions.
- **Benchmark submission** · [#4](https://github.com/thaije/derpbot-vlm/issues/4) — `validate_submission.py` + result JSONs.
- **Medium/hard tier scenarios** · [#5](https://github.com/thaije/derpbot-vlm/issues/5) — once easy ≥ 3/5.

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