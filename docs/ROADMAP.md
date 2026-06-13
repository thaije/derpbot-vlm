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

### 1. Validate simplified pipeline · [#14](https://github.com/thaije/derpbot-vlm/issues/14)
Implementation done (location text + full-image verifier + column depth projection +
VLM-owns-distance + bumper-only safety; bbox/depth-override/edge-guard/geometry-veto removed).
Scan-rotation control loop fixed (#15). Re-benchmark in progress: sweep 5 seeds vs pre-#14
baseline. Known gap from #15 spot-checks: detection misses on flat/floor targets
(seed 2 pipe_sewer_floor: 0 detections) and false positives (seed 1: fp=4) — quantify and feed item 2.

### 2. Raise detection reliability · open
Detector misses low-contrast / flat / floor targets and emits false positives. Gated on the
#14 benchmark to identify the dominant failure mode (miss vs FP, which classes). Then iterate
on prompt / verifier / projection. Spin out a dedicated issue once the benchmark lands.

### 3. Benchmark suite on more seeds · [#3](https://github.com/thaije/derpbot-vlm/issues/3)
Target success=true ≥ 3/5 seeds. Last full sweep (1/5) predates the #14 + #15 fixes. Folded
into the item 1 re-benchmark; #3 closes when ≥ 3/5 holds.

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