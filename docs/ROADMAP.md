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

### 1. Detection reliability · [#18](https://github.com/thaije/derpbot-vlm/issues/18)
Top priority — the only remaining failure mode after #14 (3/5) and #15. Two parts:
(a) **misses on flat/small targets** (seed 2 pipe 0/31 flags; seed 4 can only mislocalised FPs);
(b) **FP scatter** — tall depth-column median tracks the wall behind a cornered object, so the
same target projects to drifted map positions → extra track ids (10 FPs across the sweep).
Done when ≥ 4/5 success and ≤ 1 FP/seed. Starting hypotheses in the issue.

### 2. Real-robot structured scenario eval · [#23](https://github.com/thaije/derpbot-vlm/issues/23)
First real-world benchmark. Two modes: **hard** (one-shot, autonomous, post-hoc scoring from
saved frames + tape) and **few-shot** (N attempts, human gives `yes`/`wrong`/`far` feedback
when robot declares arrived). 5 objects × 3 runs × 2 modes = 30 trials. Reveals whether sim
failure modes transfer to real world. Builds on #21 (closed).

---

## Later

Titles only. Expand when a task is promoted to "Next".

- **VLM tool calling** — Define `navigate()` and `report_detection()` as Ollama tools; model decides when to call each. Replaces `target_visible` boolean field with separate detection tool call. Lets VLM report detections independently of navigation actions.
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