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

### 1. Close the TP gap (no model has produced a true positive yet) · open
After the #11 VLM benchmark, the new default `qwen3-vl:235b-cloud` scores 16.0/16.0 with verifier on — but zero TPs. Scores come entirely from navigation + safety + exploration. The position-accuracy bottleneck identified in #9 is still open. Concrete levers from earlier ideation, now ranked by expected ROI:
- **Approach-then-recant at close range** (#9 idea 2): when the planner enters approach mode at < 1 m from a candidate, fire a fresh full-FOV VLM query — "you came here because you thought you saw X; now that you're close, confirm or recant". Uses existing approach state; addresses the case the verifier can't fix (visual lookalikes that the verifier confirms but are not the actual target instance).
- **Depth-pattern consistency on bbox**: pipe = horizontal depth band; fire extinguisher = vertical narrow protrusion off a wall. Reject candidates whose depth pattern doesn't match the expected shape. (User has flagged that depth-as-primary-input is not desired; consider this as a *sanity* check only.)

### 2. Benchmark suite on more seeds (3–5) · [#3](https://github.com/thaije/derpbot-vlm/issues/3)
Validate on basement_find/easy seeds 1–5 with the new qwen3-vl default. Target: success=true ≥ 3/5. Blocked by item 1.

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