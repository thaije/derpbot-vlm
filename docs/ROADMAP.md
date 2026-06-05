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

### 1. Raise detection frequency (first TP landed; now 1/5 → target ≥3/5) · open
Post-#12 re-benchmark: **`gemma4:31b-cloud` is the new default** — the only model that produced a TP/success (1/5 seeds, fire_extinguisher). Every other model got 0 detections in 5 seeds despite reaching the target (qwen3-vl 0.16 m proximity). So navigation is solved; **detection/publish frequency is the bottleneck.** Levers by ROI:
- **Investigate verifier over-rejection**: most models show flag_rate > 0 (VLM says it sees the target) but 0 published detections → the verifier and/or depth-projection is dropping nearly everything. A/B verifier ON/OFF on gemma4 across seeds; check how many demotions/projection-failures occur per flagged candidate.
- **Approach-then-recant at close range** (#9 idea 2): when the planner enters approach mode < 1 m from a candidate, fire a fresh full-FOV VLM query to confirm/recant.
- **Depth-pattern consistency on bbox** as a *sanity* check only (depth-as-primary is not desired).

### 2. Benchmark suite on more seeds · [#3](https://github.com/thaije/derpbot-vlm/issues/3)
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