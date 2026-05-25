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

### 1. Boost VLM detection rate on subtle targets · [#9](https://github.com/thaije/derpbot-vlm/issues/9)
Latency-mitigation iteration landed (depth-distance commits + VLM pipelining): seed 2 now reaches min_dist 0.494 m with `proximity_reached=True` — the Phase 2 criterion is satisfied. Overall score is still 4.0 because the cloud VLM produced zero detections of `pipe_sewer_floor` across multiple runs, so no valid Detection2D ever lands. Next levers: target-aware prompt rewrites, multi-frame voting, fallback to a heavier model on borderline frames, or sharper bbox grounding requirements.

### 2. Validate on basement_find/easy · [#3](https://github.com/thaije/derpbot-vlm/issues/3)
Run seeds 1–5 on easy. Target: success=true ≥ 3/5, collision_count=0. Blocked by closing the min-dist gap above.

---

## Later

Titles only. Expand when a task is promoted to "Next".

- **VLM tool calling** — Define `navigate()` and `report_detection()` as Ollama tools; model decides when to call each. Replaces `target_visible` boolean field with separate detection tool call. Lets VLM report detections independently of navigation actions.
- **Benchmark submission** · [#4](https://github.com/thaije/derpbot-vlm/issues/4) — `validate_submission.py` + result JSONs.
- **Medium/hard tier scenarios** · [#5](https://github.com/thaije/derpbot-vlm/issues/5) — once easy ≥ 3/5.

---

## Completed

- **Local VLM prototype** · [#1](https://github.com/thaije/derpbot-vlm/issues/1) — Closed. Camera → Ollama (gemma4:e2b) → action JSON → cmd_vel. Fully local.
- **Safety layer (initial)** · [#2](https://github.com/thaije/derpbot-vlm/issues/2) — Closed. Implemented but has collision regression, see #7.
- **Cloud VLM test** · [#8](https://github.com/thaije/derpbot-vlm/issues/8) — Architecture confirmed across seeds 1-2. Cloud detects visually distinctive targets (fire_extinguisher: ~14%) but fails on ambiguous floor objects (pipe_sewer_floor: 0%). Remaining blockers: #7 (collisions) and detection positioning under #6.

---

## Open backlog

Known issues not currently prioritised. Full details in the linked issues.

_(none yet)_

Run `gh issue list --state open --label backlog` for the live list.

---

## Workflow

- **Starting a task:** read `STATE.md`, `ROADMAP.md`, and the task's issue. Check closed dead-ends in the same area.
- **During a task:** log findings and decisions as comments on the issue, not in this doc.
- **New finding:** `gh issue create` with `bug` / `dead-end` / `backlog` label. Cross-link related issues.
- **Completing a task:** close the issue with a final comment (outcome + commit SHA). Delete the task entry from "Next" here. Update `STATE.md` only if a new *invariant* came out of it.
- **Commits:** reference the issue, e.g. `feat(agent): VLM client (#1)`. GitHub auto-links.