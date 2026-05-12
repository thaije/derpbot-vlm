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

### 1. Claude API prototype · [#1](https://github.com/thaije/derpbot-vlm/issues/1)
Get Claude API version working first — fastest to iterate, no local model setup. Agent polls `/mission`, sends camera frame, receives action JSON.

### 2. Safety layer · [#2](https://github.com/thaije/derpbot-vlm/issues/2)
LiDAR subscriber in separate thread. Min range < 0.3 m in forward arc → override cmd_vel to zero. Must run independently of VLM.

### 3. Validate on basement_find/easy · [#3](https://github.com/thaije/derpbot-vlm/issues/3)
Run seeds 1–5 on easy. Target: success=true ≥ 3/5, collision_count=0.

---

## Later

Titles only. Expand when a task is promoted to "Next".

- **Phi-3-Vision local model** — switch from Claude API to local VLM for scored runs (no API dependency).
- **Benchmark submission** — `validate_submission.py` + result JSONs.
- **Medium/hard tier scenarios** — once easy ≥ 3/5.

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