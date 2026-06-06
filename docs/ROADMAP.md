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

### 1. Raise detection reliability (mechanism fixed; per-run variance remains) · open
**Binding constraint was NOT detector recall — it was the verifier.** Full-log
diagnosis (`scripts/run_diag.sh`) found the skeptical verifier rejected the
simulator's LOW-POLY target models ("a stylized 3D model lacking a real
extinguisher's hose/gauge") even with the robot centred at 1.4 m. The detector
flagged the target fine. Fixes landed (commits ef1bf72, 08b158b):
- **Sim-aware verifier** — confirm a discrete object of the right overall form,
  reject flat/repeating surfaces (red brick walls = the basement's FP source).
- **Active scan** (6×60° step-stop-shoot) so the camera looks at cornered
  targets the open-space-seeking planner steers away from.
- **Approach-then-verify** — verify+publish only within 2 m (accurate crop +
  projection); far sightings are approach-only; far force-drive removed (it
  rammed wall-occluded projections → 11 collisions/seed).
- **Precise final-approach heading** from the bbox centre (90° HFOV) to reach <1 m.
- **Edge-bbox guard** — a bbox sliced by the frame edge is approach-only, killing
  the dominant close-range FP (peripheral wall clutter).

Result: first confirmations of the stylized sim targets. Full success demonstrated
on seed 1 (fire_extinguisher, 0.71 m + TP, 0 FP, score 75.7) and seed 3 (drill,
0.80 m + TP, 0 FP). **0 FP** with the edge guard. **Remaining lever = per-run
variance**: the robot only detects when it gets a close, centred, confirmed look,
which happens ~30-50 % of runs/seed — cornered targets in tight spots (robot can't
spin within ~0.4 m of walls) are reached but not always *seen*. Next:
- Improve the odds of a centred close look (scan-on-low-clearance trigger; lower
  the rotation-clearance gate; persistent navigation to a sighting's world position).
- **Depth-pattern consistency on bbox** as a *sanity* check only (free-standing
  object vs wall patch), to further harden precision.

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