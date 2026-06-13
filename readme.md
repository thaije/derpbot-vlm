# derpbot-vlm

VLM-steered robot agent for the [Autonomous Robotics Simulation Testbed (ARST)](https://github.com/thaije/robot-sandbox). Uses a vision-language model to interpret the camera feed and produce velocity commands. No Nav2, no SLAM, no frontier explorer — the intelligence lives in the model.

Architecture: Camera+LiDAR → VLM → planner → cmd_vel. ReactiveSafetyLayer (bumper back-off) owns cmd_vel.

State: [`docs/STATE.md`](docs/STATE.md) · Roadmap: [`docs/ROADMAP.md`](docs/ROADMAP.md) · Tracker: GitHub issues

## Architecture

```
Camera + LiDAR(front) + VisitedCells(memory) → VLM (cloud)
                                                     ↓
  NavigationDecision: {target_visible, target_location, heading, drive_distance_m, reason}
                                                     ↓
  Planner: commitment lifecycle (rotate → drive → replan)
                                                     ↓
  ReactiveSafetyLayer (20 Hz, owns /cmd_vel): bumper back-off only; passthrough if --no-safety
```

Each VLM tick: system prompt = mission brief + LiDAR clearance + memory rays; user prompt = camera image.
Output: `{target_visible, target_location, heading, drive_distance_m, reason}`.
On `target_visible=true` + location: skeptical verifier call on full image before publishing detection.
Active scan: step-stop-shoot rotation sweep when no detection for 30 s.

## Prerequisites

- Ubuntu 24.04, ROS 2 Jazzy
- GPU with ≥ 5 GB VRAM (for SmolVLM-Instruct local inference)
- **Python 3.12** — `python3` may resolve to a different interpreter
- **uv** — `pip install uv` or [docs.astral.sh/uv](https://docs.astral.sh/uv)

```bash
# ROS packages (if running with sim)
sudo apt install ros-jazzy-ros-gz* ros-jazzy-tf2-ros

# Python venv
uv venv
source .venv/bin/activate
uv pip install -r requirements.txt

# Required: ROS 2 Python packages on sys.path
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH"
```

## Run

```bash
# Launch sim + agent
./scripts/start_stack.sh config/scenarios/basement_find/easy.yaml --seed 42 --headless

# Or agent only (requires running sim)
PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH" python3.12 -m agent.agent_node
```

## Debug harness

An interactive tool to see *why* the VLM does or doesn't detect a target: drive
the robot manually to any viewpoint and run the **exact production VLM prompts**
on the live camera, with the full prompt, raw model response, parsed decision,
verifier verdict and world projection printed (and saved to a transcript).

```bash
# 1. Start a sim (separate terminal)
cd ~/Projects/robot-sandbox && ./scripts/run_scenario.sh \
    config/scenarios/basement_find/easy.yaml --headless --seed 1

# 2. (optional) Watch the camera
ros2 run rqt_image_view rqt_image_view        # then pick /derpbot_0/rgbd/image

# 3. Run the harness (this terminal needs keyboard focus)
python3.12 -m agent.debug_node --config config/vlm_config_cloud.yaml
```

Controls (also printed at startup and on `?`):

| Key | Action |
|-----|--------|
| `w` `a` `s` `d` | drive forward / left / reverse / right (sticky) |
| `space` | stop |
| `v` | run one VLM query **+ verifier** on the current frame (full I/O) |
| `e` | toggle automatic periodic queries (observe-only) |
| `p` | toggle publishing confirmed detections |
| `f` | toggle the safety layer on/off |
| `q` | quit |

Frames and a full transcript are written to `--out-dir` (default
`.`). When a detection has a bounding box, an annotated frame with the
bbox and label drawn on it is saved as `frame_NNNN_bbox.png` alongside
the raw `frame_NNNN.png`. Useful flags: `--no-safety` (raw control, no
collision filtering), `--target <name>` (skip the mission server). See
`--help` for all.

The production agent also supports saving annotated frames during
autonomous runs:

```bash
python3.12 -m agent.agent_node --config config/vlm_config_cloud.yaml --save-frames ./debug_frames
```

## Configuration

All tunable parameters live in [`config/vlm_config.yaml`](config/vlm_config.yaml):
- Model selection (default: `gemma4:31b-cloud`)
- Inference rate, max retries, VLM interval
- Planner speeds and commitment timeouts
- Safety layer thresholds

## Safety layer

The safety layer runs at 20 Hz and owns `/cmd_vel`. On bumper contact it backs off (1.5 s capped reverse + unconditional turn). Geometry veto is disabled by default — the VLM sees LiDAR clearance in the prompt and picks its own distances. Use `--no-safety` for passthrough mode (debug only).

## Tests

```bash
python3.12 -m pytest tests/
```