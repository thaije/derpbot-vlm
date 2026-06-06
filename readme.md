# derpbot-vlm

VLM-steered robot agent for the [Autonomous Robotics Simulation Testbed (ARST)](https://github.com/thaije/robot-sandbox). Uses a vision-language model to interpret the camera feed and produce velocity commands. No Nav2, no SLAM, no frontier explorer — the intelligence lives in the model.

Architecture: Camera → VLM → action → cmd_vel. LiDAR safety layer overrides on collision risk.

State: [`docs/STATE.md`](docs/STATE.md) · Roadmap: [`docs/ROADMAP.md`](docs/ROADMAP.md) · Tracker: GitHub issues

## Architecture

```
Camera (10 Hz) → [rate limiter 1 Hz] → VLM subprocess
                                           ↓
                                 action + reasoning text
                                           ↓
                              action executor → /cmd_vel
                                           ↑
LiDAR → safety layer (stop if obstacle < 0.3 m, overrides cmd_vel)
```

Each VLM tick: system prompt = mission brief, user prompt = camera image + last 3 actions.
Output: `{"action": "forward|backward|left|right|stop", "reasoning": "...", "target_visible": bool}`.
Each action runs for 1 s, then VLM is re-queried.

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
```

## Run

```bash
# Launch sim + agent
./scripts/start_stack.sh config/scenarios/basement_find/easy.yaml --seed 42 --headless

# Or agent only (requires running sim)
python3.12 agent/agent_node.py
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
- Model selection (SmolVLM / Phi-3-Vision)
- Inference rate, max retries
- Action → velocity mappings
- Safety layer thresholds

## Safety layer

The safety layer runs independently of the VLM loop. If the forward LiDAR arc (±30°) detects an obstacle closer than 0.3 m, it publishes zero velocity on `/derpbot_0/cmd_vel`, overriding any VLM command. The VLM is never involved in safety decisions.

## Tests

```bash
python3.12 -m pytest tests/
```