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
- GPU with ≥ 5 GB VRAM (for Phi-3-Vision local inference)
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

_(Agent not yet implemented — see roadmap for development sequence)_

Planned:
```bash
# Launch sim + agent
./scripts/start_stack.sh --scenario easy --seed 42

# Or agent only (requires running sim)
python3.12 agent/agent_node.py
```

## Configuration

All tunable parameters live in [`config/vlm_config.yaml`](config/vlm_config.yaml):
- Model selection (Claude API / Phi-3-Vision / LLaVA)
- Inference rate, max retries
- Action → velocity mappings
- Safety layer thresholds

## Safety layer

The safety layer runs independently of the VLM loop. If the forward LiDAR arc (±30°) detects an obstacle closer than 0.3 m, it publishes zero velocity on `/cmd_vel`, overriding any VLM command. The VLM is never involved in safety decisions.

## Tests

```bash
python3.12 -m pytest tests/
```