"""Deterministic safety-layer stress driver (#12 verification harness).

NOT a VLM agent — a reproducible safety regression test. It feeds the
ReactiveSafetyLayer a fixed command stream (no VLM, no perception) and lets
the safety layer be the only thing avoiding collisions while the robot is
relentlessly driven around the basement. Because the input is deterministic
the trajectory is reproducible, so collision deltas are trustworthy — unlike
stochastic VLM runs whose run-to-run variance dwarfs any safety change.

Patterns:
  continuous  forward + a deterministic oscillating turn (smooth driving)
  phased      drive-straight then pure in-place rotate, mimicking the
              planner's discrete commits (exercises the wedge / rotation veto)

A correct safety layer keeps collision_count at 0 for both patterns across
seeds. See docs/STATE.md "How to run".

Usage (sim must be running first):
  .venv/bin/python3.12 scripts/safety_stress.py \
      --config config/vlm_config_cloud.yaml --pattern phased
"""
import argparse
import math
import os
import time

import yaml


def load_config(path):
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    with open(os.path.join(root, path)) as f:
        return yaml.safe_load(f)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config/vlm_config_cloud.yaml")
    ap.add_argument("--duration", type=float, default=300.0)
    ap.add_argument("--pattern", choices=["continuous", "phased"], default="continuous")
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from agent.safety_layer import ReactiveSafetyLayer

    config = load_config(args.config)

    rclpy.init(args=[])
    node = Node("stress_driver", parameter_overrides=[Parameter("use_sim_time", value=True)])
    safety = ReactiveSafetyLayer(node, config)

    start = {"t": None}

    def sim_now():
        return node.get_clock().now().nanoseconds / 1e9

    def drive_tick():
        now = sim_now()
        if start["t"] is None:
            start["t"] = now
            ready = os.environ.get("DERPBOT_READY_FLAG", "/tmp/derpbot_agent_ready")
            open(ready, "w").close()
        elapsed = now - start["t"]
        if elapsed > args.duration:
            safety.stop()
            return
        if args.pattern == "continuous":
            # Drunk lawnmower: full forward + deterministic oscillating heading.
            # Forward and turn always commanded together, so the auto-slide can
            # always act — exercises the geometry cap, not the wedge.
            lin = 0.5
            ang = 0.8 * math.sin(elapsed / 2.7) + 0.3 * math.sin(elapsed / 1.1)
        else:
            # Phased: mimic the planner's discrete rotate-THEN-drive commits.
            # During the pure-rotation phase lin=0, so if the robot is pressed
            # against a wall it can wedge (rotation vetoed, no forward for
            # auto-slide to redirect) — reproduces the gemma4 stuck episodes.
            phase = elapsed % 7.0
            if phase < 4.0:
                lin, ang = 0.5, 0.0          # drive straight (into walls)
            else:
                lin, ang = 0.0, 0.8          # pure in-place rotation
        safety.command(lin, ang)

    node.create_timer(0.1, drive_tick)
    print("[stress] driving; safety layer owns /cmd_vel", flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        safety.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
