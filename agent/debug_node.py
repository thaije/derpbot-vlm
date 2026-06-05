"""Interactive VLM detection-debugging harness (#13).

Drive the robot manually to any vantage point and run the EXACT production VLM
prompts on the live camera, with the full prompt, raw model response, parsed
detection and world projection (or rejection reason) printed in a readable form.
Probes the detection bottleneck (#3): only gemma4 ever published a valid
detection; qwen3-vl/gemini reach the target but detect nothing — this lets you
see *why*, frame by frame.

Reuses ``AgentNode`` wholesale (ROS subscriptions, safety layer, VLM client,
verifier, bbox crop + depth projection helpers, the production decision prompt)
so what runs here is byte-identical to the live agent. Only the run loop differs:
the planner does NOT drive — you do.

Usage (sim must be running first; see docs/STATE.md "How to run"):
  source /opt/ros/jazzy/setup.bash
  export PYTHONPATH=/home/plip/Projects/derpbot-vlm:/opt/ros/jazzy/lib/python3.12/site-packages
  .venv/bin/python3.12 -m agent.debug_node --config config/vlm_config_cloud.yaml

Open a live camera view alongside it:
  ros2 run rqt_image_view rqt_image_view          # then pick /derpbot_0/rgbd/image

Keys (this terminal must have focus):
  w / s   forward / reverse        a / d   turn left / right
  space   stop                     v       manual VLM query (decision + verifier)
  e       toggle AUTO query mode   p       toggle detection publishing
  f       toggle safety filtering  ?       help          q / Ctrl-C   quit

Every manual query saves the frame + a full I/O transcript under --out-dir.
"""
import argparse
import logging
import os
import select
import signal
import sys
import termios
import time
import tty
from threading import Thread

from agent.agent_node import AgentNode, fetch_mission, load_config

logger = logging.getLogger("debug_node")

# Sticky teleop velocities (m/s, rad/s). Conservative — this is for precise
# positioning, not racing. Linear and angular are independent: press w then a
# to drive forward while turning; space zeroes both.
DRIVE_LIN = 0.3
REVERSE_LIN = 0.2
TURN_ANG = 0.6

AUTO_INTERVAL_S = 2.0  # auto-mode query cadence (wall-clock)

HELP = (
    "\n"
    "  w/s forward/reverse   a/d turn left/right   space stop\n"
    "  v   manual VLM query (decision + verifier, full I/O, saves frame)\n"
    "  e   toggle AUTO query mode (observe-only, decision only)\n"
    "  p   toggle detection publishing      f toggle safety filtering\n"
    "  ?   this help        q quit\n"
)


class DebugNode(AgentNode):
    """AgentNode with a manual teleop + VLM-inspection loop instead of the
    autonomous planner-driven loop."""

    def __init__(self, config: dict, out_dir: str, no_safety: bool,
                 publish: bool, target_override: str | None):
        super().__init__(config)
        self._out_dir = out_dir
        os.makedirs(self._out_dir, exist_ok=True)
        self._no_safety = no_safety
        self._publish_enabled = publish
        self._target_override = target_override

        self._lin = 0.0
        self._ang = 0.0
        self._auto = False
        self._last_auto_wall_s = -1e9
        self._seq = 0
        self._quit = False

    # --------------------------------------------------------------- helpers

    def _target_object(self) -> str:
        return (self._mission or {}).get("target_object", "unknown")

    def _save_frame(self, seq: int):
        img = self._get_latest_image()
        if img is None:
            return None
        path = os.path.join(self._out_dir, f"frame_{seq:04d}.png")
        try:
            img.save(path)
            return path
        except Exception as e:
            logger.warning("Frame save failed: %s", e)
            return None

    def _report_projection(self, bbox):
        """Print depth back-projection outcome for a bbox (or why it failed)."""
        proj = self._project_target_from_bbox(bbox)
        if proj is None:
            with self._depth_lock:
                have_depth = self._depth_image is not None
            with self._camera_K_lock:
                have_k = self._camera_K is not None
            logger.info(
                "PROJECTION: SUPPRESSED — no world position "
                "(depth=%s camera_info=%s bbox=%s)",
                "ok" if have_depth else "MISSING",
                "ok" if have_k else "MISSING", bbox)
            return None
        x, y, d = proj
        logger.info("PROJECTION: world=(%.2f, %.2f) depth=%.2fm", x, y, d)
        return proj

    # ------------------------------------------------------------- VLM modes

    def _manual_query(self):
        """Synchronous decision + verifier on the current frame with full I/O.

        Stops the robot first so the inspected frame matches what the VLM sees
        and we don't drift during the (~3-6 s) cloud round-trips."""
        self._lin = self._ang = 0.0
        self.safety.command(0.0, 0.0)

        img = self._get_latest_image()
        if img is None:
            logger.warning("No camera frame yet — is the sim publishing %s?",
                           self.config["ros"]["camera_topic"])
            return

        seq = self._seq
        self._seq += 1
        frame_path = self._save_frame(seq)
        target = self._target_object()
        logger.info("=" * 70)
        logger.info("MANUAL QUERY #%04d  target=%s  frame=%s",
                    seq, target, os.path.basename(frame_path or "<none>"))

        result = self.vlm.query(img, self._build_decision_prompt(), verbose=True)
        if result is None:
            logger.warning("VLM returned no result")
            return
        logger.info(
            "PARSED DECISION: target_visible=%s heading=%s drive=%.2fm bbox=%s",
            result.target_visible, result.heading,
            result.drive_distance_m, result.target_bbox)
        logger.info("PARSED REASON: %s", result.reason)

        if not (result.target_visible and result.target_bbox):
            logger.info("No bbox candidate → skipping verifier + projection")
            logger.info("=" * 70)
            return

        # Verifier (same crop + skeptical second call as production).
        crop = self._crop_candidate(result.target_bbox)
        if crop is None:
            logger.warning("VERIFIER: could not crop bbox %s", result.target_bbox)
        else:
            vres = self.vlm.verify_candidate(crop, target, verbose=True)
            if vres is not None:
                logger.info(
                    "VERIFIER VERDICT: %s  matches=%s  mismatches=%s",
                    "CONFIRM" if vres.confirmed else "REJECT",
                    vres.matches, vres.mismatches)

        proj = self._report_projection(result.target_bbox)
        if self._publish_enabled and proj is not None:
            self._publish_detection(target, result.target_bbox, proj=proj)
        elif not self._publish_enabled:
            logger.info("(publishing disabled — press 'p' to enable)")
        logger.info("=" * 70)

    def _auto_tick(self, now_wall: float):
        """Observe-only periodic decision query (no verifier, no driving),
        mirroring the live agent's VLM cadence."""
        if self._vlm_future is not None:
            if not self._vlm_future.done():
                return
            try:
                result = self._vlm_future.result()
            except Exception as e:
                logger.error("AUTO VLM error: %s", e)
                result = None
            self._vlm_future = None
            if result is not None:
                logger.info(
                    "AUTO: vis=%s hdg=%s drive=%.2fm bbox=%s | %s",
                    result.target_visible, result.heading,
                    result.drive_distance_m, result.target_bbox,
                    result.reason[:90])
                if result.target_visible and result.target_bbox:
                    self._report_projection(result.target_bbox)
                    logger.info("  (press 'v' to run the verifier on this view)")
            return

        if now_wall - self._last_auto_wall_s >= AUTO_INTERVAL_S:
            self._submit_vlm_query()
            self._last_auto_wall_s = now_wall

    # ------------------------------------------------------------ key handling

    def _handle_key(self, key: str):
        if key in ("q", "\x03"):  # q or Ctrl-C
            self._quit = True
        elif key == "w":
            self._lin = DRIVE_LIN
        elif key == "s":
            self._lin = -REVERSE_LIN
        elif key == "a":
            self._ang = TURN_ANG
        elif key == "d":
            self._ang = -TURN_ANG
        elif key == " ":
            self._lin = self._ang = 0.0
            logger.info("STOP")
        elif key == "v":
            self._manual_query()
        elif key == "e":
            self._auto = not self._auto
            self._last_auto_wall_s = -1e9
            logger.info("AUTO query mode: %s", "ON" if self._auto else "OFF")
        elif key == "p":
            self._publish_enabled = not self._publish_enabled
            logger.info("Detection publishing: %s",
                        "ON" if self._publish_enabled else "OFF")
        elif key == "f":
            self._no_safety = not self._no_safety
            self.safety.set_passthrough(self._no_safety)
        elif key == "?":
            logger.info(HELP)

    # ---------------------------------------------------------------- run loop

    def run(self):
        ros_cfg = self.config["ros"]
        if self._target_override:
            self._mission = {
                "target_object": self._target_override,
                "target_description": "",
                "time_limit_seconds": 10 ** 9,
            }
            logger.info("Target override: %s (skipping mission fetch)",
                        self._target_override)
        else:
            self._mission = fetch_mission(ros_cfg["mission_url"])

        self.vlm.start()
        logger.info("VLM client ready (model=%s)", self.vlm.model_name)

        spin_thread = Thread(target=self._spin, daemon=True)
        spin_thread.start()

        if self._no_safety:
            self.safety.set_passthrough(True)

        logger.info("Target: %s", self._target_object())
        logger.info("Transcript + frames → %s", self._out_dir)
        logger.info(HELP)

        old_attrs = None
        stdin_fd = sys.stdin.fileno()
        try:
            old_attrs = termios.tcgetattr(stdin_fd)
            tty.setcbreak(stdin_fd)
        except (termios.error, ValueError):
            logger.warning("Not a TTY — key control unavailable; auto mode only")

        try:
            while not self._quit and not self._done_event.is_set():
                key = self._getch(0.05)
                if key:
                    self._handle_key(key)

                # Teleop → safety (passthrough flag decides filtering).
                self.safety.command(self._lin, self._ang)

                # Keep the prompt's "visited cells" memory honest as we drive.
                x, y, _ = self._get_odom()
                self.memory.mark(x, y)

                if self._auto:
                    self._auto_tick(time.time())
        except KeyboardInterrupt:
            pass
        finally:
            if old_attrs is not None:
                termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_attrs)
            self.safety.set_passthrough(False)
            self.safety.stop()
            self.vlm.stop()
            import rclpy
            rclpy.shutdown()
            logger.info("Debug session ended")

    @staticmethod
    def _getch(timeout_s: float) -> str | None:
        r, _, _ = select.select([sys.stdin], [], [], timeout_s)
        if r:
            return sys.stdin.read(1)
        return None


def main(args=None):
    parser = argparse.ArgumentParser(description="DerpBot VLM debug harness (#13)")
    parser.add_argument("--config", default="config/vlm_config_cloud.yaml",
                        help="Path to VLM config (use the model you want to debug)")
    parser.add_argument("--out-dir", default="/tmp/derpbot_debug",
                        help="Where to write the transcript log + saved frames")
    parser.add_argument("--no-safety", action="store_true",
                        help="Teleop with NO collision filtering (raw control)")
    parser.add_argument("--publish", action="store_true",
                        help="Publish confirmed detections to the detection topic")
    parser.add_argument("--target", default=None,
                        help="Override target object (skips mission-server fetch)")
    cli_args = parser.parse_args(args)

    config = load_config(cli_args.config)

    os.makedirs(cli_args.out_dir, exist_ok=True)
    file_handler = logging.FileHandler(
        os.path.join(cli_args.out_dir, "debug_session.log"))
    file_handler.setFormatter(
        logging.Formatter("%(asctime)s [%(name)s] %(levelname)s: %(message)s"))
    logging.getLogger().addHandler(file_handler)

    node = DebugNode(config, cli_args.out_dir, cli_args.no_safety,
                     cli_args.publish, cli_args.target)

    def shutdown_handler(sig, frame):
        node._quit = True
        node._done_event.set()

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    node.run()


if __name__ == "__main__":
    main()
