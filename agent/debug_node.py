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

from PIL import Image, ImageDraw, ImageFont

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
    "\n"
    "  Frames with location annotations are saved as frame_NNNN_loc.png\n"
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

    def _save_frame(self, seq: int, location=None, label=None):
        img = self._get_latest_image()
        if img is None:
            return None
        if location is not None:
            img = self._annotate_location(img, location, label)
        path = os.path.join(self._out_dir, f"frame_{seq:04d}.png")
        try:
            img.save(path)
            return path
        except Exception as e:
            logger.warning("Frame save failed: %s", e)
            return None

    def _save_annotated(self, seq: int, location: str, target_name: str,
                        status: str):
        """Save an annotated copy (raw frame + location label) alongside the
        raw frame."""
        img = self._get_latest_image()
        if img is None:
            return
        annotated = self._annotate_location(img, location, f"{target_name} ({status})")
        path = os.path.join(self._out_dir, f"frame_{seq:04d}_loc.png")
        try:
            annotated.save(path)
            logger.info("Annotated frame saved: %s", os.path.basename(path))
        except Exception as e:
            logger.warning("Annotated frame save failed: %s", e)

    @staticmethod
    def _annotate_location(img: Image.Image, location: str, label: str | None) -> Image.Image:
        """Draw a location label on a copy of the image."""
        img = img.copy()
        draw = ImageDraw.Draw(img)
        if label:
            font_size = max(14, int(min(img.size) / 25))
            try:
                font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", font_size)
            except OSError:
                font = ImageFont.load_default(size=font_size)
            draw.text((4, 4), f"[{location}] {label}", fill="lime", font=font)
        return img

    def _report_projection(self, location):
        """Print depth back-projection outcome for a location (or why it failed)."""
        proj = self._project_target_from_location(location)
        if proj is None:
            with self._depth_lock:
                have_depth = self._depth_image is not None
            with self._camera_K_lock:
                have_k = self._camera_K is not None
            logger.info(
                "PROJECTION: SUPPRESSED — no world position "
                "(depth=%s camera_info=%s location=%s)",
                "ok" if have_depth else "MISSING",
                "ok" if have_k else "MISSING", location)
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
            "PARSED DECISION: target_visible=%s heading=%s drive=%.2fm location=%s",
            result.target_visible, result.heading,
            result.drive_distance_m, result.target_location)
        logger.info("PARSED REASON: %s", result.reason)

        if result.target_visible and result.target_location:
            self._save_annotated(seq, result.target_location, target, "visible")

        if not (result.target_visible and result.target_location):
            logger.info("No location candidate → skipping verifier + projection")
            logger.info("=" * 70)
            return

        # Verifier (full image + location, same as production).
        verdict = None
        vres = self.vlm.verify_candidate(img, target, location=result.target_location, verbose=True)
        if vres is not None:
            verdict = "CONFIRM" if vres.confirmed else "REJECT"
            logger.info(
                "VERIFIER VERDICT: %s  matches=%s  mismatches=%s",
                verdict, vres.matches, vres.mismatches)

        if verdict is not None:
            self._save_annotated(seq, result.target_location, target, verdict)

        proj = self._report_projection(result.target_location)
        if self._publish_enabled and proj is not None:
            self._publish_detection(target, location=result.target_location, proj=proj)
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
                    "AUTO: vis=%s hdg=%s drive=%.2fm loc=%s | %s",
                    result.target_visible, result.heading,
                    result.drive_distance_m, result.target_location,
                    result.reason[:90])
                if result.target_visible and result.target_location:
                    self._report_projection(result.target_location)
                    seq = self._seq
                    self._seq += 1
                    self._save_frame(seq)
                    self._save_annotated(seq, result.target_location,
                                         self._target_object(), "visible (auto)")
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
    parser.add_argument("--out-dir", default=".",
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
