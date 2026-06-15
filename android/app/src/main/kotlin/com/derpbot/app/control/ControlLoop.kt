package com.derpbot.app.control

import android.util.Log
import com.derpbot.app.ble.RvrBleConnection
import com.derpbot.app.camera.CameraManager
import com.derpbot.app.vlm.VlmClient
import com.derpbot.rvr.protocol.DriveFlags
import com.derpbot.rvr.protocol.RvrCommands
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlin.math.roundToLong

/**
 * Autonomous object-finding loop for the real RVR (issue #19, Step 4).
 *
 * Mirrors the sim agent's decide→commit cycle (agent/agent_node.py + planner.py)
 * but adapted to the RVR's onboard heading controller: `driveWithHeading` takes
 * an ABSOLUTE heading (relative to the resetYaw zero), so rotate-to-align is the
 * robot's job — we just track a desired heading and nudge it ±[TURN_STEP_DEG]
 * per the VLM's left/center/right.
 *
 * No LiDAR and no depth on the phone, so:
 *  - the prompt omits clearance/memory (sim agent fills those);
 *  - "arrived" is a proxy: target confirmed by the verifier AND the VLM's own
 *    drive_distance_m has dropped to ≤ [ARRIVE_DIST_M] (it sees the object fill
 *    the frame and asks for a short hop). This replaces the sim's depth check.
 *
 * Distance → duration: the RVR has no "drive N metres" primitive, so we drive at
 * [DRIVE_SPEED_BYTE] for distance / [SPEED_MPS] seconds, then stop. SPEED_MPS is
 * a rough calibration constant — tune on hardware.
 */
class ControlLoop(
    private val scope: CoroutineScope,
    private val camera: CameraManager,
    private val vlm: VlmClient,
    private val connection: RvrBleConnection,
    private val commands: RvrCommands,
    private val targetObject: String,
    private val targetDescription: String = "",
    private val onEvent: (String) -> Unit,
) {
    @Volatile private var running = false
    private var desiredHeading = 0   // degrees, absolute vs resetYaw zero

    fun start() {
        if (running) return
        running = true
        desiredHeading = 0
        connection.send(commands.resetYaw())   // zero heading to current orientation
        onEvent("Loop started — searching for \"$targetObject\"")
        scope.launch { runLoop() }
    }

    fun stop() {
        running = false
        connection.send(commands.stop())
        onEvent("Loop stopped")
    }

    private suspend fun runLoop() {
        while (running && scope.isActive) {
            val frame = camera.captureFrame()
            if (frame == null) {
                onEvent("Frame capture failed; retrying")
                delay(VLM_INTERVAL_MS)
                continue
            }

            val decision = vlm.query(frame, buildPrompt())
            if (!running) break

            // Detection path: skeptical verify before trusting a sighting.
            var confirmed = false
            if (decision.targetVisible && decision.targetLocation != null) {
                val verdict = vlm.verify(frame, targetObject, decision.targetLocation)
                confirmed = verdict.confirmed
                onEvent(
                    "VLM vis=true loc=${decision.targetLocation} dist=${"%.2f".format(decision.driveDistanceM)} " +
                        "→ verify=${if (confirmed) "CONFIRMED" else "rejected"}"
                )
                if (confirmed && decision.driveDistanceM <= ARRIVE_DIST_M) {
                    connection.send(commands.stop())
                    onEvent("✅ ARRIVED at \"$targetObject\" (confirmed, dist≈${"%.2f".format(decision.driveDistanceM)} m)")
                    running = false
                    break
                }
            } else {
                onEvent("VLM vis=false hdg=${decision.heading} dist=${"%.2f".format(decision.driveDistanceM)} | ${decision.reason.take(60)}")
            }
            if (!running) break

            // Navigation path: nudge desired heading, then drive the committed distance.
            desiredHeading = when (decision.heading) {
                "left" -> desiredHeading - TURN_STEP_DEG
                "right" -> desiredHeading + TURN_STEP_DEG
                else -> desiredHeading
            }
            executeDrive(decision.driveDistanceM)

            delay(VLM_INTERVAL_MS)
        }
        connection.send(commands.stop())
    }

    /** Drive forward [distanceM] toward [desiredHeading], then stop. distance 0 = rescan in place. */
    private suspend fun executeDrive(distanceM: Float) {
        if (distanceM <= 0f) {
            // Stop-and-rescan: still issue the heading so the RVR rotates to align.
            connection.send(commands.driveWithHeading(0, normHeading(), DriveFlags.FORWARD))
            return
        }
        val durationMs = (distanceM / SPEED_MPS * 1000f).roundToLong().coerceIn(200L, MAX_DRIVE_MS)
        connection.send(commands.driveWithHeading(DRIVE_SPEED_BYTE, normHeading(), DriveFlags.FORWARD))
        delay(durationMs)
        if (running) connection.send(commands.stop(normHeading()))
    }

    private fun normHeading(): Int = ((desiredHeading % 360) + 360) % 360

    private fun buildPrompt(): String {
        val natural = targetObject.replace('_', ' ')
        return buildString {
            append("Target: $targetObject  (natural language: \"$natural\")\n")
            if (targetDescription.isNotEmpty()) append("Description: $targetDescription\n")
            append("\nLook at the image. Decide:\n")
            append("  - Is the target visible? Scan floor, corners, walls, edges. The target may be\n")
            append("    small or low-contrast. If you see ANY object that plausibly matches, set\n")
            append("    target_visible=true and fill target_location.\n")
            append("  - Which heading (left/center/right) leads toward the target or open space?\n")
            append("  - How far to drive in that heading (0.0-2.0 m). In tight/uncertain scenes pick\n")
            append("    a short distance (≤ 0.5 m).\n")
            append("Reply JSON only.")
        }
    }

    companion object {
        private const val TAG = "ControlLoop"

        const val TURN_STEP_DEG = 30           // matches planner ±30° heading offset
        const val DRIVE_SPEED_BYTE = 64        // RVR speed 0..255; gentle bring-up speed
        const val SPEED_MPS = 0.35f            // rough m/s at DRIVE_SPEED_BYTE — CALIBRATE on hardware
        const val ARRIVE_DIST_M = 0.4f         // confirmed + VLM dist ≤ this ⇒ arrived
        const val MAX_DRIVE_MS = 4000L         // cap a single drive commitment
        const val VLM_INTERVAL_MS = 300L       // min gap between cycles (cloud VLM ~1 s anyway)
    }
}
