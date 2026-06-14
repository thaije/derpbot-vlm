package com.derpbot.rvr.protocol

/**
 * Builds the handful of RVR command packets the object-finding control loop
 * needs (issue #19, Step 1 + Step 4). Device ids (DID) and command ids (CID)
 * are taken from the documented v2 API and the `spherov2.py` reference.
 *
 * Each builder pulls the next sequence number from [seq], so a [RvrCommands]
 * instance is bound to a single connection.
 */
class RvrCommands(private val seq: SequenceGenerator = SequenceGenerator()) {

    // --- Power (DID 19, targets PRIMARY) -----------------------------------

    /** Wake the robot from soft-sleep. Send this right after connecting. */
    fun wake(): Packet =
        Packet(DID_POWER, CID_WAKE, seq.next(), target = Processor.PRIMARY.targetByte)

    /** Put the robot into soft-sleep. */
    fun sleep(): Packet =
        Packet(DID_POWER, CID_SLEEP, seq.next(), target = Processor.PRIMARY.targetByte)

    /** Request battery charge as a percentage (response carries one byte). */
    fun getBatteryPercentage(): Packet =
        Packet(DID_POWER, CID_BATTERY_PERCENTAGE, seq.next(), target = Processor.PRIMARY.targetByte)

    // --- Drive (DID 22, targets SECONDARY) ---------------------------------

    /**
     * Drive at [speed] (0..255) toward an absolute [heading] in degrees
     * (0..359, relative to the yaw zeroed by [resetYaw]). Speed 0 stops while
     * holding the heading.
     *
     * Heading is sent big-endian as two bytes; [flags] selects forward/reverse
     * and turn behaviour (see [DriveFlags]).
     */
    fun driveWithHeading(speed: Int, heading: Int, flags: Int = DriveFlags.FORWARD): Packet {
        val s = speed.coerceIn(0, 255)
        val h = ((heading % 360) + 360) % 360
        val data = intArrayOf(s, (h shr 8) and 0xFF, h and 0xFF, flags and 0xFF)
        return Packet(DID_DRIVE, CID_DRIVE_WITH_HEADING, seq.next(), data, Processor.SECONDARY.targetByte)
    }

    /** Convenience: stop the robot, holding [heading]. */
    fun stop(heading: Int = 0): Packet = driveWithHeading(0, heading, DriveFlags.FORWARD)

    /**
     * Drive the two motors independently. Useful for in-place turns during the
     * "scan" behaviour. [leftMode]/[rightMode] are [RawMotorMode] values;
     * speeds are 0..255.
     */
    fun setRawMotors(leftMode: Int, leftSpeed: Int, rightMode: Int, rightSpeed: Int): Packet {
        val data = intArrayOf(
            leftMode and 0xFF, leftSpeed.coerceIn(0, 255),
            rightMode and 0xFF, rightSpeed.coerceIn(0, 255),
        )
        return Packet(DID_DRIVE, CID_RAW_MOTORS, seq.next(), data, Processor.SECONDARY.targetByte)
    }

    /** Zero the heading reference to the robot's current orientation. */
    fun resetYaw(): Packet =
        Packet(DID_DRIVE, CID_RESET_YAW, seq.next(), target = Processor.SECONDARY.targetByte)

    companion object {
        const val DID_POWER = 19
        const val DID_DRIVE = 22

        const val CID_WAKE = 13
        const val CID_SLEEP = 1
        const val CID_BATTERY_PERCENTAGE = 16

        const val CID_RAW_MOTORS = 1
        const val CID_RESET_YAW = 6
        const val CID_DRIVE_WITH_HEADING = 7
    }
}

/** Drive flag bits for [RvrCommands.driveWithHeading]. */
object DriveFlags {
    const val FORWARD = 0x00
    const val BACKWARD = 0x01
    const val TURBO = 0x02
    const val FAST_TURN = 0x04
    const val LEFT_DIRECTION = 0x08
    const val RIGHT_DIRECTION = 0x10
    const val ENABLE_DRIFT = 0x20
}

/** Per-motor mode for [RvrCommands.setRawMotors]. */
object RawMotorMode {
    const val OFF = 0
    const val FORWARD = 1
    const val REVERSE = 2
}
