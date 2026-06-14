package com.derpbot.rvr.protocol

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/**
 * Byte-level tests for the RVR protocol port.
 *
 * The `expected` vectors are CANONICAL: generated from the reference
 * `spherov2.py` implementation and cross-checked against it (the generator's
 * "ref cross-check: MATCH"). If the Kotlin port produces these exact bytes, it
 * is wire-compatible with a real RVR without needing the robot to confirm it.
 */
class PacketTest {

    private fun hex(bytes: ByteArray): String =
        bytes.joinToString(" ") { "%02X".format(it.toInt() and 0xFF) }

    private fun assertWire(expected: String, packet: Packet) {
        assertEquals(expected, hex(packet.build()))
    }

    // --- Command vectors (fresh RvrCommands → seq starts at 0) --------------

    @Test fun wake() =
        assertWire("8D 3A 11 01 13 0D 00 93 D8", RvrCommands().wake())

    @Test fun sleep() =
        assertWire("8D 3A 11 01 13 01 00 9F D8", RvrCommands().sleep())

    @Test fun batteryPercentage() =
        assertWire("8D 3A 11 01 13 10 00 90 D8", RvrCommands().getBatteryPercentage())

    @Test fun resetYaw() =
        assertWire("8D 3A 12 01 16 06 00 96 D8", RvrCommands().resetYaw())

    @Test fun driveForward() =
        assertWire("8D 3A 12 01 16 07 00 40 00 5A 00 FB D8",
            RvrCommands().driveWithHeading(speed = 64, heading = 90))

    @Test fun driveStop() =
        assertWire("8D 3A 12 01 16 07 00 00 00 00 00 95 D8",
            RvrCommands().driveWithHeading(speed = 0, heading = 0))

    @Test fun driveMaxSpeedHighHeading() =
        assertWire("8D 3A 12 01 16 07 00 FF 01 67 00 2E D8",
            RvrCommands().driveWithHeading(speed = 255, heading = 359))

    @Test fun rawMotorsEscapedChecksum() =
        // checksum lands on 0xAB and must be escaped → "AB 50" before EOP.
        assertWire("8D 3A 12 01 16 01 00 01 80 02 40 AB 50 D8",
            RvrCommands().setRawMotors(RawMotorMode.FORWARD, 128, RawMotorMode.REVERSE, 64))

    // --- Escaping of payload bytes (SOP/EOP/ESCAPE appearing in data) -------

    @Test fun payloadEscaping() {
        // data = [0xAB, 0x8D, 0xD8, 0x00] → each special byte gets escaped.
        val pkt = Packet(
            did = 22, cid = 7, seq = 0,
            data = intArrayOf(0xAB, 0x8D, 0xD8, 0x00),
            target = Processor.SECONDARY.targetByte,
        )
        assertWire("8D 3A 12 01 16 07 00 AB 23 AB 05 AB 50 00 85 D8", pkt)
    }

    // --- Checksum & helpers -------------------------------------------------

    @Test fun checksumIsOnesComplementOfLowByte() {
        // 0xFF − (sum & 0xFF). Body [0x3A,0x11,0x01,0x13,0x0D,0x00] → 0x93.
        assertEquals(0x93, Packet.checksum(listOf(0x3A, 0x11, 0x01, 0x13, 0x0D, 0x00)))
    }

    @Test fun headingIsNormalisedModulo360() {
        // 450° → 90°; build must equal the plain heading-90 vector.
        assertWire("8D 3A 12 01 16 07 00 40 00 5A 00 FB D8",
            RvrCommands().driveWithHeading(speed = 64, heading = 450))
    }

    @Test fun speedIsClampedTo255() {
        assertWire("8D 3A 12 01 16 07 00 FF 00 00 00 96 D8",
            RvrCommands().driveWithHeading(speed = 999, heading = 0))
    }

    // --- Sequence numbering -------------------------------------------------

    @Test fun sequenceIncrementsPerCommand() {
        val cmd = RvrCommands(SequenceGenerator())
        assertEquals(0, cmd.wake().seq)
        assertEquals(1, cmd.driveWithHeading(10, 0).seq)
        assertEquals(2, cmd.stop().seq)
    }

    @Test fun sequenceWrapsAt255() {
        val gen = SequenceGenerator()
        var last = 0
        repeat(0xFF) { last = gen.next() }   // pulls 0..254
        assertEquals(254, last)
        assertEquals(0, gen.next())          // wraps back to 0
    }

    // --- Round trip: build → parse ------------------------------------------

    @Test fun buildThenParseRoundTrips() {
        val original = RvrCommands().driveWithHeading(speed = 200, heading = 123)
        val parsed = Packet.parse(original.build())
        assertEquals(RvrCommands.DID_DRIVE, parsed.did)
        assertEquals(RvrCommands.CID_DRIVE_WITH_HEADING, parsed.cid)
        assertEquals(0, parsed.seq)
        assertTrue(intArrayOf(200, 0, 123, 0).contentEquals(parsed.data))
        assertEquals(Processor.SECONDARY.targetByte, parsed.target)
    }

    @Test fun parseDecodesResponseError() {
        // Hand-build a response frame: flags=is_response|has_tid|has_sid,
        // tid=0x11, sid=0x01, did=19, cid=13, seq=5, err=0x00(success).
        // Body = [0x31,0x11,0x01,0x13,0x0D,0x05,0x00]; chk = 0xFF-(sum&0xFF).
        val body = listOf(0x31, 0x11, 0x01, 0x13, 0x0D, 0x05, 0x00)
        val chk = Packet.checksum(body)
        val frame = (listOf(Encoding.START) + body + chk + Encoding.END)
            .map { it.toByte() }.toByteArray()
        val parsed = Packet.parse(frame)
        assertTrue(parsed.isResponse)
        assertEquals(ResponseError.SUCCESS, parsed.error)
        assertEquals(5, parsed.seq)
    }

    // --- Collector: chunk reassembly ----------------------------------------

    @Test fun collectorReassemblesSplitFrames() {
        val frame = RvrCommands().getBatteryPercentage().build()
        // Make it a "response" so parse accepts it: reuse a known-good response.
        val respBody = listOf(0x31, 0x11, 0x01, 0x13, 0x10, 0x07, 0x00, 0x55)
        val respChk = Packet.checksum(respBody)
        val resp = (listOf(Encoding.START) + respBody + respChk + Encoding.END)
            .map { it.toByte() }.toByteArray()

        val received = ArrayList<Packet>()
        val collector = PacketCollector { received.add(it) }
        // Deliver in awkward chunks straddling the frame boundary.
        collector.feed(resp.copyOfRange(0, 3))
        collector.feed(resp.copyOfRange(3, resp.size))
        assertEquals(1, received.size)
        assertEquals(0x55, received[0].data[0])   // battery payload byte
        // unused: ensures driveStop vector compiles against `frame`
        assertTrue(frame.isNotEmpty())
    }
}
