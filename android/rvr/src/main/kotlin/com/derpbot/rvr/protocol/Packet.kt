package com.derpbot.rvr.protocol

/**
 * Sphero API v2 wire protocol — the framing the RVR / RVR+ speaks over BLE.
 *
 * There is no maintained official Android SDK (the only one was archived in
 * 2019 and targets the original Sphero ball, not RVR — see issue #19). This is
 * a clean-room Kotlin port of the documented v2 protocol, cross-checked
 * byte-for-byte against the reference `spherov2.py` implementation.
 *
 * Frame layout (a built, escaped packet on the wire):
 *
 *   [SOP] [FLAGS] (TID) (SID) [DID] [CID] [SEQ] (ERR) [DATA…] [CHK] [EOP]
 *          \__________________ body, escaped & checksummed __________/
 *
 * - TID/SID are present only when the corresponding flag bit is set.
 * - ERR is present only on response packets (is_response flag).
 * - CHK = 0xFF − (sum(body) & 0xFF), computed over the unescaped body.
 * - Escaping (applied to body + CHK, never to SOP/EOP themselves):
 *     0xAB → 0xAB 0x23,  0x8D → 0xAB 0x05,  0xD8 → 0xAB 0x50
 *
 * Bytes are handled as unsigned [Int] (0..255) internally and only narrowed to
 * signed [Byte] at the wire boundary, to avoid Kotlin's signed-byte foot-guns.
 */
object Encoding {
    const val ESCAPE = 0xAB
    const val START = 0x8D
    const val END = 0xD8
    const val ESCAPED_ESCAPE = 0x23
    const val ESCAPED_START = 0x05
    const val ESCAPED_END = 0x50
}

/** Packet header flag bits (v2). */
object Flags {
    const val IS_RESPONSE = 0x01
    const val REQUESTS_RESPONSE = 0x02
    const val REQUESTS_ONLY_ERROR_RESPONSE = 0x04
    const val IS_ACTIVITY = 0x08
    const val HAS_TARGET_ID = 0x10
    const val HAS_SOURCE_ID = 0x20
    const val EXTENDED_FLAGS = 0x80
}

/**
 * The two onboard microcontrollers. Every RVR command targets one of them
 * (the RVR requires an explicit target). The target byte packs a fixed high
 * nibble of 1 with the processor id in the low nibble: `(1 << 4) | id`.
 */
enum class Processor(val id: Int) {
    /** Nordic nRF52 — BLE, power, LEDs, color sensor. */
    PRIMARY(1),

    /** ST — motors, IMU, encoders, locator, collision. */
    SECONDARY(2);

    val targetByte: Int get() = (1 shl 4) or id
}

/** Error code carried in a response packet's ERR byte. */
enum class ResponseError(val code: Int) {
    SUCCESS(0x00),
    BAD_DEVICE_ID(0x01),
    BAD_COMMAND_ID(0x02),
    NOT_YET_IMPLEMENTED(0x03),
    COMMAND_IS_RESTRICTED(0x04),
    BAD_DATA_LENGTH(0x05),
    COMMAND_FAILED(0x06),
    BAD_PARAMETER_VALUE(0x07),
    BUSY(0x08),
    BAD_TARGET_ID(0x09),
    TARGET_UNAVAILABLE(0x0a),
    UNKNOWN(0xff);

    companion object {
        fun from(code: Int): ResponseError =
            entries.firstOrNull { it.code == code } ?: UNKNOWN
    }
}

class PacketDecodingException(message: String) : Exception(message)

/**
 * A single command (or decoded response) packet.
 *
 * @param did      device (command group) id, e.g. Power=19, Drive=22
 * @param cid      command id within the group
 * @param seq      sequence number (echoed back in the matching response)
 * @param data     payload bytes (unsigned 0..255)
 * @param target   processor to target, or null for an untargeted packet
 * @param sourceId source id sent alongside a target (RVR uses 0x01)
 * @param isResponse / error  set only on decoded inbound packets
 */
data class Packet(
    val did: Int,
    val cid: Int,
    val seq: Int,
    val data: IntArray = IntArray(0),
    val target: Int? = null,
    val sourceId: Int? = if (target != null) 0x01 else null,
    val isResponse: Boolean = false,
    val error: ResponseError = ResponseError.SUCCESS,
) {
    /** Serialise to the on-the-wire byte array (with SOP/EOP and escaping). */
    fun build(): ByteArray {
        var flags = Flags.REQUESTS_RESPONSE or Flags.IS_ACTIVITY
        if (target != null) flags = flags or Flags.HAS_TARGET_ID or Flags.HAS_SOURCE_ID

        val body = ArrayList<Int>(8 + data.size)
        body.add(flags)
        if (target != null) body.add(target)
        if (sourceId != null) body.add(sourceId)
        body.add(did)
        body.add(cid)
        body.add(seq)
        for (b in data) body.add(b and 0xFF)
        body.add(checksum(body))

        val out = ArrayList<Int>(body.size + 4)
        out.add(Encoding.START)
        for (b in body) {
            when (b) {
                Encoding.ESCAPE -> { out.add(Encoding.ESCAPE); out.add(Encoding.ESCAPED_ESCAPE) }
                Encoding.START -> { out.add(Encoding.ESCAPE); out.add(Encoding.ESCAPED_START) }
                Encoding.END -> { out.add(Encoding.ESCAPE); out.add(Encoding.ESCAPED_END) }
                else -> out.add(b)
            }
        }
        out.add(Encoding.END)

        return ByteArray(out.size) { out[it].toByte() }
    }

    companion object {
        /** v2 checksum: 0xFF − (sum of body bytes, low 8 bits). */
        fun checksum(body: List<Int>): Int {
            var sum = 0
            for (b in body) sum += b and 0xFF
            return 0xFF - (sum and 0xFF)
        }

        /**
         * Parse one complete on-the-wire frame (SOP … EOP inclusive) into a
         * [Packet]. Mirrors `spherov2.py`'s parse_response: unescape, verify
         * checksum, then split header fields per the flag bits.
         */
        fun parse(frame: ByteArray): Packet {
            if (frame.size < 6) throw PacketDecodingException("frame too small: ${frame.size}")
            val ints = IntArray(frame.size) { frame[it].toInt() and 0xFF }
            if (ints.first() != Encoding.START) throw PacketDecodingException("unexpected SOP")
            if (ints.last() != Encoding.END) throw PacketDecodingException("unexpected EOP")

            val unescaped = unescape(ints.copyOfRange(1, ints.size - 1))
            if (unescaped.isEmpty()) throw PacketDecodingException("empty body")
            val chk = unescaped.removeAt(unescaped.size - 1)
            if (checksum(unescaped) != chk) throw PacketDecodingException("bad checksum")

            var i = 0
            val flags = unescaped[i++]
            val target = if (flags and Flags.HAS_TARGET_ID != 0) unescaped[i++] else null
            val source = if (flags and Flags.HAS_SOURCE_ID != 0) unescaped[i++] else null
            val did = unescaped[i++]
            val cid = unescaped[i++]
            val seq = unescaped[i++]
            val isResponse = flags and Flags.IS_RESPONSE != 0
            val err = if (isResponse) ResponseError.from(unescaped[i++]) else ResponseError.SUCCESS
            val data = IntArray(unescaped.size - i) { unescaped[i + it] }

            return Packet(did, cid, seq, data, target, source, isResponse, err)
        }

        private fun unescape(escaped: IntArray): MutableList<Int> {
            val out = ArrayList<Int>(escaped.size)
            var i = 0
            while (i < escaped.size) {
                val b = escaped[i++]
                if (b == Encoding.ESCAPE) {
                    if (i >= escaped.size) throw PacketDecodingException("dangling escape")
                    out.add(
                        when (escaped[i++]) {
                            Encoding.ESCAPED_ESCAPE -> Encoding.ESCAPE
                            Encoding.ESCAPED_START -> Encoding.START
                            Encoding.ESCAPED_END -> Encoding.END
                            else -> throw PacketDecodingException("bad escape sequence")
                        }
                    )
                } else {
                    out.add(b)
                }
            }
            return out
        }
    }

    // data class with an IntArray member needs hand-written equals/hashCode.
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is Packet) return false
        return did == other.did && cid == other.cid && seq == other.seq &&
            data.contentEquals(other.data) && target == other.target &&
            sourceId == other.sourceId && isResponse == other.isResponse &&
            error == other.error
    }

    override fun hashCode(): Int {
        var result = did
        result = 31 * result + cid
        result = 31 * result + seq
        result = 31 * result + data.contentHashCode()
        result = 31 * result + (target ?: 0)
        result = 31 * result + (sourceId ?: 0)
        result = 31 * result + isResponse.hashCode()
        result = 31 * result + error.hashCode()
        return result
    }
}

/**
 * Hands out monotonically increasing sequence numbers (wrapping 0..254, as the
 * reference does with `% 0xff`). One instance per connection; the seq lets a
 * caller match a response back to the command that produced it.
 */
class SequenceGenerator {
    private var seq = 0
    fun next(): Int {
        val current = seq
        seq = (seq + 1) % 0xFF
        return current
    }
}

/**
 * Reassembles BLE notification chunks into whole frames. BLE delivers the TX
 * characteristic in MTU-sized pieces that don't align to packet boundaries;
 * feed every chunk here and it invokes [onPacket] once per complete frame
 * (delimited by the EOP byte).
 */
class PacketCollector(private val onPacket: (Packet) -> Unit) {
    private val buffer = ArrayList<Int>()

    fun feed(chunk: ByteArray) {
        for (b in chunk) {
            val v = b.toInt() and 0xFF
            buffer.add(v)
            if (v == Encoding.END) {
                val frame = ByteArray(buffer.size) { buffer[it].toByte() }
                buffer.clear()
                if (frame.size >= 6) {
                    runCatching { Packet.parse(frame) }.onSuccess(onPacket)
                }
            }
        }
    }
}
