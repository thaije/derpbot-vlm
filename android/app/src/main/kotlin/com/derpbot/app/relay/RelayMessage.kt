package com.derpbot.app.relay

import android.util.Base64
import org.json.JSONArray
import org.json.JSONObject

sealed class RelayMessage {
    abstract fun toJson(): String
}

data class FrameMessage(
    val jpegB64: String,
    val width: Int,
    val height: Int,
    val rotation: Int,
) : RelayMessage() {
    override fun toJson(): String = JSONObject().apply {
        put("type", "frame")
        put("jpeg_b64", jpegB64)
        put("width", width)
        put("height", height)
        put("rotation", rotation)
    }.toString()
}

data class ImuMessage(
    val accel: FloatArray,
    val gyro: FloatArray,
    val ts: Double,
) : RelayMessage() {
    override fun toJson(): String = JSONObject().apply {
        put("type", "imu")
        put("accel", JSONArray().apply { accel.forEach { put(it) } })
        put("gyro", JSONArray().apply { gyro.forEach { put(it) } })
        put("ts", ts)
    }.toString()
}

data class BatteryMessage(val pct: Int) : RelayMessage() {
    override fun toJson(): String = JSONObject().apply {
        put("type", "battery")
        put("pct", pct)
    }.toString()
}

data class BleStateMessage(val state: String) : RelayMessage() {
    override fun toJson(): String = JSONObject().apply {
        put("type", "ble_state")
        put("state", state)
    }.toString()
}

fun decodeCommand(raw: String): CommandMessage? {
    val obj = JSONObject(raw)
    val type = obj.optString("type") ?: return null
    return when (type) {
        "capture_frame" -> CaptureFrameCommand
        "drive" -> DriveCommand(
            speed = obj.getInt("speed"),
            heading = obj.getInt("heading"),
            flags = obj.optInt("flags", 0),
        )
        "raw_motors" -> RawMotorsCommand(
            lMode = obj.getInt("l_mode"),
            lSpeed = obj.getInt("l_speed"),
            rMode = obj.getInt("r_mode"),
            rSpeed = obj.getInt("r_speed"),
        )
        "stop" -> StopCommand(heading = obj.optInt("heading", 0))
        "wake" -> WakeCommand
        "sleep" -> SleepCommand
        "reset_yaw" -> ResetYawCommand
        "get_battery" -> GetBatteryCommand
        else -> null
    }
}

sealed class CommandMessage

object CaptureFrameCommand : CommandMessage()
data class DriveCommand(val speed: Int, val heading: Int, val flags: Int) : CommandMessage()
data class RawMotorsCommand(val lMode: Int, val lSpeed: Int, val rMode: Int, val rSpeed: Int) : CommandMessage()
data class StopCommand(val heading: Int) : CommandMessage()
object WakeCommand : CommandMessage()
object SleepCommand : CommandMessage()
object ResetYawCommand : CommandMessage()
object GetBatteryCommand : CommandMessage()