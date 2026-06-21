package com.derpbot.app.relay

import android.content.Context
import android.graphics.Bitmap
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.BatteryManager
import android.util.Base64
import android.util.Log
import com.derpbot.app.ble.RvrBleConnection
import com.derpbot.app.camera.CameraManager
import com.derpbot.rvr.protocol.RvrCommands
import kotlinx.coroutines.*
import kotlin.math.max
import okhttp3.*
import java.io.ByteArrayOutputStream
import java.util.concurrent.TimeUnit

class RvrRelay(
    private val scope: CoroutineScope,
    private val context: Context,
    private val camera: CameraManager,
    private val connection: RvrBleConnection,
    private val commands: RvrCommands,
    private val serverUrl: String,
    private val onEvent: (String) -> Unit,
) {
    private val client = OkHttpClient.Builder()
        .readTimeout(0, TimeUnit.MILLISECONDS)
        .pingInterval(30, TimeUnit.SECONDS)
        .build()

    private var webSocket: WebSocket? = null
    private var connected = false

    @Volatile private var running = false

    private var sensorManager: SensorManager? = null
    private var accelerometer: Sensor? = null
    private var gyroscope: Sensor? = null
    private var sensorListener: SensorEventListener? = null
    private var lastImuSendMs: Long = 0

    fun start() {
        if (running) return
        running = true
        scope.launch { connect() }
    }

    fun stop() {
        running = false
        disconnectSensors()
        webSocket?.close(1000, "relay stopped")
        webSocket = null
    }

    private suspend fun connect() {
        onEvent("Connecting to $serverUrl ...")
        Log.i(TAG, "Connecting to $serverUrl ...")
        val request = Request.Builder().url(serverUrl).build()
        val wsListener = object : WebSocketListener() {
            override fun onOpen(ws: WebSocket, response: Response) {
                connected = true
                onEvent("WebSocket connected")
                Log.i(TAG, "WebSocket connected")
                scope.launch { startSensors() }
            }

            override fun onMessage(ws: WebSocket, text: String) {
                Log.d(TAG, "onMessage: ${text.take(80)}")
                val cmd = decodeCommand(text) ?: return
                handleCommand(cmd)
            }

            override fun onClosing(ws: WebSocket, code: Int, reason: String) {
                Log.i(TAG, "onClosing: $code $reason")
                ws.close(code, reason)
            }

            override fun onClosed(ws: WebSocket, code: Int, reason: String) {
                connected = false
                onEvent("WebSocket closed: $code $reason")
                Log.i(TAG, "onClosed: $code $reason")
                scheduleReconnect()
            }

            override fun onFailure(ws: WebSocket, t: Throwable, response: Response?) {
                connected = false
                onEvent("WebSocket failure: ${t.message}")
                Log.e(TAG, "onFailure: ${t.javaClass.simpleName}: ${t.message}", t)
                scheduleReconnect()
            }
        }
        webSocket = client.newWebSocket(request, wsListener)
    }

    private fun scheduleReconnect() {
        if (!running) return
        scope.launch {
            delay(3000)
            if (running && !connected) {
                onEvent("Reconnecting...")
                connect()
            }
        }
    }

    private fun handleCommand(cmd: CommandMessage) {
        when (cmd) {
            is CaptureFrameCommand -> scope.launch { sendFrame() }
            is DriveCommand -> {
                connection.send(commands.driveWithHeading(cmd.speed, cmd.heading, cmd.flags))
                onEvent("DRIVE speed=${cmd.speed} hdg=${cmd.heading}")
            }
            is RawMotorsCommand -> {
                connection.send(commands.setRawMotors(cmd.lMode, cmd.lSpeed, cmd.rMode, cmd.rSpeed))
            }
            is StopCommand -> {
                connection.send(commands.stop(cmd.heading))
                onEvent("STOP hdg=${cmd.heading}")
            }
            is WakeCommand -> {
                connection.send(commands.wake())
                onEvent("WAKE")
            }
            is SleepCommand -> {
                connection.send(commands.sleep())
                onEvent("SLEEP")
            }
            is ResetYawCommand -> {
                connection.send(commands.resetYaw())
                onEvent("RESET_YAW")
            }
            is GetBatteryCommand -> {
                connection.send(commands.getBatteryPercentage())
            }
            is GetBleStateCommand -> {
                sendBleState(connection.currentState.name.lowercase())
            }
            is GetPhoneBatteryCommand -> {
                sendPhoneBattery()
            }
        }
    }

    private suspend fun sendFrame() {
        val bitmap = camera.captureFrame() ?: run {
            onEvent("Frame capture failed")
            return
        }
        val msg = encodeFrame(bitmap)
        webSocket?.send(msg)
        bitmap.recycle()
    }

    private fun encodeFrame(bitmap: Bitmap): String {
        val maxDim = 768
        val w = bitmap.width
        val h = bitmap.height
        val scaled = if (max(w, h) > maxDim) {
            val s = maxDim.toFloat() / max(w, h)
            Bitmap.createScaledBitmap(bitmap, (w * s).toInt(), (h * s).toInt(), true)
        } else bitmap

        val buf = ByteArrayOutputStream()
        scaled.compress(Bitmap.CompressFormat.JPEG, 90, buf)
        val b64 = Base64.encodeToString(buf.toByteArray(), Base64.NO_WRAP)

        return FrameMessage(
            jpegB64 = b64,
            width = scaled.width,
            height = scaled.height,
            rotation = 0,
        ).toJson()
    }

    private fun startSensors() {
        sensorManager = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager
        accelerometer = sensorManager?.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        gyroscope = sensorManager?.getDefaultSensor(Sensor.TYPE_GYROSCOPE)

        sensorListener = object : SensorEventListener {
            override fun onSensorChanged(event: SensorEvent) {
                sendImu(event)
            }
            override fun onAccuracyChanged(sensor: Sensor, accuracy: Int) {}
        }

        accelerometer?.let { sensorManager?.registerListener(sensorListener, it, 20_000) }
        gyroscope?.let { sensorManager?.registerListener(sensorListener, it, 20_000) }
    }

    private fun disconnectSensors() {
        sensorListener?.let { sensorManager?.unregisterListener(it) }
        sensorListener = null
    }

    private fun sendImu(event: SensorEvent) {
        val now = System.currentTimeMillis()
        if (now - lastImuSendMs < 20) return // cap at ~50 Hz
        lastImuSendMs = now

        val values = event.values
        val (accel, gyro) = when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> Pair(
                floatArrayOf(values[0], values[1], values[2]),
                floatArrayOf(0f, 0f, 0f),
            )
            Sensor.TYPE_GYROSCOPE -> Pair(
                floatArrayOf(0f, 0f, 0f),
                floatArrayOf(values[0], values[1], values[2]),
            )
            else -> return
        }
        val msg = ImuMessage(accel, gyro, now / 1000.0).toJson()
        webSocket?.send(msg)
    }

    fun sendBleState(state: String) {
        val msg = BleStateMessage(state).toJson()
        webSocket?.send(msg)
    }

    fun sendBattery(pct: Int) {
        val msg = BatteryMessage(pct).toJson()
        webSocket?.send(msg)
    }

    fun sendPhoneBattery() {
        val bm = context.getSystemService(Context.BATTERY_SERVICE) as BatteryManager
        val pct = bm.getIntProperty(BatteryManager.BATTERY_PROPERTY_CAPACITY)
        val msg = PhoneBatteryMessage(pct).toJson()
        webSocket?.send(msg)
    }

    companion object {
        private const val TAG = "RvrRelay"
    }
}