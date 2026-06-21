package com.derpbot.app

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.app.Service
import android.content.Context
import android.content.Intent
import android.content.SharedPreferences
import android.content.pm.ServiceInfo
import android.os.Build
import android.os.Binder
import android.os.IBinder
import android.util.Log
import androidx.core.app.NotificationCompat
import androidx.lifecycle.LifecycleService
import androidx.lifecycle.lifecycleScope
import com.derpbot.app.ble.RvrBleConnection
import com.derpbot.app.camera.CameraManager
import com.derpbot.app.relay.RvrRelay
import com.derpbot.rvr.protocol.Packet
import com.derpbot.rvr.protocol.RvrCommands
import kotlinx.coroutines.launch

/**
 * Foreground service owning the BLE link, camera, and WebSocket relay (#26).
 *
 * Promoting the relay to a foreground service exempts it from Samsung's
 * "Freecess" app-freezer, which previously killed the WebSocket before BLE
 * reached `ready` if the screen was off at launch (STATE.md #21). The screen
 * can now dim or turn off while a run is in progress — the activity is a UI
 * on top, not the lifecycle owner.
 *
 * The activity binds to this service and reads [statusText]/[logLines] to
 * render state. All intelligence (VLM, planner, safety) still runs in
 * Python on the computer; this service is a thin transport.
 */
class RvrRelayService : LifecycleService(), RvrBleConnection.Listener {

    private val commands = RvrCommands()
    private lateinit var connection: RvrBleConnection
    private lateinit var camera: CameraManager
    private var relay: RvrRelay? = null

    private var cameraBound = false

    private val prefs: SharedPreferences by lazy {
        getSharedPreferences("rvr_relay", Context.MODE_PRIVATE)
    }

    /** UI observable state. Updated on the main thread. */
    @Volatile var statusText: String = "derpbot relay — Idle"
        private set
    @Volatile var bleState: String = "idle"
        private set

    private val _logLines: MutableList<String> = mutableListOf()
    val logLines: List<String> get() = _logLines

    interface UiListener { fun onUiUpdate() }
    private val uiListeners = mutableListOf<UiListener>()
    fun addUiListener(l: UiListener) { uiListeners += l }
    fun removeUiListener(l: UiListener) { uiListeners -= l }

    inner class LocalBinder : Binder() { val service: RvrRelayService get() = this@RvrRelayService }
    private val binder = LocalBinder()

    override fun onBind(intent: Intent): IBinder {
        super.onBind(intent)
        return binder
    }

    override fun onCreate() {
        super.onCreate()
        connection = RvrBleConnection(this, this)
        camera = CameraManager(this, this)
        Log.i(TAG, "Service created")
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        super.onStartCommand(intent, flags, startId)
        when (intent?.action) {
            ACTION_START -> startRelay(intent.getStringExtra(EXTRA_SERVER_URL)
                ?: prefs.getString("server_url", DEFAULT_URL)!!)
            ACTION_STOP -> stopRelay()
        }
        return START_STICKY
    }

    fun startRelay(url: String) {
        prefs.edit().putString("server_url", url).apply()
        ensureForeground()
        if (relay != null) return
        connection.startScanAndConnect()
        bindCameraOnce()
        relay = RvrRelay(
            scope = lifecycleScope,
            context = this,
            camera = camera,
            connection = connection,
            commands = commands,
            serverUrl = url,
            onEvent = { msg -> runOnUiThread { appendLog(msg) } },
        ).also { it.start() }
        statusText = "derpbot relay — running"
        notifyUi()
    }

    fun stopRelay() {
        relay?.stop()
        relay = null
        connection.send(commands.stop())
        statusText = "derpbot relay — stopped"
        notifyUi()
    }

    fun disconnect() {
        relay?.stop()
        relay = null
        connection.disconnect()
        statusText = "derpbot relay — disconnected"
        notifyUi()
    }

    override fun onDestroy() {
        relay?.stop()
        connection.disconnect()
        super.onDestroy()
    }

    // --- BLE listener ------------------------------------------------------

    override fun onStateChange(state: RvrBleConnection.State) {
        bleState = state.name.lowercase()
        statusText = "BLE: $state"
        relay?.sendBleState(state.name.lowercase())
        if (state == RvrBleConnection.State.READY) connection.send(commands.wake())
        notifyUi()
    }

    override fun onPacket(packet: Packet) {
        if (packet.did == RvrCommands.DID_POWER && packet.cid == RvrCommands.CID_BATTERY_PERCENTAGE) {
            val pct = packet.data.firstOrNull() ?: return
            appendLog("Battery: $pct%")
            relay?.sendBattery(pct)
        } else {
            appendLog("RX did=${packet.did} cid=${packet.cid} data=${packet.data.toList()}")
        }
    }

    override fun onError(message: String) {
        appendLog("Error: $message")
    }

    // --- Camera ------------------------------------------------------------

    private fun bindCameraOnce() {
        if (cameraBound) return
        cameraBound = true
        lifecycleScope.launch {
            runCatching { camera.bind() }
                .onFailure { appendLog("Camera bind failed: ${it.message}") }
        }
    }

    // --- Foreground notification ------------------------------------------

    private fun ensureForeground() {
        val nm = getSystemService(NotificationManager::class.java)
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            if (nm.getNotificationChannel(CHANNEL_ID) == null) {
                nm.createNotificationChannel(NotificationChannel(
                    CHANNEL_ID, "Relay", NotificationManager.IMPORTANCE_LOW,
                ).apply { description = "derpbot phone relay active" })
            }
        }
        val pi = PendingIntent.getActivity(
            this, 0,
            Intent(this, RelayActivity::class.java),
            PendingIntent.FLAG_IMMUTABLE or PendingIntent.FLAG_UPDATE_CURRENT,
        )
        val notif: Notification = NotificationCompat.Builder(this, CHANNEL_ID)
            .setContentTitle("derpbot relay")
            .setContentText("Busy — screen can dim")
            .setSmallIcon(android.R.drawable.ic_menu_camera)
            .setOngoing(true)
            .setContentIntent(pi)
            .build()
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.UPSIDE_DOWN_CAKE) {
            startForeground(
                NOTIF_ID, notif,
                ServiceInfo.FOREGROUND_SERVICE_TYPE_CONNECTED_DEVICE,
            )
        } else {
            startForeground(NOTIF_ID, notif)
        }
    }

    // --- UI plumbing -------------------------------------------------------

    private fun appendLog(msg: String) {
        _logLines += "• $msg"
        notifyUi()
    }

    private fun notifyUi() {
        val snapshot = uiListeners.toList()
        for (l in snapshot) l.onUiUpdate()
    }

    private fun runOnUiThread(block: () -> Unit) {
        android.os.Handler(android.os.Looper.getMainLooper()).post { block() }
    }

    companion object {
        private const val TAG = "RvrRelayService"
        private const val CHANNEL_ID = "rvr_relay"
        private const val NOTIF_ID = 1
        const val DEFAULT_URL = "ws://192.168.2.20:8765"
        const val ACTION_START = "com.derpbot.app.START"
        const val ACTION_STOP = "com.derpbot.app.STOP"
        const val EXTRA_SERVER_URL = "server_url"
    }
}