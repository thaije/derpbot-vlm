package com.derpbot.app

import android.Manifest
import android.content.Context
import android.content.SharedPreferences
import android.os.Build
import android.os.Bundle
import android.text.InputType
import android.view.Gravity
import android.widget.Button
import android.widget.EditText
import android.widget.ImageView
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.TextView
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.lifecycle.lifecycleScope
import kotlinx.coroutines.launch
import com.derpbot.app.ble.RvrBleConnection
import com.derpbot.app.camera.CameraManager
import com.derpbot.app.relay.RvrRelay
import com.derpbot.rvr.protocol.Packet
import com.derpbot.rvr.protocol.RvrCommands

class RelayActivity : AppCompatActivity(), RvrBleConnection.Listener {

    private val commands = RvrCommands()
    private lateinit var connection: RvrBleConnection
    private lateinit var camera: CameraManager
    private var relay: RvrRelay? = null

    private lateinit var status: TextView
    private lateinit var log: TextView
    private lateinit var serverInput: EditText

    private var cameraBound = false

    private val prefs: SharedPreferences by lazy {
        getSharedPreferences("rvr_relay", Context.MODE_PRIVATE)
    }

    private val permissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { grants ->
        if (grants.values.all { it }) {
            connection.startScanAndConnect()
            bindCameraOnce()
        } else status.text = "Permissions denied — need BLE + camera"
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        connection = RvrBleConnection(this, this)
        camera = CameraManager(this, this)

        val root = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(40, 96, 40, 64)
        }

        val logo = ImageView(this).apply {
            setImageResource(android.R.drawable.ic_menu_camera)
            layoutParams = LinearLayout.LayoutParams(LinearLayout.LayoutParams.WRAP_CONTENT, LinearLayout.LayoutParams.WRAP_CONTENT).apply {
                gravity = Gravity.CENTER_HORIZONTAL
            }
            setPadding(0, 0, 0, 16)
        }
        root.addView(logo)

        status = TextView(this).apply {
            text = "derpbot relay — Idle"
            textSize = 16f
            setPadding(0, 0, 0, 16)
        }
        root.addView(status)

        serverInput = EditText(this).apply {
            hint = "Server URL (e.g. ws://192.168.2.20:8765)"
            inputType = InputType.TYPE_TEXT_VARIATION_URI
            setText(prefs.getString("server_url", "ws://192.168.2.20:8765"))
        }
        root.addView(serverInput)

        root.addView(button("Connect BLE + camera + start relay") { startRelay() })
        root.addView(button("STOP") { stopAll() })
        root.addView(button("Disconnect") { disconnect() })

        log = TextView(this).apply { textSize = 12f; setPadding(0, 24, 0, 0) }
        root.addView(log)

        setContentView(ScrollView(this).apply {
            isFillViewport = true
            addView(root)
        })

        startRelay()
    }

    private fun button(label: String, onClick: () -> Unit) =
        Button(this).apply { text = label; setOnClickListener { onClick() } }

    private fun startRelay() {
        val url = serverInput.text.toString().trim()
        prefs.edit().putString("server_url", url).apply()
        requestPermissionsThenScan()

        relay = RvrRelay(
            scope = lifecycleScope,
            context = this,
            camera = camera,
            connection = connection,
            commands = commands,
            serverUrl = url,
            onEvent = { msg -> runOnUiThread { appendLog(msg) } },
        ).also { it.start() }
    }

    private fun stopAll() {
        relay?.stop()
        connection.send(commands.stop())
    }

    private fun disconnect() {
        relay?.stop()
        connection.disconnect()
    }

    private fun requestPermissionsThenScan() {
        val perms = mutableListOf(Manifest.permission.CAMERA)
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            perms += Manifest.permission.BLUETOOTH_SCAN
            perms += Manifest.permission.BLUETOOTH_CONNECT
        } else {
            perms += Manifest.permission.ACCESS_FINE_LOCATION
        }
        permissionLauncher.launch(perms.toTypedArray())
    }

    private fun bindCameraOnce() {
        if (cameraBound) return
        cameraBound = true
        lifecycleScope.launch {
            runCatching { camera.bind() }
                .onFailure { appendLog("Camera bind failed: ${it.message}") }
        }
    }

    private fun appendLog(msg: String) {
        log.append("• $msg\n")
    }

    override fun onStateChange(state: RvrBleConnection.State) {
        status.text = "BLE: $state"
        relay?.sendBleState(state.name.lowercase())
        if (state == RvrBleConnection.State.READY) connection.send(commands.wake())
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

    override fun onDestroy() {
        relay?.stop()
        connection.disconnect()
        super.onDestroy()
    }
}