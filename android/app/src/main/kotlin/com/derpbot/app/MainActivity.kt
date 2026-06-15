package com.derpbot.app

import android.Manifest
import android.os.Build
import android.os.Bundle
import android.text.InputType
import android.view.Gravity
import android.widget.Button
import android.widget.EditText
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.TextView
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.lifecycle.lifecycleScope
import com.derpbot.app.ble.RvrBleConnection
import com.derpbot.app.camera.CameraManager
import com.derpbot.app.control.ControlLoop
import com.derpbot.app.vlm.VlmClient
import com.derpbot.rvr.protocol.Packet
import com.derpbot.rvr.protocol.RvrCommands
import kotlinx.coroutines.launch

/**
 * Bring-up + autonomous harness (issue #19).
 *
 * Step 1: Connect / Wake / STOP / Battery buttons (manual BLE bring-up).
 * Steps 2-4: enter a target object + the Ollama server URL, tap "Start loop" to
 * run the camera→VLM→drive cycle. STOP halts both the loop and the motors and is
 * the always-live manual override (Step 5).
 *
 * The phone reaches the cloud VLM through a local-network Ollama daemon (the
 * computer running `ollama signin`, started with OLLAMA_HOST=0.0.0.0). Point the
 * URL field at that machine, e.g. http://192.168.1.50:11434.
 */
class MainActivity : AppCompatActivity(), RvrBleConnection.Listener {

    private val commands = RvrCommands()
    private lateinit var connection: RvrBleConnection
    private lateinit var camera: CameraManager
    private var controlLoop: ControlLoop? = null

    private lateinit var status: TextView
    private lateinit var log: TextView
    private lateinit var targetInput: EditText
    private lateinit var urlInput: EditText
    private lateinit var modelInput: EditText
    private lateinit var apiKeyInput: EditText

    private var cameraBound = false

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
            // Generous top padding clears the status bar (NoActionBar theme draws
            // content edge-to-edge); bottom padding leaves room above the nav bar.
            setPadding(40, 96, 40, 64)
        }
        status = TextView(this).apply {
            text = "derpbot RVR — Idle"
            textSize = 16f
            setPadding(0, 0, 0, 16)
        }
        root.addView(status)

        targetInput = EditText(this).apply {
            hint = "target object (e.g. fire_extinguisher)"
            inputType = InputType.TYPE_CLASS_TEXT
        }
        urlInput = EditText(this).apply {
            hint = "Ollama URL"
            setText("https://ollama.com")   // direct cloud; or http://<pc-ip>:11434 for a local daemon
            inputType = InputType.TYPE_TEXT_VARIATION_URI
        }
        modelInput = EditText(this).apply {
            hint = "model"
            setText("gemma4:31b-cloud")
            inputType = InputType.TYPE_CLASS_TEXT
        }
        apiKeyInput = EditText(this).apply {
            hint = "Ollama API key (blank for local daemon)"
            inputType = InputType.TYPE_CLASS_TEXT or InputType.TYPE_TEXT_VARIATION_PASSWORD
        }
        root.addView(targetInput)
        root.addView(urlInput)
        root.addView(modelInput)
        root.addView(apiKeyInput)

        root.addView(button("Connect (BLE + camera)") { requestPermissionsThenScan() })
        root.addView(button("Start loop") { startLoop() })
        root.addView(button("STOP") { stopAll() })
        root.addView(button("Battery %") { connection.send(commands.getBatteryPercentage()) })
        root.addView(button("Disconnect") { connection.disconnect() })

        log = TextView(this).apply { textSize = 12f; setPadding(0, 24, 0, 0) }
        root.addView(log)

        // Whole screen scrolls so every field (incl. target, top) is reachable
        // even with the keyboard up — a plain LinearLayout clipped the top row
        // behind the action bar.
        setContentView(ScrollView(this).apply {
            isFillViewport = true
            addView(root)
        })
    }

    private fun button(label: String, onClick: () -> Unit) =
        Button(this).apply { text = label; setOnClickListener { onClick() } }

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

    private fun startLoop() {
        val target = targetInput.text.toString().trim()
        val url = urlInput.text.toString().trim().removeSuffix("/")
        val model = modelInput.text.toString().trim()
        val apiKey = apiKeyInput.text.toString().trim().ifEmpty { null }
        if (target.isEmpty() || url.isEmpty() || model.isEmpty()) {
            appendLog("Fill target, URL, and model first")
            return
        }
        val vlm = VlmClient(this, url, model, apiKey)
        controlLoop = ControlLoop(
            scope = lifecycleScope,
            camera = camera,
            vlm = vlm,
            connection = connection,
            commands = commands,
            targetObject = target,
            onEvent = { msg -> runOnUiThread { appendLog(msg) } },
        ).also { it.start() }
    }

    private fun stopAll() {
        controlLoop?.stop()
        connection.send(commands.stop())
    }

    private fun appendLog(msg: String) {
        log.append("• $msg\n")
    }

    // --- RvrBleConnection.Listener -----------------------------------------

    override fun onStateChange(state: RvrBleConnection.State) {
        status.text = "BLE: $state"
        if (state == RvrBleConnection.State.READY) connection.send(commands.wake())
    }

    override fun onPacket(packet: Packet) {
        appendLog("RX did=${packet.did} cid=${packet.cid} err=${packet.error} data=${packet.data.toList()}")
    }

    override fun onError(message: String) {
        appendLog("Error: $message")
    }

    override fun onDestroy() {
        controlLoop?.stop()
        connection.disconnect()
        super.onDestroy()
    }
}
