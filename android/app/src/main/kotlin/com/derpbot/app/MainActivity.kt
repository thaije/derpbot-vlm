package com.derpbot.app

import android.Manifest
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.view.Gravity
import android.widget.Button
import android.widget.LinearLayout
import android.widget.TextView
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import com.derpbot.app.ble.RvrBleConnection
import com.derpbot.rvr.protocol.DriveFlags
import com.derpbot.rvr.protocol.Packet
import com.derpbot.rvr.protocol.RvrCommands

/**
 * Step-1 bring-up harness (issue #19): connect to the RVR, wake it, drive a
 * short forward burst, then stop. A STOP button is always live as the manual
 * override (Step 5). This is throwaway scaffolding for hardware validation; the
 * autonomous camera→VLM→drive loop (Steps 2-4) replaces the buttons later.
 */
class MainActivity : AppCompatActivity(), RvrBleConnection.Listener {

    private val main = Handler(Looper.getMainLooper())
    private val commands = RvrCommands()
    private lateinit var connection: RvrBleConnection
    private lateinit var status: TextView

    private val permissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { grants ->
        if (grants.values.all { it }) connection.startScanAndConnect()
        else status.text = "Permissions denied — cannot use BLE"
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        connection = RvrBleConnection(this, this)

        val root = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            gravity = Gravity.CENTER
            setPadding(48, 48, 48, 48)
        }
        status = TextView(this).apply { text = "Idle"; textSize = 18f }
        root.addView(status)
        root.addView(button("Connect") { requestPermissionsThenScan() })
        root.addView(button("Wake") { connection.send(commands.wake()) })
        root.addView(button("Drive forward (1s)") { driveBurst() })
        root.addView(button("STOP") { connection.send(commands.stop()) })
        root.addView(button("Battery %") { connection.send(commands.getBatteryPercentage()) })
        root.addView(button("Disconnect") { connection.disconnect() })
        setContentView(root)
    }

    private fun button(label: String, onClick: () -> Unit) =
        Button(this).apply { text = label; setOnClickListener { onClick() } }

    private fun requestPermissionsThenScan() {
        val perms = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            arrayOf(Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT)
        } else {
            arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)
        }
        permissionLauncher.launch(perms)
    }

    private fun driveBurst() {
        connection.send(commands.driveWithHeading(speed = 64, heading = 0, flags = DriveFlags.FORWARD))
        main.postDelayed({ connection.send(commands.stop()) }, 1000)
    }

    // --- RvrBleConnection.Listener -----------------------------------------

    override fun onStateChange(state: RvrBleConnection.State) {
        status.text = "State: $state"
        if (state == RvrBleConnection.State.READY) {
            // Wake automatically once the link is up.
            connection.send(commands.wake())
        }
    }

    override fun onPacket(packet: Packet) {
        status.text = "RX did=${packet.did} cid=${packet.cid} err=${packet.error} data=${packet.data.toList()}"
    }

    override fun onError(message: String) {
        status.text = "Error: $message"
    }

    override fun onDestroy() {
        connection.disconnect()
        super.onDestroy()
    }
}
