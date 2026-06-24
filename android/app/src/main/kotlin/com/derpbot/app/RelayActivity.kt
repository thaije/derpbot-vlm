package com.derpbot.app

import android.Manifest
import android.content.ComponentName
import android.content.Context
import android.content.Intent
import android.content.ServiceConnection
import android.content.SharedPreferences
import android.graphics.Color
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.IBinder
import android.os.Looper
import android.text.InputType
import android.view.Gravity
import android.view.MotionEvent
import android.view.View
import android.view.WindowManager
import android.webkit.WebView
import android.widget.Button
import android.widget.EditText
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.TextView
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import com.derpbot.app.RvrRelayService.LocalBinder

/**
 * Thin UI on top of [RvrRelayService].
 *
 * Default screen: dim dark background + the animated derpbot logo (rendered
 * in a WebView so the SVG's CSS @keyframes animations run as-is) + a status
 * line. Tap anywhere → fade in the Connect/STOP/Disconnect/log block. After
 * [IDLE_TIMEOUT_MS] of no touch, fade back to logo-only.
 *
 * The activity no longer owns BLE/camera/relay — the foreground service does.
 * Screen can turn off mid-run without freezing the relay (Freecess exemption).
 */
class RelayActivity : AppCompatActivity() {

    private var service: RvrRelayService? = null
    private var bound = false

    private lateinit var prefs: SharedPreferences
    private lateinit var status: TextView
    private lateinit var log: TextView
    private lateinit var serverInput: EditText
    private lateinit var controls: LinearLayout

    private val handler = Handler(Looper.getMainLooper())
    private val hideControlsRunnable = Runnable { fadeControls(false) }

    private val uiListener = object : RvrRelayService.UiListener {
        override fun onUiUpdate() {
            val s = service ?: return
            status.text = s.statusText
            log.text = s.logLines.joinToString("\n")
        }
    }

    private val connection = object : ServiceConnection {
        override fun onServiceConnected(name: ComponentName?, binder: IBinder?) {
            val s = (binder as LocalBinder).service
            service = s
            bound = true
            s.addUiListener(uiListener)
            uiListener.onUiUpdate()
        }
        override fun onServiceDisconnected(name: ComponentName?) {
            service?.removeUiListener(uiListener)
            service = null
            bound = false
        }
    }

    private val permissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { grants ->
        if (grants.values.all { it }) {
            startRelayService()
        } else status.text = "Permissions denied — need BLE + camera"
    }

    /**
     * Auto-start on launch (matches the pre-#26 activity behaviour): if all
     * runtime permissions are already granted, start the foreground service
     * immediately. Otherwise request permissions first — the granted-callback
     * starts the service. This keeps `drive_test.py`'s no-touch restart working.
     */
    private fun autoStartIfReady() {
        val cameraOnly = getIntent().getBooleanExtra(RvrRelayService.EXTRA_CAMERA_ONLY, false)
        val perms = mutableListOf(Manifest.permission.CAMERA)
        if (!cameraOnly) {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                perms += Manifest.permission.BLUETOOTH_SCAN
                perms += Manifest.permission.BLUETOOTH_CONNECT
            } else {
                perms += Manifest.permission.ACCESS_FINE_LOCATION
            }
        }
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            perms += Manifest.permission.POST_NOTIFICATIONS
        }
        if (perms.all { checkSelfPermission(it) == android.content.pm.PackageManager.PERMISSION_GRANTED }) {
            startRelayService()
        } else {
            permissionLauncher.launch(perms.toTypedArray())
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        prefs = getSharedPreferences("rvr_relay", Context.MODE_PRIVATE)

        // Dim brightness — the screen can be near-black while the relay runs.
        setDimBrightness(true)

        val root = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(40, 96, 40, 64)
            setBackgroundColor(Color.parseColor("#0d0c0a"))
        }
        root.setOnClickListener { fadeControls(true); resetIdleTimer() }

        // Animated logo via WebView (lets the SVG's CSS keyframes run as-is).
        val logo = WebView(this).apply {
            setBackgroundColor(Color.TRANSPARENT)
            settings.javaScriptEnabled = true
            loadUrl("file:///android_asset/logo-animated.svg")
            layoutParams = LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, 0, 1f
            ).apply { gravity = Gravity.CENTER_HORIZONTAL }
        }
        root.addView(logo)

        status = TextView(this).apply {
            text = "derpbot relay — Idle"
            textSize = 16f
            setTextColor(Color.parseColor("#cfc8b8"))
            setPadding(0, 16, 0, 16)
            gravity = Gravity.CENTER_HORIZONTAL
        }
        root.addView(status)

        controls = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            visibility = View.GONE
            alpha = 0f
        }
        root.addView(controls)

        serverInput = EditText(this).apply {
            hint = "Server URL (e.g. ws://192.168.2.20:8765)"
            inputType = InputType.TYPE_TEXT_VARIATION_URI
            setText(prefs.getString("server_url", RvrRelayService.DEFAULT_URL))
            setTextColor(Color.parseColor("#e8e2d2"))
            setHintTextColor(Color.parseColor("#6f685c"))
        }
        controls.addView(serverInput)

        val cameraOnly = getIntent().getBooleanExtra(RvrRelayService.EXTRA_CAMERA_ONLY, false)
        controls.addView(button(
            if (cameraOnly) "Start camera-only relay" else "Connect BLE + camera + start relay"
        ) { startRelay() })
        controls.addView(button("STOP") { service?.stopRelay() })
        controls.addView(button("Disconnect") { service?.disconnect() })

        log = TextView(this).apply {
            textSize = 12f
            setTextColor(Color.parseColor("#8b877f"))
            setPadding(0, 24, 0, 0)
        }
        controls.addView(log)

        setContentView(ScrollView(this).apply {
            isFillViewport = true
            addView(root)
        })

        // Auto-start on launch (no-touch restart for drive_test.py).
        autoStartIfReady()
    }

    /**
     * The WebView eats ACTION_DOWN so root.setOnClickListener never fires for
     * taps on the logo. Catch taps here instead: any down event on the activity
     * window reveals the controls and resets the idle timer. Children still
     * receive the events (buttons keep working) — we just observe.
     */
    override fun dispatchTouchEvent(ev: MotionEvent): Boolean {
        if (ev.action == MotionEvent.ACTION_DOWN) {
            if (controls.visibility != View.VISIBLE) fadeControls(true)
            resetIdleTimer()
        }
        return super.dispatchTouchEvent(ev)
    }

    private fun button(label: String, onClick: () -> Unit) =
        Button(this).apply { text = label; setOnClickListener { onClick(); resetIdleTimer() } }

    private fun startRelay() {
        requestPermissionsThenStart()
    }

    private fun requestPermissionsThenStart() {
        val cameraOnly = getIntent().getBooleanExtra(RvrRelayService.EXTRA_CAMERA_ONLY, false)
        val perms = mutableListOf(Manifest.permission.CAMERA)
        if (!cameraOnly) {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                perms += Manifest.permission.BLUETOOTH_SCAN
                perms += Manifest.permission.BLUETOOTH_CONNECT
            } else {
                perms += Manifest.permission.ACCESS_FINE_LOCATION
            }
        }
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            perms += Manifest.permission.POST_NOTIFICATIONS
        }
        permissionLauncher.launch(perms.toTypedArray())
    }

    private fun startRelayService() {
        val url = serverInput.text.toString().trim()
        prefs.edit().putString("server_url", url).apply()
        val intent = Intent(this, RvrRelayService::class.java)
            .setAction(RvrRelayService.ACTION_START)
            .putExtra(RvrRelayService.EXTRA_SERVER_URL, url)
        // Forward camera_only flag from the launch intent (deploy.sh --camera-only)
        intent.putExtra(RvrRelayService.EXTRA_CAMERA_ONLY,
            getIntent().getBooleanExtra(RvrRelayService.EXTRA_CAMERA_ONLY, false))
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            startForegroundService(intent)
        } else {
            startService(intent)
        }
        bindService(Intent(this, RvrRelayService::class.java), connection, Context.BIND_AUTO_CREATE)
    }

    private fun fadeControls(show: Boolean) {
        controls.animate().alpha(if (show) 1f else 0f).setDuration(200)
            .withStartAction { if (show) controls.visibility = View.VISIBLE }
            .withEndAction { if (!show) controls.visibility = View.GONE }
            .start()
    }

    private fun resetIdleTimer() {
        handler.removeCallbacks(hideControlsRunnable)
        handler.postDelayed(hideControlsRunnable, IDLE_TIMEOUT_MS)
    }

    private fun setDimBrightness(dim: Boolean) {
        val lp = window.attributes
        lp.screenBrightness = if (dim) 0.05f else -1f
        window.attributes = lp
        // FLAG_KEEP_SCREEN_ON is intentionally NOT set — the screen can turn
        // off entirely; the foreground service keeps the relay alive.
    }

    override fun onResume() {
        super.onResume()
        if (!bound) {
            bindService(Intent(this, RvrRelayService::class.java), connection, Context.BIND_AUTO_CREATE)
        }
        resetIdleTimer()
    }

    override fun onPause() {
        super.onPause()
        handler.removeCallbacks(hideControlsRunnable)
    }

    override fun onDestroy() {
        if (bound) {
            service?.removeUiListener(uiListener)
            unbindService(connection)
            bound = false
        }
        super.onDestroy()
    }

    companion object {
        private const val IDLE_TIMEOUT_MS = 10_000L
    }
}