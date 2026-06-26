package com.derpbot.app.ble

import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattDescriptor
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothProfile
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanFilter
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.Context
import android.os.Build
import android.os.Handler
import android.os.Looper
import android.util.Log
import com.derpbot.rvr.protocol.Packet
import com.derpbot.rvr.protocol.PacketCollector
import java.util.UUID

/**
 * BLE transport to a Sphero RVR / RVR+ (issue #19, Step 1).
 *
 * Drives Android's [BluetoothGatt] against the Sphero v2 API service and feeds
 * inbound notification chunks through the protocol's [PacketCollector]. Outbound
 * commands are built by `RvrCommands` (in the :rvr module) and handed to [send].
 *
 * UUIDs are from the `spherov2.py` reference: a single characteristic is used
 * for both writing commands and receiving notifications. Confirm these against
 * `nRF Connect` on first hardware bring-up — Sphero firmware revisions have
 * shipped slightly different layouts (alternatives are listed in the reference).
 *
 * GATT is single-threaded and callback-driven: never issue a second GATT op
 * before the previous one's callback fires. This class only issues one op at a
 * time (connect → discover → enable-notify → then writes), so it stays within
 * that contract for the simple Step-1 control loop.
 */
class RvrBleConnection(
    private val context: Context,
    private val listener: Listener,
) {
    interface Listener {
        fun onStateChange(state: State)
        fun onPacket(packet: Packet)
        fun onError(message: String)
    }

    enum class State { IDLE, SCANNING, CONNECTING, DISCOVERING, READY, DISCONNECTED }

    private val main = Handler(Looper.getMainLooper())
    private val adapter: BluetoothAdapter? =
        (context.getSystemService(Context.BLUETOOTH_SERVICE) as? BluetoothManager)?.adapter

    private var gatt: BluetoothGatt? = null
    private var commandChar: BluetoothGattCharacteristic? = null
    private val collector = PacketCollector { packet -> main.post { listener.onPacket(packet) } }

    @Volatile private var state: State = State.IDLE
        private set(value) {
            field = value
            main.post { listener.onStateChange(value) }
        }

    /** Current state, for callers that need it outside an [onStateChange] push. */
    val currentState: State get() = state

    // --- Scanning -----------------------------------------------------------

    @SuppressLint("MissingPermission")
    fun startScanAndConnect() {
        if (adapter == null) {
            listener.onError("Bluetooth not available on this device")
            state = State.IDLE
            return
        }
        if (!adapter!!.isEnabled) {
            // Try to enable BT automatically. Requires BLUETOOTH_CONNECT on API 31+,
            // which is already in the activity's permission set. If the system
            // blocks the request (e.g. device policy), surface a clear error so
            // the user knows to turn Bluetooth on manually.
            val enabled = try { adapter!!.enable() } catch (e: SecurityException) {
                Log.w(TAG, "enable() blocked: ${e.message}")
                false
            }
            if (!enabled) {
                listener.onError("Bluetooth is off — enable it in Settings")
                state = State.IDLE
                return
            }
            // adapter.enable() is async; give the stack a moment to come up.
            main.postDelayed({ startScanAndConnect() }, 1500)
            return
        }
        val scanner = adapter?.bluetoothLeScanner
        if (scanner == null) {
            listener.onError("Bluetooth scanner unavailable (BT off?)")
            state = State.IDLE
            return
        }
        state = State.SCANNING
        // RVR advertises with a name beginning "RV-"; match on that since the
        // API service UUID is not always present in the advertisement packet.
        val settings = ScanSettings.Builder()
            .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
            .build()
        scanner.startScan(/* filters = */ emptyList<ScanFilter>(), settings, scanCallback)
    }

    private val scanCallback = object : ScanCallback() {
        @SuppressLint("MissingPermission")
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            val name = result.device.name ?: result.scanRecord?.deviceName ?: return
            if (!name.startsWith(RVR_NAME_PREFIX)) return
            Log.i(TAG, "Found RVR '$name' (${result.device.address}); connecting")
            adapter?.bluetoothLeScanner?.stopScan(this)
            connect(result.device)
        }

        override fun onScanFailed(errorCode: Int) {
            listener.onError("BLE scan failed: $errorCode")
            state = State.IDLE
        }
    }

    // --- Connection ---------------------------------------------------------

    @SuppressLint("MissingPermission")
    private fun connect(device: BluetoothDevice) {
        state = State.CONNECTING
        gatt = device.connectGatt(context, /* autoConnect = */ false, gattCallback)
    }

    private val gattCallback = object : BluetoothGattCallback() {
        @SuppressLint("MissingPermission")
        override fun onConnectionStateChange(g: BluetoothGatt, status: Int, newState: Int) {
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                state = State.DISCOVERING
                g.discoverServices()
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                cleanup()
                state = State.DISCONNECTED
            }
        }

        @SuppressLint("MissingPermission")
        override fun onServicesDiscovered(g: BluetoothGatt, status: Int) {
            if (status != BluetoothGatt.GATT_SUCCESS) {
                listener.onError("Service discovery failed: $status")
                return
            }
            val service = g.getService(SERVICE_UUID)
            val char = service?.getCharacteristic(COMMAND_UUID)
            if (char == null) {
                listener.onError("RVR API characteristic not found — check UUIDs")
                return
            }
            commandChar = char
            Log.i(TAG, "Service discovered, enabling notifications...")
            enableNotifications(g, char)
        }

        @SuppressLint("MissingPermission")
        override fun onDescriptorWrite(g: BluetoothGatt, descriptor: BluetoothGattDescriptor, status: Int) {
            // CCCD write completed → notifications are live, we're ready to drive.
            if (descriptor.uuid == CCCD_UUID) {
                Log.i(TAG, "CCCD write status=$status → READY")
                state = State.READY
            }
        }

        @SuppressLint("MissingPermission")
        override fun onCharacteristicWrite(g: BluetoothGatt, char: BluetoothGattCharacteristic, status: Int) {
            Log.i(TAG, "onCharacteristicWrite status=$status (0=success)")
        }

        // Notifications: API 33+ delivers bytes via the `value` overload.
        override fun onCharacteristicChanged(
            g: BluetoothGatt, char: BluetoothGattCharacteristic, value: ByteArray,
        ) = collector.feed(value)

        @Deprecated("Pre-33 overload")
        override fun onCharacteristicChanged(g: BluetoothGatt, char: BluetoothGattCharacteristic) {
            @Suppress("DEPRECATION")
            char.value?.let { collector.feed(it) }
        }
    }

    @SuppressLint("MissingPermission")
    private fun enableNotifications(g: BluetoothGatt, char: BluetoothGattCharacteristic) {
        g.setCharacteristicNotification(char, true)
        val cccd = char.getDescriptor(CCCD_UUID) ?: run {
            listener.onError("CCCD descriptor missing")
            return
        }
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            g.writeDescriptor(cccd, BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE)
        } else {
            @Suppress("DEPRECATION")
            cccd.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
            @Suppress("DEPRECATION")
            g.writeDescriptor(cccd)
        }
    }

    // --- Sending ------------------------------------------------------------

    /** Serialise and write a command packet. No-op (logs) if not READY. */
    @SuppressLint("MissingPermission")
    fun send(packet: Packet) {
        val g = gatt
        val char = commandChar
        if (g == null || char == null || state != State.READY) {
            Log.w(TAG, "send() ignored — not ready (state=$state, gatt=${g != null}, char=${char != null})")
            return
        }
        val bytes = packet.build()
        Log.i(TAG, "send() DID=${packet.did} CID=${packet.cid} ${bytes.size}B state=$state")
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            val rc = g.writeCharacteristic(char, bytes, BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT)
            Log.i(TAG, "writeCharacteristic rc=$rc (1=success)")
        } else {
            @Suppress("DEPRECATION")
            char.value = bytes
            @Suppress("DEPRECATION")
            char.writeType = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
            @Suppress("DEPRECATION")
            val rc = g.writeCharacteristic(char)
            Log.i(TAG, "writeCharacteristic rc=$rc (true=success)")
        }
    }

    @SuppressLint("MissingPermission")
    fun disconnect() {
        adapter?.bluetoothLeScanner?.stopScan(scanCallback)
        gatt?.disconnect()
    }

    @SuppressLint("MissingPermission")
    private fun cleanup() {
        gatt?.close()
        gatt = null
        commandChar = null
    }

    companion object {
        private const val TAG = "RvrBle"
        private const val RVR_NAME_PREFIX = "RV-"

        // Sphero v2 API service + command/notify characteristic (from spherov2.py).
        val SERVICE_UUID: UUID = UUID.fromString("00010001-574f-4f20-5370-6865726f2121")
        val COMMAND_UUID: UUID = UUID.fromString("00010002-574f-4f20-5370-6865726f2121")
        // Standard Client Characteristic Configuration Descriptor.
        val CCCD_UUID: UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")
    }
}
