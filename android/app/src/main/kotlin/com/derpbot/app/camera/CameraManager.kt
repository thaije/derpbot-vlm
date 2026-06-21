package com.derpbot.app.camera

import android.content.Context
import android.graphics.Bitmap
import android.graphics.Matrix
import android.util.Log
import androidx.camera.core.Camera
import androidx.camera.core.CameraSelector
import androidx.camera.core.ImageAnalysis
import androidx.camera.core.ImageProxy
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.core.content.ContextCompat
import androidx.lifecycle.LifecycleOwner
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.suspendCancellableCoroutine
import kotlinx.coroutines.withContext
import java.util.concurrent.Executors
import kotlin.coroutines.resume
import kotlin.coroutines.resumeWithException

/**
 * CameraX wrapper for the RVR control loop (issue #19 Step 2; reworked #24).
 *
 * [bind] must be called once from a coroutine (before the control loop starts).
 *
 * Uses [ImageAnalysis], not `ImageCapture`: `takePicture()` is a full
 * still-photo shutter (AE/AF settle, full capture pipeline) — fine for one
 * frame every ~0.5-1s (the VLM's "occasional image" cadence) but ~1.1-1.25s
 * round-trip in practice, far short of the panel's 2 fps live teleop feed.
 * ImageAnalysis streams frames continuously off the live preview pipeline
 * with no per-frame shutter cost; [captureFrame] just returns whichever one
 * is already sitting in memory.
 */
class CameraManager(
    private val context: Context,
    private val lifecycleOwner: LifecycleOwner,
) {
    private val analyzerExecutor = Executors.newSingleThreadExecutor()

    @Volatile private var latestFrame: Bitmap? = null
    @Volatile private var lastConvertMs: Long = 0L
    private var camera: Camera? = null

    suspend fun bind() {
        val provider: ProcessCameraProvider = suspendCancellableCoroutine { cont ->
            val future = ProcessCameraProvider.getInstance(context)
            future.addListener({
                runCatching { cont.resume(future.get()) }
                    .onFailure { cont.resumeWithException(it) }
            }, ContextCompat.getMainExecutor(context))
        }
        // bindToLifecycle must run on the main thread.
        withContext(Dispatchers.Main) {
            val analysis = ImageAnalysis.Builder()
                .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                .build()
            analysis.setAnalyzer(analyzerExecutor, ::onFrame)
            provider.unbindAll()
            camera = provider.bindToLifecycle(lifecycleOwner, CameraSelector.DEFAULT_BACK_CAMERA, analysis)
            Log.i(TAG, "Camera bound (ImageAnalysis)")
        }
    }

    fun enableTorch(on: Boolean) {
        camera?.let { c ->
            runCatching { c.cameraControl.enableTorch(on) }
                .onFailure { Log.e(TAG, "Torch failed: ${it.message}") }
        }
    }

    /**
     * Runs on [analyzerExecutor]. Throttled to ~10 fps — comfortably above
     * the panel's 2 fps target, well below the camera's native analysis
     * rate, so we're not burning CPU/memory on Bitmap conversions nobody
     * reads.
     */
    private fun onFrame(image: ImageProxy) {
        try {
            val now = System.currentTimeMillis()
            if (now - lastConvertMs < 100) return
            lastConvertMs = now
            val bitmap = image.toBitmap()
            val deg = image.imageInfo.rotationDegrees
            latestFrame = if (deg != 0) {
                val m = Matrix().apply { postRotate(deg.toFloat()) }
                Bitmap.createBitmap(bitmap, 0, 0, bitmap.width, bitmap.height, m, true)
            } else bitmap
        } catch (e: Exception) {
            Log.e(TAG, "Frame conversion failed", e)
        } finally {
            image.close()
        }
    }

    suspend fun captureFrame(): Bitmap? = latestFrame

    companion object {
        private const val TAG = "CameraManager"
    }
}
