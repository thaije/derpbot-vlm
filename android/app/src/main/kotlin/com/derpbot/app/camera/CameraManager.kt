package com.derpbot.app.camera

import android.content.Context
import android.graphics.Bitmap
import android.graphics.Matrix
import android.util.Log
import androidx.camera.core.CameraSelector
import androidx.camera.core.ImageCapture
import androidx.camera.core.ImageCaptureException
import androidx.camera.core.ImageProxy
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.core.content.ContextCompat
import androidx.lifecycle.LifecycleOwner
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.suspendCancellableCoroutine
import kotlinx.coroutines.withContext
import kotlin.coroutines.resume
import kotlin.coroutines.resumeWithException

/**
 * CameraX wrapper for the RVR control loop (issue #19, Step 2).
 *
 * [bind] must be called once from a coroutine (before the control loop starts).
 * [captureFrame] is called per VLM cycle; it suspends while the shutter fires
 * and returns the rotated Bitmap, or null on error.
 */
class CameraManager(
    private val context: Context,
    private val lifecycleOwner: LifecycleOwner,
) {
    private var imageCapture: ImageCapture? = null

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
            val capture = ImageCapture.Builder()
                .setCaptureMode(ImageCapture.CAPTURE_MODE_MINIMIZE_LATENCY)
                .build()
            provider.unbindAll()
            provider.bindToLifecycle(lifecycleOwner, CameraSelector.DEFAULT_BACK_CAMERA, capture)
            imageCapture = capture
            Log.i(TAG, "Camera bound")
        }
    }

    suspend fun captureFrame(): Bitmap? = suspendCancellableCoroutine { cont ->
        val ic = imageCapture
        if (ic == null) {
            Log.w(TAG, "captureFrame called before bind()")
            cont.resume(null)
            return@suspendCancellableCoroutine
        }
        ic.takePicture(ContextCompat.getMainExecutor(context), object : ImageCapture.OnImageCapturedCallback() {
            override fun onCaptureSuccess(image: ImageProxy) {
                try {
                    val bitmap = image.toBitmap()
                    val deg = image.imageInfo.rotationDegrees
                    val rotated = if (deg != 0) {
                        val m = Matrix().apply { postRotate(deg.toFloat()) }
                        Bitmap.createBitmap(bitmap, 0, 0, bitmap.width, bitmap.height, m, true)
                    } else bitmap
                    cont.resume(rotated)
                } catch (e: Exception) {
                    Log.e(TAG, "Frame conversion failed", e)
                    cont.resume(null)
                } finally {
                    image.close()
                }
            }

            override fun onError(exception: ImageCaptureException) {
                Log.e(TAG, "captureFrame failed", exception)
                cont.resume(null)
            }
        })
    }

    companion object {
        private const val TAG = "CameraManager"
    }
}
