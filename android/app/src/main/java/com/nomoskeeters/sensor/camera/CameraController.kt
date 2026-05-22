package com.nomoskeeters.sensor.camera

import android.annotation.SuppressLint
import android.content.Context
import android.graphics.Rect
import android.hardware.camera2.CameraCaptureSession
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraDevice
import android.hardware.camera2.CameraManager
import android.hardware.camera2.CaptureRequest
import android.hardware.camera2.CaptureResult
import android.hardware.camera2.TotalCaptureResult
import android.hardware.camera2.params.MeteringRectangle
import android.os.Handler
import android.os.HandlerThread
import android.util.Log
import android.util.Range
import android.view.Surface
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit
import kotlin.math.roundToInt

/**
 * Camera2 session manager. One physical camera open at a time, rendering into
 * one or more target [Surface]s (the H.264 encoder's input surface, or a raw
 * YUV ImageReader surface). Owns the control surface the PC drives: AF mode and
 * metering region, focus lock, and manual/auto exposure.
 *
 * Camera2 (rather than CameraX) is deliberate — the smart-sensor contract needs
 * per-lens physical switching, AF metering rectangles, and manual ISO/shutter,
 * all of which Camera2 exposes directly and at lower latency.
 *
 * [configure] blocks until the session is streaming (or fails), because the
 * PC's `stream_start` / `set_active_camera` replies are expected only once the
 * camera is actually producing frames.
 */
class CameraController(
    context: Context,
    private val listener: Listener,
) {
    interface Listener {
        /** AF reached a terminal state after a trigger. */
        fun onAfSettled(state: String, region: FloatArray?)
        fun onCameraError(message: String)
    }

    private val cm = context.getSystemService(Context.CAMERA_SERVICE) as CameraManager
    private val bgThread = HandlerThread("phone-camera").apply { start() }
    private val bgHandler = Handler(bgThread.looper)

    private var device: CameraDevice? = null
    private var session: CameraCaptureSession? = null
    private var builder: CaptureRequest.Builder? = null
    private var surfaces: List<Surface> = emptyList()
    private var physicalId: String? = null
    private var fpsRange: Range<Int> = Range(30, 30)

    // ── Control state (re-applied across lens switches). ───────────────────
    @Volatile var manualExposure = false; private set
    @Volatile private var shutterNs = 0L
    @Volatile private var iso = 0
    @Volatile var afMode = "auto"; private set
    @Volatile private var afRegionNorm: FloatArray? = null
    @Volatile var focusLocked = false; private set
    private var awaitingAfSettle = false

    /**
     * Open [physicalId] and start a repeating request to [targets] at [fps].
     * Blocking; returns false on any failure within [timeoutMs].
     */
    @SuppressLint("MissingPermission")
    fun configure(
        physicalId: String,
        targets: List<Surface>,
        fps: Int,
        timeoutMs: Long = 4000L,
    ): Boolean {
        closeSessionAndDevice()
        this.physicalId = physicalId
        this.surfaces = targets
        this.fpsRange = pickFpsRange(physicalId, fps)

        val opened = CountDownLatch(1)
        var ok = false
        try {
            cm.openCamera(physicalId, object : CameraDevice.StateCallback() {
                override fun onOpened(camera: CameraDevice) {
                    device = camera
                    ok = startSession(camera, opened)
                    if (!ok) opened.countDown()
                }
                override fun onDisconnected(camera: CameraDevice) {
                    Log.w(TAG, "camera $physicalId disconnected")
                    camera.close(); if (device === camera) device = null
                    opened.countDown()
                }
                override fun onError(camera: CameraDevice, error: Int) {
                    Log.e(TAG, "camera $physicalId error $error")
                    camera.close(); if (device === camera) device = null
                    listener.onCameraError("camera open error $error")
                    opened.countDown()
                }
            }, bgHandler)
        } catch (e: Exception) {
            Log.e(TAG, "openCamera($physicalId) threw: ${e.message}")
            return false
        }
        return try {
            opened.await(timeoutMs, TimeUnit.MILLISECONDS) && ok && session != null
        } catch (e: InterruptedException) {
            false
        }
    }

    private fun startSession(camera: CameraDevice, latch: CountDownLatch): Boolean {
        return try {
            val b = camera.createCaptureRequest(CameraDevice.TEMPLATE_RECORD)
            for (s in surfaces) b.addTarget(s)
            builder = b
            applyControls(b)
            @Suppress("DEPRECATION")
            camera.createCaptureSession(surfaces, object :
                CameraCaptureSession.StateCallback() {
                override fun onConfigured(s: CameraCaptureSession) {
                    session = s
                    try {
                        s.setRepeatingRequest(b.build(), captureCallback, bgHandler)
                    } catch (e: Exception) {
                        Log.e(TAG, "setRepeatingRequest failed: ${e.message}")
                    }
                    latch.countDown()
                }
                override fun onConfigureFailed(s: CameraCaptureSession) {
                    Log.e(TAG, "session configure failed for $physicalId")
                    listener.onCameraError("session configure failed")
                    latch.countDown()
                }
            }, bgHandler)
            true
        } catch (e: Exception) {
            Log.e(TAG, "startSession threw: ${e.message}")
            false
        }
    }

    // ── Control setters — mutate the builder and re-issue the repeat. ──────

    fun setExposureMode(mode: String): Boolean {
        manualExposure = mode == "manual"
        return reapply()
    }

    fun setExposureValue(shutterUs: Long, isoValue: Int): Boolean {
        shutterNs = shutterUs * 1000L
        iso = isoValue
        manualExposure = true
        return reapply()
    }

    fun setAfMode(mode: String): Boolean {
        afMode = mode
        focusLocked = mode == "locked"
        return reapply()
    }

    fun setAfRegion(x: Double, y: Double, w: Double, h: Double): Boolean {
        afRegionNorm = floatArrayOf(x.toFloat(), y.toFloat(), w.toFloat(), h.toFloat())
        if (afMode == "locked") afMode = "auto"      // a new region implies a re-focus
        focusLocked = false
        val ok = reapply()
        triggerAf()
        return ok
    }

    fun lockFocus(): Boolean {
        triggerAf()
        focusLocked = true
        afMode = "locked"
        return reapply()
    }

    fun unlockFocus(): Boolean {
        focusLocked = false
        afMode = "auto"
        return reapply()
    }

    private fun reapply(): Boolean {
        val b = builder ?: return false
        val s = session ?: return false
        applyControls(b)
        return try {
            s.setRepeatingRequest(b.build(), captureCallback, bgHandler); true
        } catch (e: Exception) {
            Log.w(TAG, "reapply failed: ${e.message}"); false
        }
    }

    private fun triggerAf() {
        val b = builder ?: return
        val s = session ?: return
        try {
            awaitingAfSettle = true
            b.set(CaptureRequest.CONTROL_AF_TRIGGER, CaptureRequest.CONTROL_AF_TRIGGER_START)
            s.capture(b.build(), captureCallback, bgHandler)
            b.set(CaptureRequest.CONTROL_AF_TRIGGER, CaptureRequest.CONTROL_AF_TRIGGER_IDLE)
            s.setRepeatingRequest(b.build(), captureCallback, bgHandler)
        } catch (e: Exception) {
            Log.w(TAG, "AF trigger failed: ${e.message}")
        }
    }

    private fun applyControls(b: CaptureRequest.Builder) {
        b.set(CaptureRequest.CONTROL_MODE, CaptureRequest.CONTROL_MODE_AUTO)

        if (manualExposure && shutterNs > 0 && iso > 0) {
            b.set(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_OFF)
            b.set(CaptureRequest.SENSOR_EXPOSURE_TIME, shutterNs)
            b.set(CaptureRequest.SENSOR_SENSITIVITY, iso)
        } else {
            b.set(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_ON)
            b.set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, fpsRange)
        }

        val af = when (afMode) {
            "manual" -> CaptureRequest.CONTROL_AF_MODE_OFF
            "locked" -> CaptureRequest.CONTROL_AF_MODE_AUTO   // held after trigger
            else -> CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_VIDEO
        }
        b.set(CaptureRequest.CONTROL_AF_MODE, af)

        afRegionNorm?.let { region ->
            meteringRect(region)?.let {
                b.set(CaptureRequest.CONTROL_AF_REGIONS, arrayOf(it))
                b.set(CaptureRequest.CONTROL_AE_REGIONS, arrayOf(it))
            }
        }
    }

    private fun meteringRect(norm: FloatArray): MeteringRectangle? {
        val id = physicalId ?: return null
        val arr = cm.getCameraCharacteristics(id)
            .get(CameraCharacteristics.SENSOR_INFO_ACTIVE_ARRAY_SIZE) ?: return null
        val (nx, ny, nw, nh) = norm
        val left = (nx * arr.width()).roundToInt().coerceIn(0, arr.width() - 1)
        val top = (ny * arr.height()).roundToInt().coerceIn(0, arr.height() - 1)
        val w = (nw * arr.width()).roundToInt().coerceIn(1, arr.width() - left)
        val h = (nh * arr.height()).roundToInt().coerceIn(1, arr.height() - top)
        return MeteringRectangle(Rect(left, top, left + w, top + h),
            MeteringRectangle.METERING_WEIGHT_MAX)
    }

    private fun pickFpsRange(id: String, fps: Int): Range<Int> {
        return try {
            val ranges = cm.getCameraCharacteristics(id)
                .get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES)
                ?: return Range(fps, fps)
            // Prefer a fixed [fps,fps]; else the range whose upper == fps; else
            // the highest range that doesn't exceed fps.
            ranges.firstOrNull { it.lower == fps && it.upper == fps }
                ?: ranges.filter { it.upper == fps }.maxByOrNull { it.lower }
                ?: ranges.filter { it.upper <= fps }.maxByOrNull { it.upper }
                ?: ranges.minByOrNull { it.upper }
                ?: Range(fps, fps)
        } catch (e: Exception) {
            Range(fps, fps)
        }
    }

    private val captureCallback = object : CameraCaptureSession.CaptureCallback() {
        override fun onCaptureCompleted(
            s: CameraCaptureSession, request: CaptureRequest, result: TotalCaptureResult
        ) {
            if (!awaitingAfSettle) return
            val afState = result.get(CaptureResult.CONTROL_AF_STATE) ?: return
            if (afState == CaptureResult.CONTROL_AF_STATE_FOCUSED_LOCKED ||
                afState == CaptureResult.CONTROL_AF_STATE_NOT_FOCUSED_LOCKED) {
                awaitingAfSettle = false
                val state = if (afState == CaptureResult.CONTROL_AF_STATE_FOCUSED_LOCKED)
                    "settled" else "searching"
                listener.onAfSettled(state, afRegionNorm)
            }
        }
    }

    /** Stop the session + close the device, but keep the worker thread alive
     *  so a subsequent [configure] (stream restart / lens switch) is cheap. */
    fun stopSession() {
        closeSessionAndDevice()
    }

    fun close() {
        closeSessionAndDevice()
        bgThread.quitSafely()
    }

    private fun closeSessionAndDevice() {
        try { session?.close() } catch (_: Exception) {}
        session = null
        try { device?.close() } catch (_: Exception) {}
        device = null
        builder = null
    }

    companion object {
        private const val TAG = "PhoneCameraCtl"
    }
}

private operator fun FloatArray.component1() = this[0]
private operator fun FloatArray.component2() = this[1]
private operator fun FloatArray.component3() = this[2]
private operator fun FloatArray.component4() = this[3]
