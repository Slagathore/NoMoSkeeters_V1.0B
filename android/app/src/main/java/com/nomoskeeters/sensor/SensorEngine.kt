package com.nomoskeeters.sensor

import android.content.Context
import android.content.pm.PackageManager
import android.util.Log
import android.view.Surface
import androidx.core.content.ContextCompat
import com.nomoskeeters.sensor.camera.CameraController
import com.nomoskeeters.sensor.camera.CameraEnumerator
import com.nomoskeeters.sensor.net.CommandServer
import com.nomoskeeters.sensor.net.FrameStreamer
import com.nomoskeeters.sensor.protocol.CommandRouter
import com.nomoskeeters.sensor.protocol.Protocol
import com.nomoskeeters.sensor.protocol.SensorBackend
import com.nomoskeeters.sensor.stream.StreamPipeline
import com.nomoskeeters.sensor.thermal.ThermalMonitor
import org.json.JSONArray
import org.json.JSONObject
import java.net.InetAddress

/** Snapshot of engine state for the UI. */
data class EngineStatus(
    val connected: Boolean,
    val peer: String?,
    val activeCamera: String,
    val streaming: Boolean,
    val mode: String,
    val width: Int,
    val height: Int,
    val fps: Int,
    val thermal: String,
    val battery: Int,
)

/**
 * The phone-side brain. Wires the TCP command server, UDP frame streamer,
 * Camera2 controller, encoder pipeline and thermal monitor into one object that
 * implements [SensorBackend] (what the [CommandRouter] calls) and
 * [CameraController.Listener] (AF/error callbacks → protocol events).
 *
 * Dumb-on-purpose: it executes PC commands and reports state. It never decides
 * what to shoot, when to switch lenses, or what to record — that's the PC.
 */
class SensorEngine(private val context: Context) :
    SensorBackend, CameraController.Listener {

    private val enumerator = CameraEnumerator(context)
    private val streamer = FrameStreamer()
    private val pipeline = StreamPipeline(streamer)
    private val camera = CameraController(context, this)
    private lateinit var thermal: ThermalMonitor
    private lateinit var router: CommandRouter
    private lateinit var server: CommandServer

    // ── Mirrored state (for get_status + reconfigure). ─────────────────────
    @Volatile private var activeCameraRole = Protocol.CAM_MAIN
    @Volatile private var streaming = false
    @Volatile private var currentSurface: Surface? = null
    @Volatile private var exposureMode = "auto"
    @Volatile private var iso = 0
    @Volatile private var shutterUs = 0L
    @Volatile private var afMode = "auto"
    @Volatile private var focusLocked = false
    @Volatile private var lastBitrate = 0
    @Volatile private var peerAddr: String? = null

    var statusListener: ((EngineStatus) -> Unit)? = null

    // ── Lifecycle ──────────────────────────────────────────────────────────

    fun start() {
        streamer.start()
        thermal = ThermalMonitor(
            context,
            onThermal = { state ->
                server.sendEvent(CommandRouter.event(Protocol.EV_THERMAL_WARNING, mapOf(
                    "state" to state,
                    "battery_percent" to thermal.batteryPercent(),
                    "camera_id" to activeCameraRole,
                )))
                notifyStatus()
            },
            onBatteryLow = { pct ->
                server.sendEvent(CommandRouter.event(Protocol.EV_BATTERY_LOW, mapOf(
                    "battery_percent" to pct)))
            },
        )
        router = CommandRouter(this)
        server = CommandServer(
            router = router,
            onClientConnected = { addr -> onPcConnected(addr) },
            onClientDisconnected = { onPcDisconnected() },
            onLinkIdle = { enterSafeState() },
        )
        thermal.start()
        server.start()
        notifyStatus()
        Log.i(TAG, "engine up — TCP ${Protocol.CMD_PORT} / UDP ${Protocol.FRAME_PORT}")
    }

    fun stop() {
        stopStream()
        if (::server.isInitialized) server.stop()
        if (::thermal.isInitialized) thermal.stop()
        camera.close()
        streamer.stop()
    }

    private fun onPcConnected(addr: InetAddress) {
        peerAddr = addr.hostAddress
        streamer.setDestination(addr)
        notifyStatus()
    }

    private fun onPcDisconnected() {
        stopStream()
        streamer.setDestination(null)
        peerAddr = null
        notifyStatus()
    }

    /** Heartbeat went silent — drop to a safe state but keep listening. */
    private fun enterSafeState() {
        Log.w(TAG, "safe state: stopping stream until PC returns")
        stopStream()
        notifyStatus()
    }

    private fun hasCameraPermission(): Boolean =
        ContextCompat.checkSelfPermission(context, android.Manifest.permission.CAMERA) ==
            PackageManager.PERMISSION_GRANTED

    // ── SensorBackend ──────────────────────────────────────────────────────

    override fun capabilitiesManifest(): JSONObject = enumerator.manifest()

    override fun setActiveCamera(cameraId: String): Boolean {
        val id = enumerator.physicalIdFor(cameraId) ?: run {
            Log.w(TAG, "unknown camera role: $cameraId"); return false
        }
        activeCameraRole = cameraId
        pipeline.activeCameraRole = cameraId
        val surface = currentSurface
        if (streaming && surface != null) {
            val ok = camera.configure(id, listOf(surface), pipeline.fps)
            if (ok) emitCameraChanged(cameraId)
            notifyStatus()
            return ok
        }
        // Not streaming yet — role recorded; takes effect at stream_start.
        notifyStatus()
        return true
    }

    override fun startStream(mode: String, width: Int, height: Int, fps: Int): String? {
        if (!hasCameraPermission()) {
            Log.e(TAG, "stream_start denied — CAMERA permission not granted")
            return null
        }
        if (mode !in Protocol.STREAM_MODES) {
            Log.e(TAG, "unknown stream mode: $mode"); return null
        }
        val bitrate = if (lastBitrate > 0) lastBitrate else defaultBitrate(width, height)
        val surface = pipeline.start(mode, width, height, fps, bitrate) ?: return null
        currentSurface = surface
        val id = enumerator.physicalIdFor(activeCameraRole)
            ?: enumerator.physicalIdFor(Protocol.CAM_MAIN)
        if (id == null) { pipeline.stop(); currentSurface = null; return null }
        if (!camera.configure(id, listOf(surface), fps)) {
            pipeline.stop(); currentSurface = null
            return null
        }
        streaming = true
        server.sendEvent(CommandRouter.event(Protocol.EV_STREAM_STARTED, mapOf(
            "mode" to pipeline.mode, "width" to width, "height" to height, "fps" to fps)))
        notifyStatus()
        return pipeline.mode
    }

    override fun stopStream(): Boolean {
        if (!streaming && !pipeline.isRunning) return true
        camera.stopSession()
        pipeline.stop()
        currentSurface = null
        streaming = false
        if (::server.isInitialized)
            server.sendEvent(CommandRouter.event(Protocol.EV_STREAM_STOPPED))
        notifyStatus()
        return true
    }

    override fun setExposureMode(mode: String): Boolean {
        exposureMode = mode
        camera.setExposureMode(mode)
        return true
    }

    override fun setExposureValue(shutterUs: Long, iso: Int): Boolean {
        this.shutterUs = shutterUs
        this.iso = iso
        exposureMode = "manual"
        camera.setExposureValue(shutterUs, iso)
        return true
    }

    override fun setAfMode(mode: String): Boolean {
        afMode = mode
        focusLocked = mode == "locked"
        camera.setAfMode(mode)
        return true
    }

    override fun setAfRegion(x: Double, y: Double, w: Double, h: Double): Boolean {
        camera.setAfRegion(x, y, w, h)
        return true
    }

    override fun lockFocus(): Boolean {
        focusLocked = true
        camera.lockFocus()
        return true
    }

    override fun unlockFocus(): Boolean {
        focusLocked = false
        camera.unlockFocus()
        return true
    }

    override fun setStreamResolution(width: Int, height: Int): Boolean {
        // Mid-stream resolution change: restart the sink + camera. The PC must
        // also resize its decoder (its h264 frame size is fixed per stream).
        if (!streaming) return true
        return startStream(pipeline.mode, width, height, pipeline.fps) != null
    }

    override fun setTargetBitrate(bitrateBps: Int): Boolean {
        if (bitrateBps <= 0) return false
        lastBitrate = bitrateBps
        pipeline.setBitrate(bitrateBps)
        return true
    }

    override fun setTargetFps(fps: Int): Boolean {
        if (fps <= 0) return false
        if (!streaming) return true
        return startStream(pipeline.mode, pipeline.width, pipeline.height, fps) != null
    }

    override fun status(): JSONObject = JSONObject().apply {
        put("camera_id", activeCameraRole)
        put("streaming", streaming)
        put("mode", pipeline.mode)
        put("width", pipeline.width)
        put("height", pipeline.height)
        put("fps", pipeline.fps)
        put("bitrate_bps", pipeline.bitrateBps)
        put("exposure_mode", exposureMode)
        put("iso", iso)
        put("shutter_us", shutterUs)
        put("af_mode", afMode)
        put("focus_locked", focusLocked)
        put("thermal_state", if (::thermal.isInitialized) thermal.thermalState else "none")
        put("battery_percent", if (::thermal.isInitialized) thermal.batteryPercent() else -1)
        put("recording", false)
    }

    override fun intrinsics(cameraId: String?): JSONObject = JSONObject().apply {
        // Intrinsics are calibrated PC-side (PHONE_SENSOR_BOOTSTRAP §5); the
        // phone reports only which lens this would be for.
        put("camera_id", cameraId ?: activeCameraRole)
        put("calibrated_on", "pc")
    }

    // Local recording is unsupported in v1: the PC's phone_sensor package has
    // no file-receive path for recording_transfer, so we answer honestly rather
    // than pretend. The extension point is a MediaMuxer tee off the H.264
    // pipeline (see PHONE_APP.md "What's deferred").
    override fun recordingStart(filename: String, format: String): Boolean = false
    override fun recordingStop(): Boolean = false
    override fun recordingList(): JSONArray = JSONArray()
    override fun recordingTransfer(filename: String): Boolean = false

    // ── CameraController.Listener ──────────────────────────────────────────

    override fun onAfSettled(state: String, region: FloatArray?) {
        val fields = mutableMapOf<String, Any?>(
            "state" to state, "camera_id" to activeCameraRole)
        if (region != null) fields["region"] = JSONArray(region.map { it.toDouble() })
        server.sendEvent(CommandRouter.event(Protocol.EV_AF_SETTLED, fields))
    }

    override fun onCameraError(message: String) {
        if (::server.isInitialized)
            server.sendEvent(CommandRouter.event(Protocol.EV_CAMERA_UNAVAILABLE, mapOf(
                "reason" to message, "camera_id" to activeCameraRole)))
    }

    // ── Helpers ──────────────────────────────────────────────────────────

    private fun emitCameraChanged(role: String) {
        val entry = enumerator.cameraEntry(role)
        server.sendEvent(CommandRouter.event(Protocol.EV_CAMERA_CHANGED, mapOf(
            "camera_id" to role,
            "width" to pipeline.width,
            "height" to pipeline.height,
            "fov_h_deg" to (entry?.optDouble("fov_h_deg", 0.0) ?: 0.0),
            "has_optical_zoom" to (entry?.optBoolean("has_optical_zoom", false) ?: false),
            "optical_zoom_factor" to (entry?.optDouble("optical_zoom_factor", 1.0) ?: 1.0),
        )))
    }

    private fun defaultBitrate(width: Int, height: Int): Int {
        val px = width.toLong() * height
        return when {
            px >= 1920L * 1080 -> 6_000_000   // 1080p — kept moderate so frames
                                              // stay inside one UDP datagram
            px >= 1280L * 720 -> 3_000_000
            else -> 1_500_000
        }
    }

    private fun notifyStatus() {
        statusListener?.invoke(
            EngineStatus(
                connected = ::server.isInitialized && server.isConnected,
                peer = peerAddr,
                activeCamera = activeCameraRole,
                streaming = streaming,
                mode = pipeline.mode,
                width = pipeline.width,
                height = pipeline.height,
                fps = pipeline.fps,
                thermal = if (::thermal.isInitialized) thermal.thermalState else "none",
                battery = if (::thermal.isInitialized) thermal.batteryPercent() else -1,
            )
        )
    }

    companion object {
        private const val TAG = "PhoneEngine"
    }
}
