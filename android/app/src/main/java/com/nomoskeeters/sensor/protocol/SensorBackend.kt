package com.nomoskeeters.sensor.protocol

import org.json.JSONArray
import org.json.JSONObject

/**
 * Everything [CommandRouter] needs the device to actually do. The Android
 * implementation ([com.nomoskeeters.sensor.SensorEngine]) drives Camera2,
 * MediaCodec and the UDP streamer; tests supply a fake. This split is what
 * keeps the command-dispatch logic pure and JVM-testable.
 *
 * Methods return `true`/non-null on success; the router maps that onto the
 * `ok` field the PC checks.
 */
interface SensorBackend {
    /** The capabilities manifest returned on `connect` / `get_camera_capabilities`. */
    fun capabilitiesManifest(): JSONObject

    /** Switch active lens. Blocks until the camera is open and producing
     *  frames (300-800 ms typical). */
    fun setActiveCamera(cameraId: String): Boolean

    /** Begin streaming. Returns the negotiated mode (may downgrade), or null. */
    fun startStream(mode: String, width: Int, height: Int, fps: Int): String?
    fun stopStream(): Boolean

    fun setExposureMode(mode: String): Boolean
    fun setExposureValue(shutterUs: Long, iso: Int): Boolean
    fun setAfMode(mode: String): Boolean
    fun setAfRegion(x: Double, y: Double, w: Double, h: Double): Boolean
    fun lockFocus(): Boolean
    fun unlockFocus(): Boolean

    fun setStreamResolution(width: Int, height: Int): Boolean
    fun setTargetBitrate(bitrateBps: Int): Boolean
    fun setTargetFps(fps: Int): Boolean

    /** Current camera / resolution / fps / exposure / focus / thermal / battery. */
    fun status(): JSONObject
    /** Calibrated intrinsics for [cameraId] (or the active camera if null). */
    fun intrinsics(cameraId: String?): JSONObject

    fun recordingStart(filename: String, format: String): Boolean
    fun recordingStop(): Boolean
    fun recordingList(): JSONArray
    fun recordingTransfer(filename: String): Boolean
}
