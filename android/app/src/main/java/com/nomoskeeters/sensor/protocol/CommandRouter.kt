package com.nomoskeeters.sensor.protocol

import org.json.JSONException
import org.json.JSONObject

/**
 * Parses one JSON command from the PC and produces the reply, delegating the
 * actual work to a [SensorBackend].
 *
 * Reply convention (matches the PC's `_FakePhone` test double and what
 * `PhoneSensorClient` reads): `{"type":"<cmd>_reply","cmd_id":<echoed>,
 * "ok":<bool>, ...}`. The PC routes replies to waiting callers by `cmd_id`,
 * so every reply echoes it. Unsolicited events (built elsewhere) deliberately
 * carry NO `cmd_id` so the PC never mistakes one for a reply.
 *
 * Pure logic (org.json only) → unit-testable without an emulator.
 */
class CommandRouter(private val backend: SensorBackend) {

    /** True after a `disconnect` command — the server should close the link. */
    @Volatile
    var disconnectRequested: Boolean = false
        private set

    /** Parse a raw line into a command object. Throws [JSONException] on
     *  malformed input or a missing `type` (the server drops the line). */
    fun parse(line: String): JSONObject {
        val obj = JSONObject(line)
        if (!obj.has("type")) throw JSONException("phone command missing 'type'")
        return obj
    }

    /**
     * Handle one already-parsed command. Returns the reply object to send back,
     * or null for commands that warrant no reply. Never throws — a backend that
     * fails yields `ok=false`.
     */
    fun handle(msg: JSONObject): JSONObject? {
        val type = msg.optString("type")
        val reply = JSONObject()
        reply.put("type", "${type}_reply")
        if (msg.has("cmd_id")) reply.put("cmd_id", msg.get("cmd_id"))

        return try {
            when (type) {
                Protocol.CMD_CONNECT -> {
                    reply.put("ok", true)
                    reply.put("protocol_version", Protocol.VERSION)
                    reply.put("capabilities", backend.capabilitiesManifest())
                }
                Protocol.CMD_PING -> reply.put("ok", true)
                Protocol.CMD_DISCONNECT -> {
                    backend.stopStream()
                    disconnectRequested = true
                    reply.put("ok", true)
                }
                Protocol.CMD_GET_CAMERA_CAPABILITIES ->
                    reply.put("ok", true)
                        .put("capabilities", backend.capabilitiesManifest())
                Protocol.CMD_SET_ACTIVE_CAMERA -> {
                    val cam = msg.optString("camera_id", Protocol.CAM_MAIN)
                    reply.put("ok", backend.setActiveCamera(cam))
                    reply.put("camera_id", cam)
                }
                Protocol.CMD_STREAM_START -> {
                    val mode = msg.optString("mode", Protocol.MODE_H264_LOWLAT)
                    val w = msg.optInt("width", 1920)
                    val h = msg.optInt("height", 1080)
                    val fps = msg.optInt("fps", 60)
                    val negotiated = backend.startStream(mode, w, h, fps)
                    reply.put("ok", negotiated != null)
                    reply.put("mode", negotiated ?: mode)
                }
                Protocol.CMD_STREAM_STOP -> reply.put("ok", backend.stopStream())
                Protocol.CMD_STREAM_SET_RESOLUTION ->
                    reply.put("ok", backend.setStreamResolution(
                        msg.optInt("width", 1920), msg.optInt("height", 1080)))
                Protocol.CMD_STREAM_SET_TARGET_BITRATE ->
                    reply.put("ok", backend.setTargetBitrate(
                        msg.optInt("bitrate_bps", 0)))
                Protocol.CMD_STREAM_SET_TARGET_FPS ->
                    reply.put("ok", backend.setTargetFps(msg.optInt("fps", 0)))
                Protocol.CMD_SET_EXPOSURE_MODE ->
                    reply.put("ok", backend.setExposureMode(
                        msg.optString("mode", "auto")))
                Protocol.CMD_SET_EXPOSURE_VALUE ->
                    reply.put("ok", backend.setExposureValue(
                        msg.optLong("shutter_us", 0L), msg.optInt("iso", 0)))
                Protocol.CMD_SET_AF_MODE ->
                    reply.put("ok", backend.setAfMode(msg.optString("mode", "auto")))
                Protocol.CMD_SET_AF_REGION ->
                    reply.put("ok", backend.setAfRegion(
                        msg.optDouble("x", 0.0), msg.optDouble("y", 0.0),
                        msg.optDouble("w", 0.0), msg.optDouble("h", 0.0)))
                Protocol.CMD_LOCK_FOCUS -> reply.put("ok", backend.lockFocus())
                Protocol.CMD_UNLOCK_FOCUS -> reply.put("ok", backend.unlockFocus())
                Protocol.CMD_GET_STATUS -> {
                    // The PC returns the whole reply dict to its caller, so the
                    // status fields go in alongside ok.
                    val status = backend.status()
                    for (key in status.keys()) reply.put(key, status.get(key))
                    reply.put("ok", true)
                }
                Protocol.CMD_GET_INTRINSICS -> {
                    val cam = if (msg.has("camera_id"))
                        msg.optString("camera_id") else null
                    reply.put("ok", true)
                    reply.put("intrinsics", backend.intrinsics(cam))
                }
                Protocol.CMD_RECORDING_START ->
                    reply.put("ok", backend.recordingStart(
                        msg.optString("filename", "rec.mp4"),
                        msg.optString("format", "mp4")))
                Protocol.CMD_RECORDING_STOP ->
                    reply.put("ok", backend.recordingStop())
                Protocol.CMD_RECORDING_LIST ->
                    reply.put("ok", true)
                        .put("recordings", backend.recordingList())
                Protocol.CMD_RECORDING_TRANSFER ->
                    reply.put("ok", backend.recordingTransfer(
                        msg.optString("filename", "")))
                else -> {
                    // Unknown command — answer honestly rather than hang the PC.
                    reply.put("ok", false)
                    reply.put("error", "unknown command: $type")
                }
            }
            reply
        } catch (e: Exception) {
            reply.put("ok", false)
            reply.put("error", e.message ?: e.javaClass.simpleName)
            reply
        }
    }

    companion object {
        /** Build an unsolicited event object (no `cmd_id`). */
        fun event(type: String, fields: Map<String, Any?> = emptyMap()): JSONObject {
            val o = JSONObject()
            o.put("type", type)
            for ((k, v) in fields) o.put(k, v)
            return o
        }
    }
}
