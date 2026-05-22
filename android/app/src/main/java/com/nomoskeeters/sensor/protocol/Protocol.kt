package com.nomoskeeters.sensor.protocol

/**
 * NoMoSkeeters Sensor Protocol v1 — constants shared by both channels.
 *
 * This is the phone (server) side of the contract the PC's `phone_sensor/`
 * package speaks. The PC opens a TCP connection to the phone on [CMD_PORT] and
 * sends `\n`-delimited JSON commands; the phone replies on the same socket and
 * pushes unsolicited events. Decoded video frames flow the other way as UDP
 * datagrams to the PC on [FRAME_PORT], one frame per datagram.
 *
 * Pure Kotlin (no Android, no JSON lib) so it is unit-testable on a plain JVM.
 *
 * Reference: phone_sensor/protocol.py, PHONE_SENSOR_BOOTSTRAP.md §2.
 */
object Protocol {
    const val VERSION = 1

    /** TCP — commands + replies + events + heartbeat. PC connects here. */
    const val CMD_PORT = 45470
    /** UDP — frame packets. Phone sends to the PC on this port. */
    const val FRAME_PORT = 45471

    // ── Commands the PC may send (phone_sensor/protocol.py COMMANDS). ──────
    const val CMD_CONNECT = "connect"
    const val CMD_DISCONNECT = "disconnect"
    const val CMD_PING = "ping"
    const val CMD_SET_ACTIVE_CAMERA = "set_active_camera"
    const val CMD_GET_CAMERA_CAPABILITIES = "get_camera_capabilities"
    const val CMD_SET_EXPOSURE_MODE = "set_exposure_mode"
    const val CMD_SET_EXPOSURE_VALUE = "set_exposure_value"
    const val CMD_SET_AF_MODE = "set_af_mode"
    const val CMD_SET_AF_REGION = "set_af_region"
    const val CMD_LOCK_FOCUS = "lock_focus"
    const val CMD_UNLOCK_FOCUS = "unlock_focus"
    const val CMD_STREAM_START = "stream_start"
    const val CMD_STREAM_STOP = "stream_stop"
    const val CMD_STREAM_SET_RESOLUTION = "stream_set_resolution"
    const val CMD_STREAM_SET_TARGET_BITRATE = "stream_set_target_bitrate"
    const val CMD_STREAM_SET_TARGET_FPS = "stream_set_target_fps"
    const val CMD_GET_STATUS = "get_status"
    const val CMD_GET_INTRINSICS = "get_intrinsics"
    const val CMD_RECORDING_START = "recording_start"
    const val CMD_RECORDING_STOP = "recording_stop"
    const val CMD_RECORDING_LIST = "recording_list"
    const val CMD_RECORDING_TRANSFER = "recording_transfer"

    // ── Unsolicited events the phone pushes (phone_sensor/protocol.py EVENTS).
    const val EV_AF_SETTLED = "event:af_settled"
    const val EV_EXPOSURE_CHANGED = "event:exposure_changed"
    const val EV_THERMAL_WARNING = "event:thermal_warning"
    const val EV_BATTERY_LOW = "event:battery_low"
    const val EV_CAMERA_UNAVAILABLE = "event:camera_unavailable"
    const val EV_CAMERA_CHANGED = "event:camera_changed"
    const val EV_STREAM_STARTED = "event:stream_started"
    const val EV_STREAM_STOPPED = "event:stream_stopped"

    // ── Stream modes (phone_sensor.protocol.StreamMode). ───────────────────
    const val MODE_RAW_YUV = "raw_yuv"
    const val MODE_H264_LOWLAT = "h264_lowlat"
    const val MODE_H264_QUALITY = "h264_quality"

    val STREAM_MODES = setOf(MODE_RAW_YUV, MODE_H264_LOWLAT, MODE_H264_QUALITY)

    /** Pixel-format strings the PC's frame decoder recognises (raw path). */
    const val FMT_NV21 = "nv21"
    const val FMT_I420 = "i420"
    /** Codec string for the H.264 elementary-stream payload. */
    const val FMT_H264 = "h264"

    /** Logical camera roles the PC addresses lenses by. */
    const val CAM_ULTRAWIDE = "ultrawide"
    const val CAM_MAIN = "main"
    const val CAM_TELEPHOTO = "telephoto"
    const val CAM_WIDE = "wide"
}
