package com.nomoskeeters.sensor.protocol

import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.charset.StandardCharsets

/**
 * Frame-channel wire format — the binary header the PC's
 * `phone_sensor.protocol.parse_frame_packet` expects, byte-for-byte.
 *
 * Little-endian, Python struct `"<4sQQHHHHI"` (header = [HEADER_SIZE] = 32 B):
 *
 * ```
 *   uint32 magic       = "NMS1"
 *   uint64 frame_id     — monotonic
 *   uint64 capture_ts   — microseconds, phone monotonic clock
 *   uint16 cam_id_len
 *   uint16 fmt_len
 *   uint16 width
 *   uint16 height
 *   uint32 payload_len
 *   <camera_id bytes>  (utf-8)
 *   <format bytes>     (utf-8, e.g. "nv21" / "h264")
 *   <payload bytes>
 * ```
 *
 * The PC does no datagram reassembly — one frame is one UDP packet — so a
 * packed packet MUST stay within the IPv4 UDP payload ceiling
 * ([MAX_UDP_PAYLOAD]). [pack] returns the bytes regardless; the streamer is
 * responsible for the size guard.
 *
 * Pure Kotlin (java.nio only) → unit-testable on a plain JVM.
 */
object FramePacket {
    private val MAGIC = byteArrayOf('N'.code.toByte(), 'M'.code.toByte(),
        'S'.code.toByte(), '1'.code.toByte())
    const val HEADER_SIZE = 32
    /** 65535 − 8 (UDP) − 20 (IPv4) = 65507. Frames larger than this can't be
     *  carried in a single datagram and will be dropped by the streamer. */
    const val MAX_UDP_PAYLOAD = 65507

    /**
     * Encode one frame packet. [payload] is sent in full unless [payloadLen]
     * is given (lets callers reuse an oversized backing array, e.g. a codec
     * output buffer copied into a pool slot).
     */
    fun pack(
        frameId: Long,
        captureTsUs: Long,
        cameraId: String,
        fmt: String,
        width: Int,
        height: Int,
        payload: ByteArray,
        payloadOffset: Int = 0,
        payloadLen: Int = payload.size,
    ): ByteArray {
        val cid = cameraId.toByteArray(StandardCharsets.UTF_8)
        val fmtBytes = fmt.toByteArray(StandardCharsets.UTF_8)
        require(cid.size <= 0xFFFF && fmtBytes.size <= 0xFFFF) {
            "camera_id or format too long for the header"
        }
        val total = HEADER_SIZE + cid.size + fmtBytes.size + payloadLen
        val buf = ByteBuffer.allocate(total).order(ByteOrder.LITTLE_ENDIAN)
        buf.put(MAGIC)
        buf.putLong(frameId)
        buf.putLong(captureTsUs)
        buf.putShort((cid.size and 0xFFFF).toShort())
        buf.putShort((fmtBytes.size and 0xFFFF).toShort())
        buf.putShort((width and 0xFFFF).toShort())
        buf.putShort((height and 0xFFFF).toShort())
        buf.putInt(payloadLen)
        buf.put(cid)
        buf.put(fmtBytes)
        buf.put(payload, payloadOffset, payloadLen)
        return buf.array()
    }
}
