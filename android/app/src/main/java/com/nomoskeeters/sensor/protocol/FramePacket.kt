package com.nomoskeeters.sensor.protocol

import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.charset.StandardCharsets

/**
 * Frame-channel wire format — the binary header the PC's
 * `phone_sensor.protocol.parse_frame_packet` expects, byte-for-byte.
 *
 * Little-endian, Python struct `"<4sQQHHHHIHH"` (header = [HEADER_SIZE] = 36 B):
 *
 * ```
 *   uint32 magic        = "NMS2"
 *   uint64 frame_id      — monotonic; identical across a frame's chunks
 *   uint64 capture_ts    — microseconds, phone monotonic clock
 *   uint16 cam_id_len
 *   uint16 fmt_len
 *   uint16 width
 *   uint16 height
 *   uint32 payload_len   — bytes of payload in THIS chunk
 *   uint16 chunk_index   — 0-based position of this chunk within the frame
 *   uint16 chunk_count   — total chunks for the frame (>= 1)
 *   <camera_id bytes>   (utf-8, repeated in every chunk)
 *   <format bytes>      (utf-8, e.g. "nv21" / "h264", repeated in every chunk)
 *   <payload bytes>     (this chunk's slice of the frame)
 * ```
 *
 * One frame is split into one or more chunks, each its own UDP datagram, so a
 * frame larger than the IPv4 UDP ceiling ([MAX_UDP_PAYLOAD]) still gets through;
 * the PC reassembles by `frame_id`. Each packed *chunk* MUST stay within the
 * ceiling — split the payload on [MAX_CHUNK_PAYLOAD] (see
 * `stream.StreamPipeline`). An unfragmented frame is just `chunkCount == 1`.
 *
 * Pure Kotlin (java.nio only) → unit-testable on a plain JVM.
 */
object FramePacket {
    private val MAGIC = byteArrayOf('N'.code.toByte(), 'M'.code.toByte(),
        'S'.code.toByte(), '2'.code.toByte())
    const val HEADER_SIZE = 36
    /** 65535 − 8 (UDP) − 20 (IPv4) = 65507 datagram ceiling. */
    const val MAX_UDP_PAYLOAD = 65507
    /** Max payload bytes per chunk; leaves headroom for header + cam/fmt
     *  strings under [MAX_UDP_PAYLOAD]. The phone splits frames on this. */
    const val MAX_CHUNK_PAYLOAD = 60000

    /**
     * Encode one frame *chunk*. [payload] is packed in full unless
     * [payloadOffset]/[payloadLen] select a slice (lets the pipeline carve a
     * frame into chunks without copying). [chunkIndex]/[chunkCount] place this
     * chunk within the frame; the defaults (`0` / `1`) pack an unfragmented
     * frame.
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
        chunkIndex: Int = 0,
        chunkCount: Int = 1,
    ): ByteArray {
        val cid = cameraId.toByteArray(StandardCharsets.UTF_8)
        val fmtBytes = fmt.toByteArray(StandardCharsets.UTF_8)
        require(cid.size <= 0xFFFF && fmtBytes.size <= 0xFFFF) {
            "camera_id or format too long for the header"
        }
        require(chunkCount in 1..0xFFFF) { "chunk_count out of range: $chunkCount" }
        require(chunkIndex in 0 until chunkCount) {
            "chunk_index $chunkIndex out of range for count $chunkCount"
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
        buf.putShort((chunkIndex and 0xFFFF).toShort())
        buf.putShort((chunkCount and 0xFFFF).toShort())
        buf.put(cid)
        buf.put(fmtBytes)
        buf.put(payload, payloadOffset, payloadLen)
        return buf.array()
    }
}
