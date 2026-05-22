package com.nomoskeeters.sensor.stream

import android.util.Log
import android.view.Surface
import com.nomoskeeters.sensor.encoder.H264Encoder
import com.nomoskeeters.sensor.encoder.RawYuvReader
import com.nomoskeeters.sensor.net.FrameStreamer
import com.nomoskeeters.sensor.protocol.FramePacket
import com.nomoskeeters.sensor.protocol.Protocol
import java.util.concurrent.atomic.AtomicLong

/**
 * Owns the active stream sink (H.264 encoder or raw-YUV reader), tags each
 * frame with the protocol header, and hands the packed datagram to the
 * [FrameStreamer]. The camera renders into the [Surface] this returns from
 * [start]; the engine wires that up.
 *
 * Frames are tagged with [activeCameraRole] so the PC sees the lens identity
 * (and notices a mid-stream lens switch) per packet.
 */
class StreamPipeline(private val streamer: FrameStreamer) {

    @Volatile var activeCameraRole: String = Protocol.CAM_MAIN

    private var encoder: H264Encoder? = null
    private var rawReader: RawYuvReader? = null
    private val frameId = AtomicLong(0)

    @Volatile var mode: String = Protocol.MODE_H264_LOWLAT; private set
    @Volatile var width: Int = 0; private set
    @Volatile var height: Int = 0; private set
    @Volatile var fps: Int = 0; private set
    @Volatile var bitrateBps: Int = 0; private set

    /**
     * Build the sink for [mode] and return the camera target surface, or null
     * on failure. [bitrateBps] applies to h264 modes only.
     */
    fun start(mode: String, width: Int, height: Int, fps: Int, bitrateBps: Int): Surface? {
        stop()
        this.mode = mode
        this.width = width
        this.height = height
        this.fps = fps
        this.bitrateBps = bitrateBps
        frameId.set(0)

        return when (mode) {
            Protocol.MODE_RAW_YUV -> {
                val reader = RawYuvReader(width, height) { nv21, ts ->
                    emit(nv21, Protocol.FMT_NV21, width, height, ts)
                }
                rawReader = reader
                Log.i(TAG, "raw_yuv sink ${width}x$height")
                reader.surface
            }
            Protocol.MODE_H264_LOWLAT, Protocol.MODE_H264_QUALITY -> {
                val lowLat = mode == Protocol.MODE_H264_LOWLAT
                val enc = H264Encoder(width, height, fps, bitrateBps, lowLat) { payload, _, ptsUs ->
                    emit(payload, Protocol.FMT_H264, width, height, ptsUs)
                }
                val surface = enc.start()
                if (surface == null) { enc.release(); return null }
                encoder = enc
                surface
            }
            else -> {
                Log.e(TAG, "unknown stream mode: $mode")
                null
            }
        }
    }

    /**
     * Tag one frame and ship it. Frames larger than a single UDP datagram are
     * split into [FramePacket.MAX_CHUNK_PAYLOAD]-byte chunks sharing one
     * frameId; the PC reassembles them by frameId. A small frame is a single
     * chunk (count 1) — same cost as before.
     */
    private fun emit(payload: ByteArray, fmt: String, w: Int, h: Int, captureTsUs: Long) {
        val fid = frameId.getAndIncrement()
        val chunkSize = FramePacket.MAX_CHUNK_PAYLOAD
        val total = payload.size
        val chunkCount = if (total <= chunkSize) 1 else (total + chunkSize - 1) / chunkSize
        var index = 0
        var offset = 0
        while (index < chunkCount) {
            val len = minOf(chunkSize, total - offset)
            streamer.send(FramePacket.pack(
                frameId = fid,
                captureTsUs = captureTsUs,
                cameraId = activeCameraRole,
                fmt = fmt,
                width = w,
                height = h,
                payload = payload,
                payloadOffset = offset,
                payloadLen = len,
                chunkIndex = index,
                chunkCount = chunkCount,
            ))
            offset += len
            index++
        }
    }

    fun setBitrate(bps: Int) {
        encoder?.setBitrate(bps)
        if (bps > 0) bitrateBps = bps
    }

    fun stop() {
        encoder?.release(); encoder = null
        rawReader?.release(); rawReader = null
    }

    val isRunning: Boolean get() = encoder != null || rawReader != null

    companion object {
        private const val TAG = "PhoneStreamPipe"
    }
}
