package com.nomoskeeters.sensor.encoder

import android.media.MediaCodec
import android.media.MediaCodecInfo
import android.media.MediaFormat
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.HandlerThread
import android.util.Log
import android.view.Surface
import java.nio.ByteBuffer

/**
 * Hardware H.264 encoder fed by a camera input [Surface] (zero-copy: the camera
 * renders straight into the encoder, no per-frame YUV copy).
 *
 * Output is an Annex-B elementary stream — exactly what the PC pipes into
 * `ffmpeg -f h264 -i pipe:0`. SPS/PPS (the codec-config buffer) is cached and
 * prepended to every keyframe so a decoder that joined late (UDP drops) can
 * resync without a separate signalling channel.
 *
 * Low-latency knobs: `KEY_LATENCY=1`, realtime `KEY_PRIORITY`, CBR, and — the
 * key adaptation for this project — **cyclic intra-refresh** instead of periodic
 * full IDR frames. The PC's UDP frame channel carries one frame per datagram
 * with no reassembly, so a fat 1080p IDR would blow the 65507-byte ceiling and
 * be dropped. Intra-refresh spreads the intra blocks across frames, keeping
 * every packet small. (Documented deviation from the every-frame-keyframe
 * snippet in PHONE_SENSOR_BOOTSTRAP §4.4, forced by the transport.)
 */
class H264Encoder(
    private val width: Int,
    private val height: Int,
    private val fps: Int,
    private var bitrateBps: Int,
    private val lowLatency: Boolean,
    private val onEncoded: (payload: ByteArray, isKeyframe: Boolean, ptsUs: Long) -> Unit,
) {
    private var codec: MediaCodec? = null
    private var inputSurface: Surface? = null
    private val bgThread = HandlerThread("phone-h264").apply { start() }
    private val bgHandler = Handler(bgThread.looper)
    private var csd: ByteArray? = null            // cached SPS+PPS (Annex-B)
    @Volatile private var started = false

    /** Configure + start. Returns the input surface for the camera to target,
     *  or null on failure. */
    fun start(): Surface? {
        return try {
            val mime = MediaFormat.MIMETYPE_VIDEO_AVC
            val codec = MediaCodec.createEncoderByType(mime)
            this.codec = codec
            val fmt = buildFormat(mime)
            codec.setCallback(callback, bgHandler)
            codec.configure(fmt, null, null, MediaCodec.CONFIGURE_FLAG_ENCODE)
            inputSurface = codec.createInputSurface()
            codec.start()
            started = true
            Log.i(TAG, "H.264 up: ${width}x$height @${fps} ${bitrateBps / 1000}kbps " +
                    "lowlat=$lowLatency")
            inputSurface
        } catch (e: Exception) {
            Log.e(TAG, "encoder start failed: ${e.message}")
            release()
            null
        }
    }

    private fun buildFormat(mime: String): MediaFormat {
        val fmt = MediaFormat.createVideoFormat(mime, width, height)
        fmt.setInteger(MediaFormat.KEY_COLOR_FORMAT,
            MediaCodecInfo.CodecCapabilities.COLOR_FormatSurface)
        fmt.setInteger(MediaFormat.KEY_BIT_RATE, bitrateBps)
        fmt.setInteger(MediaFormat.KEY_FRAME_RATE, fps)
        fmt.setInteger(MediaFormat.KEY_BITRATE_MODE,
            MediaCodecInfo.EncoderCapabilities.BITRATE_MODE_CBR)
        // Long GOP; intra-refresh (below) does the resilience work instead of
        // fat IDR frames that wouldn't fit one datagram.
        fmt.setInteger(MediaFormat.KEY_I_FRAME_INTERVAL, 4)
        // Cyclic intra-refresh: refresh the whole frame over ~1s of frames.
        try {
            fmt.setInteger(MediaFormat.KEY_INTRA_REFRESH_PERIOD, fps.coerceAtLeast(1))
        } catch (_: Exception) {}
        if (Build.VERSION.SDK_INT >= 30) {
            fmt.setInteger(MediaFormat.KEY_LATENCY, if (lowLatency) 1 else 0)
        }
        // Realtime priority + operating rate hint = encoder favours latency.
        fmt.setInteger(MediaFormat.KEY_PRIORITY, 0)
        fmt.setInteger(MediaFormat.KEY_OPERATING_RATE, fps * 2)
        // Profile/level are deliberately left to the encoder's defaults —
        // forcing them is the most common cause of configure() rejection
        // across vendors, and the ffmpeg decoder on the PC accepts whatever
        // the hardware emits. Tune here only if a specific device needs it.
        return fmt
    }

    /** Dynamically retarget the bitrate (PC `stream_set_target_bitrate`). */
    fun setBitrate(bps: Int) {
        if (bps <= 0) return
        bitrateBps = bps
        val c = codec ?: return
        try {
            c.setParameters(Bundle().apply {
                putInt(MediaCodec.PARAMETER_KEY_VIDEO_BITRATE, bps)
            })
        } catch (e: Exception) {
            Log.w(TAG, "setBitrate failed: ${e.message}")
        }
    }

    private val callback = object : MediaCodec.Callback() {
        override fun onInputBufferAvailable(c: MediaCodec, index: Int) {
            // Surface input — no client-fed input buffers.
        }

        override fun onOutputBufferAvailable(
            c: MediaCodec, index: Int, info: MediaCodec.BufferInfo
        ) {
            val buf: ByteBuffer? = try { c.getOutputBuffer(index) } catch (e: Exception) { null }
            if (buf != null && info.size > 0) {
                buf.position(info.offset)
                buf.limit(info.offset + info.size)
                if (info.flags and MediaCodec.BUFFER_FLAG_CODEC_CONFIG != 0) {
                    // SPS/PPS — stash, don't emit on its own.
                    val cfg = ByteArray(info.size)
                    buf.get(cfg)
                    csd = cfg
                } else {
                    val isKey = info.flags and MediaCodec.BUFFER_FLAG_KEY_FRAME != 0
                    val frame = ByteArray(info.size)
                    buf.get(frame)
                    val payload = if (isKey && csd != null) csd!! + frame else frame
                    onEncoded(payload, isKey, info.presentationTimeUs)
                }
            }
            try { c.releaseOutputBuffer(index, false) } catch (_: Exception) {}
        }

        override fun onError(c: MediaCodec, e: MediaCodec.CodecException) {
            Log.e(TAG, "codec error: ${e.message}")
        }

        override fun onOutputFormatChanged(c: MediaCodec, format: MediaFormat) {
            // SPS/PPS also arrive here on some devices; capture as fallback.
            format.getByteBuffer("csd-0")?.let { sps ->
                val pps = format.getByteBuffer("csd-1")
                val a = ByteArray(sps.remaining()); sps.get(a)
                csd = if (pps != null) {
                    val b = ByteArray(pps.remaining()); pps.get(b); a + b
                } else a
            }
        }
    }

    fun release() {
        started = false
        try { codec?.stop() } catch (_: Exception) {}
        try { codec?.release() } catch (_: Exception) {}
        codec = null
        try { inputSurface?.release() } catch (_: Exception) {}
        inputSurface = null
        bgThread.quitSafely()
    }

    companion object {
        private const val TAG = "PhoneH264"
    }
}
