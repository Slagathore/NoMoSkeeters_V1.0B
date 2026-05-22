package com.nomoskeeters.sensor.encoder

import android.graphics.ImageFormat
import android.media.Image
import android.media.ImageReader
import android.os.Handler
import android.os.HandlerThread
import android.util.Log
import android.view.Surface

/**
 * Raw `YUV_420_888` capture → NV21 bytes, for the `raw_yuv` stream mode. The PC
 * decodes NV21 in-process with `cv2.cvtColor(..., COLOR_YUV2BGR_NV21)`.
 *
 * NOTE: a raw NV21 frame is `w*h*3/2` bytes — 1080p is ~3.1 MB. The frame
 * channel now fragments frames across datagrams (the PC reassembles by
 * frame_id), so full-resolution raw is deliverable — but a 1080p frame is ~53
 * chunks, and losing any one drops the whole frame, so raw mode is still best
 * kept to modest resolutions. h264 modes remain the right choice at 1080p.
 */
class RawYuvReader(
    private val width: Int,
    private val height: Int,
    private val onFrame: (nv21: ByteArray, captureTsUs: Long) -> Unit,
) {
    private val bgThread = HandlerThread("phone-rawyuv").apply { start() }
    private val bgHandler = Handler(bgThread.looper)
    private val reader: ImageReader =
        ImageReader.newInstance(width, height, ImageFormat.YUV_420_888, 3)

    val surface: Surface get() = reader.surface

    init {
        reader.setOnImageAvailableListener({ r ->
            val image = try { r.acquireLatestImage() } catch (e: Exception) { null }
            if (image != null) {
                try {
                    val nv21 = toNv21(image)
                    onFrame(nv21, image.timestamp / 1000L)
                } catch (e: Exception) {
                    Log.w(TAG, "nv21 convert failed: ${e.message}")
                } finally {
                    image.close()
                }
            }
        }, bgHandler)
    }

    private fun toNv21(image: Image): ByteArray {
        val ySize = width * height
        val nv21 = ByteArray(ySize + width * height / 2)
        val yPlane = image.planes[0]
        val uPlane = image.planes[1]
        val vPlane = image.planes[2]
        val yBuf = yPlane.buffer
        val uBuf = uPlane.buffer
        val vBuf = vPlane.buffer

        // Y plane.
        var pos = 0
        val yRow = yPlane.rowStride
        val yPix = yPlane.pixelStride
        if (yRow == width && yPix == 1) {
            yBuf.get(nv21, 0, ySize)
            pos = ySize
        } else {
            for (row in 0 until height) {
                val base = row * yRow
                for (col in 0 until width) nv21[pos++] = yBuf.get(base + col * yPix)
            }
        }

        // Chroma → NV21 is V,U interleaved.
        val uRow = uPlane.rowStride; val uPix = uPlane.pixelStride
        val vRow = vPlane.rowStride; val vPix = vPlane.pixelStride
        val cw = width / 2; val ch = height / 2
        for (row in 0 until ch) {
            val uBase = row * uRow
            val vBase = row * vRow
            for (col in 0 until cw) {
                nv21[pos++] = vBuf.get(vBase + col * vPix)
                nv21[pos++] = uBuf.get(uBase + col * uPix)
            }
        }
        return nv21
    }

    fun release() {
        try { reader.close() } catch (_: Exception) {}
        bgThread.quitSafely()
    }

    companion object {
        private const val TAG = "PhoneRawYuv"
    }
}
