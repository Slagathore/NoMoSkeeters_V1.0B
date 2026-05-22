package com.nomoskeeters.sensor.net

import android.util.Log
import com.nomoskeeters.sensor.protocol.FramePacket
import com.nomoskeeters.sensor.protocol.Protocol
import java.io.IOException
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress

/**
 * UDP frame channel — sends [FramePacket]-encoded datagrams (one per chunk) to
 * the PC on [Protocol.FRAME_PORT].
 *
 * The destination IP is the PC's address as learned from the TCP command
 * connection ([setDestination]); the port is fixed by the protocol. Frames are
 * already chunked to fit by `StreamPipeline` (split on
 * [FramePacket.MAX_CHUNK_PAYLOAD]); the [FramePacket.MAX_UDP_PAYLOAD] check here
 * is a defensive backstop — a datagram over the ceiling can't be sent, so it is
 * dropped with a warning rather than failing at the socket.
 */
class FrameStreamer {
    private var socket: DatagramSocket? = null
    @Volatile private var dest: InetAddress? = null
    private var oversizeWarned = false

    fun start() {
        if (socket == null) {
            socket = DatagramSocket().apply {
                // A generous send buffer smooths bursty keyframes.
                try { sendBufferSize = 1 shl 20 } catch (_: Exception) {}
            }
        }
    }

    fun setDestination(addr: InetAddress?) {
        dest = addr
        Log.i(TAG, "frame destination -> ${addr?.hostAddress}:${Protocol.FRAME_PORT}")
    }

    /** Send one fully-packed chunk datagram. Cheap no-op until a destination
     *  is set. */
    fun send(packet: ByteArray) {
        val sock = socket ?: return
        val to = dest ?: return
        if (packet.size > FramePacket.MAX_UDP_PAYLOAD) {
            // Shouldn't happen: StreamPipeline splits on MAX_CHUNK_PAYLOAD.
            if (!oversizeWarned) {
                Log.w(TAG, "chunk ${packet.size}B exceeds UDP ceiling " +
                        "${FramePacket.MAX_UDP_PAYLOAD}B — dropping. This is a " +
                        "chunking bug; MAX_CHUNK_PAYLOAD should keep chunks small.")
                oversizeWarned = true
            }
            return
        }
        oversizeWarned = false
        try {
            sock.send(DatagramPacket(packet, packet.size, to, Protocol.FRAME_PORT))
        } catch (e: IOException) {
            Log.w(TAG, "frame send failed: ${e.message}")
        }
    }

    fun stop() {
        try { socket?.close() } catch (_: Exception) {}
        socket = null
        dest = null
    }

    companion object {
        private const val TAG = "PhoneFrameStreamer"
    }
}
