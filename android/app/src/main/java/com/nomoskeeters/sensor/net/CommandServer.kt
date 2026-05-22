package com.nomoskeeters.sensor.net

import android.util.Log
import com.nomoskeeters.sensor.protocol.CommandRouter
import com.nomoskeeters.sensor.protocol.Protocol
import org.json.JSONException
import org.json.JSONObject
import java.io.IOException
import java.io.OutputStream
import java.net.InetAddress
import java.net.ServerSocket
import java.net.Socket
import java.net.SocketTimeoutException
import java.nio.charset.StandardCharsets
import java.util.concurrent.atomic.AtomicLong
import kotlin.concurrent.thread

/**
 * TCP command channel — the phone is the **server**. The PC dials in on
 * [Protocol.CMD_PORT]; we read `\n`-delimited JSON commands, route them through
 * [CommandRouter], and write the reply line back on the same socket.
 *
 * One PC at a time. A fresh connection supersedes any stale one. A heartbeat
 * watchdog fires [onLinkIdle] when no command (the PC pings every ~1 s) has
 * arrived within [idleTimeoutMs] — the phone then drops to a safe state
 * (stops streaming) until the PC comes back.
 *
 * [sendEvent] pushes an unsolicited event to the connected PC. The PC's peer
 * address (where frames must be sent) is reported via [onClientConnected].
 */
class CommandServer(
    private val router: CommandRouter,
    private val idleTimeoutMs: Long = 3500L,
    private val onClientConnected: (InetAddress) -> Unit = {},
    private val onClientDisconnected: () -> Unit = {},
    private val onLinkIdle: () -> Unit = {},
) {
    @Volatile private var serverSocket: ServerSocket? = null
    @Volatile private var client: Socket? = null
    @Volatile private var clientOut: OutputStream? = null
    private val writeLock = Any()
    @Volatile private var running = false
    private val lastRxMs = AtomicLong(0L)
    @Volatile private var idleSignalled = false

    fun start() {
        if (running) return
        running = true
        thread(name = "phone-cmd-accept", isDaemon = true) { acceptLoop() }
        thread(name = "phone-cmd-watchdog", isDaemon = true) { watchdogLoop() }
        Log.i(TAG, "command server listening on 0.0.0.0:${Protocol.CMD_PORT}")
    }

    fun stop() {
        running = false
        closeClient()
        try { serverSocket?.close() } catch (_: IOException) {}
        serverSocket = null
    }

    /** Is a PC currently connected? */
    val isConnected: Boolean get() = client?.isConnected == true && client?.isClosed == false

    /** Send one unsolicited event line to the connected PC. No-op if none. */
    fun sendEvent(event: JSONObject) {
        val out = clientOut ?: return
        val line = (event.toString() + "\n").toByteArray(StandardCharsets.UTF_8)
        synchronized(writeLock) {
            try {
                out.write(line)
                out.flush()
            } catch (e: IOException) {
                Log.w(TAG, "sendEvent failed: ${e.message}")
            }
        }
    }

    private fun acceptLoop() {
        try {
            val ss = ServerSocket(Protocol.CMD_PORT)
            ss.reuseAddress = true
            ss.soTimeout = 1000           // re-check `running` every second
            serverSocket = ss
            while (running) {
                val sock = try {
                    ss.accept()
                } catch (e: SocketTimeoutException) {
                    continue
                } catch (e: IOException) {
                    if (running) Log.w(TAG, "accept failed: ${e.message}")
                    break
                }
                // Supersede any previous connection.
                closeClient()
                handleClient(sock)
            }
        } catch (e: IOException) {
            Log.e(TAG, "could not bind ${Protocol.CMD_PORT}: ${e.message}")
        }
    }

    private fun handleClient(sock: Socket) {
        try {
            sock.tcpNoDelay = true        // commands are tiny; never Nagle them
        } catch (_: IOException) {}
        client = sock
        clientOut = sock.getOutputStream()
        lastRxMs.set(System.currentTimeMillis())
        idleSignalled = false
        val peer = sock.inetAddress
        Log.i(TAG, "PC connected from ${peer.hostAddress}")
        onClientConnected(peer)
        thread(name = "phone-cmd-reader", isDaemon = true) { readerLoop(sock) }
    }

    private fun readerLoop(sock: Socket) {
        val input = try { sock.getInputStream() } catch (e: IOException) { return }
        val buf = ByteArray(8192)
        val acc = StringBuilder()
        try {
            while (running && !sock.isClosed) {
                val n = input.read(buf)
                if (n < 0) break          // PC closed the link
                lastRxMs.set(System.currentTimeMillis())
                acc.append(String(buf, 0, n, StandardCharsets.UTF_8))
                var nl = acc.indexOf("\n")
                while (nl >= 0) {
                    val line = acc.substring(0, nl).trim()
                    acc.delete(0, nl + 1)
                    if (line.isNotEmpty()) dispatch(line)
                    nl = acc.indexOf("\n")
                }
            }
        } catch (e: IOException) {
            // fall through to cleanup
        } finally {
            if (client === sock) {
                closeClient()
                onClientDisconnected()
            }
        }
    }

    private fun dispatch(line: String) {
        val msg: JSONObject = try {
            router.parse(line)
        } catch (e: JSONException) {
            Log.w(TAG, "dropping malformed command: ${e.message}")
            return
        }
        val reply = router.handle(msg) ?: return
        val out = clientOut ?: return
        val bytes = (reply.toString() + "\n").toByteArray(StandardCharsets.UTF_8)
        synchronized(writeLock) {
            try {
                out.write(bytes)
                out.flush()
            } catch (e: IOException) {
                Log.w(TAG, "reply write failed: ${e.message}")
            }
        }
        if (router.disconnectRequested) closeClient()
    }

    private fun watchdogLoop() {
        while (running) {
            try { Thread.sleep(500) } catch (e: InterruptedException) { return }
            if (!isConnected) continue
            val silence = System.currentTimeMillis() - lastRxMs.get()
            if (silence > idleTimeoutMs && !idleSignalled) {
                idleSignalled = true
                Log.w(TAG, "link idle ${silence}ms — entering safe state")
                onLinkIdle()
            } else if (silence <= idleTimeoutMs) {
                idleSignalled = false
            }
        }
    }

    private fun closeClient() {
        val sock = client
        client = null
        clientOut = null
        if (sock != null) {
            try { sock.close() } catch (_: IOException) {}
        }
    }

    companion object {
        private const val TAG = "PhoneCmdServer"
    }
}
