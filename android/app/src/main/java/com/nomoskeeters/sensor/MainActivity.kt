package com.nomoskeeters.sensor

import android.Manifest
import android.content.ComponentName
import android.content.Context
import android.content.Intent
import android.content.ServiceConnection
import android.os.Build
import android.os.Bundle
import android.os.IBinder
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import com.nomoskeeters.sensor.databinding.ActivityMainBinding
import com.nomoskeeters.sensor.protocol.Protocol
import java.net.Inet4Address
import java.net.NetworkInterface

/**
 * Operator-facing status screen. Shows the phone's reachable IP (so you know
 * what to set `PHONE_IP` to on the PC), the command/frame ports, and live
 * connection / stream / thermal state. Start/Stop control the foreground
 * [SensorService] that hosts the engine.
 */
class MainActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMainBinding
    private var service: SensorService? = null
    private var bound = false

    private val permLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { /* result handled lazily — stream_start re-checks CAMERA */ }

    private val connection = object : ServiceConnection {
        override fun onServiceConnected(name: ComponentName?, b: IBinder?) {
            val local = (b as SensorService.LocalBinder).service
            service = local
            bound = true
            local.engine.statusListener = { status -> runOnUiThread { render(status) } }
        }
        override fun onServiceDisconnected(name: ComponentName?) {
            service = null; bound = false
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        binding.txtIp.text = getString(
            R.string.ip_line, localIpv4() ?: "?", Protocol.CMD_PORT, Protocol.FRAME_PORT)
        binding.txtConn.text = getString(R.string.status_idle)

        requestPermissions()

        binding.btnStart.setOnClickListener { startSensor() }
        binding.btnStop.setOnClickListener { stopSensor() }
    }

    private fun requestPermissions() {
        val perms = mutableListOf(Manifest.permission.CAMERA)
        if (Build.VERSION.SDK_INT >= 33) perms.add(Manifest.permission.POST_NOTIFICATIONS)
        permLauncher.launch(perms.toTypedArray())
    }

    private fun startSensor() {
        val intent = Intent(this, SensorService::class.java)
        startForegroundService(intent)
        bindService(intent, connection, Context.BIND_AUTO_CREATE)
        binding.txtConn.text = getString(R.string.status_listening, Protocol.CMD_PORT)
    }

    private fun stopSensor() {
        if (bound) { unbindService(connection); bound = false }
        service = null
        stopService(Intent(this, SensorService::class.java))
        binding.txtConn.text = getString(R.string.status_idle)
        binding.txtStream.text = ""
        binding.txtHealth.text = ""
    }

    private fun render(s: EngineStatus) {
        binding.txtConn.text = if (s.connected)
            getString(R.string.status_connected, s.peer ?: "?")
        else getString(R.string.status_listening, Protocol.CMD_PORT)
        binding.txtStream.text = if (s.streaming)
            getString(R.string.stream_line, s.activeCamera, s.mode, s.width, s.height, s.fps)
        else getString(R.string.stream_off, s.activeCamera)
        binding.txtHealth.text = getString(R.string.health_line, s.thermal, s.battery)
    }

    override fun onDestroy() {
        if (bound) { unbindService(connection); bound = false }
        super.onDestroy()
    }

    /** First site-local IPv4 — the address the PC dials. Skips loopback and
     *  any USB-NCM/Wi-Fi distinction (operator picks the reachable one). */
    private fun localIpv4(): String? {
        return try {
            NetworkInterface.getNetworkInterfaces().toList()
                .filter { it.isUp && !it.isLoopback }
                .flatMap { it.inetAddresses.toList() }
                .filterIsInstance<Inet4Address>()
                .firstOrNull { it.isSiteLocalAddress }
                ?.hostAddress
        } catch (e: Exception) {
            null
        }
    }
}
