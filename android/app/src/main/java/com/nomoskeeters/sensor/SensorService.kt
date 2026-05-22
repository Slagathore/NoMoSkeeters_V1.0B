package com.nomoskeeters.sensor

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.Service
import android.content.Intent
import android.content.pm.ServiceInfo
import android.os.Binder
import android.os.Build
import android.os.IBinder
import android.util.Log

/**
 * Foreground service that hosts the [SensorEngine] for the life of a session.
 * Foreground + `camera` service type keeps OxygenOS from killing the capture
 * pipeline when the screen sleeps (PHONE_SENSOR_BOOTSTRAP §4.1).
 *
 * Bound by [MainActivity] so the UI can read live status; also started sticky
 * so it survives the activity going away.
 */
class SensorService : Service() {

    private val binder = LocalBinder()
    val engine: SensorEngine by lazy { SensorEngine(applicationContext) }
    private var started = false

    inner class LocalBinder : Binder() {
        val service: SensorService get() = this@SensorService
    }

    override fun onBind(intent: Intent?): IBinder = binder

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        if (!started) {
            startForegroundWithType()
            engine.start()
            started = true
            Log.i(TAG, "SensorService started")
        }
        return START_STICKY
    }

    override fun onDestroy() {
        if (started) {
            engine.stop()
            started = false
        }
        super.onDestroy()
        Log.i(TAG, "SensorService destroyed")
    }

    private fun startForegroundWithType() {
        val nm = getSystemService(NotificationManager::class.java)
        val channel = NotificationChannel(
            CHANNEL_ID, "NoMoSkeeters Sensor",
            NotificationManager.IMPORTANCE_LOW
        ).apply { description = "Phone-as-sensor capture service" }
        nm.createNotificationChannel(channel)

        val notif: Notification = Notification.Builder(this, CHANNEL_ID)
            .setContentTitle("NoMoSkeeters Sensor")
            .setContentText("Listening for the PC on :${com.nomoskeeters.sensor.protocol.Protocol.CMD_PORT}")
            .setSmallIcon(android.R.drawable.ic_menu_camera)
            .setOngoing(true)
            .build()

        if (Build.VERSION.SDK_INT >= 29) {
            startForeground(NOTIF_ID, notif, ServiceInfo.FOREGROUND_SERVICE_TYPE_CAMERA)
        } else {
            startForeground(NOTIF_ID, notif)
        }
    }

    companion object {
        private const val TAG = "PhoneSensorService"
        private const val CHANNEL_ID = "nomoskeeters_sensor"
        private const val NOTIF_ID = 1
    }
}
