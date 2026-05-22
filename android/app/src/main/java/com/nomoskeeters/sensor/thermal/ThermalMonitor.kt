package com.nomoskeeters.sensor.thermal

import android.content.Context
import android.os.BatteryManager
import android.os.PowerManager
import android.util.Log
import kotlin.concurrent.thread

/**
 * Watches the Snapdragon's thermal state and the battery level, reporting
 * changes upward so the engine can emit `event:thermal_warning` /
 * `event:battery_low`. The PC's safety layer may gate firing when severity
 * rises (PHONE_SENSOR_BOOTSTRAP §4.5).
 *
 * The phone never silently degrades the stream — it reports and lets the PC
 * decide. (Unilateral framerate/resolution cuts are a future extension; for
 * now we surface the signal honestly.)
 */
class ThermalMonitor(
    context: Context,
    private val batteryLowThreshold: Int = 15,
    private val onThermal: (state: String) -> Unit,
    private val onBatteryLow: (percent: Int) -> Unit,
) {
    private val pm = context.getSystemService(Context.POWER_SERVICE) as PowerManager
    private val bm = context.getSystemService(Context.BATTERY_SERVICE) as BatteryManager
    @Volatile private var running = false
    @Volatile var thermalState: String = "none"; private set
    @Volatile private var batteryWarned = false

    private val listener = PowerManager.OnThermalStatusChangedListener { status ->
        thermalState = mapThermal(status)
        Log.i(TAG, "thermal status -> $thermalState")
        onThermal(thermalState)
    }

    fun start() {
        if (running) return
        running = true
        try {
            thermalState = mapThermal(pm.currentThermalStatus)
            pm.addThermalStatusListener(listener)
        } catch (e: Exception) {
            Log.w(TAG, "thermal listener unavailable: ${e.message}")
        }
        thread(name = "phone-battery", isDaemon = true) { batteryLoop() }
    }

    fun stop() {
        running = false
        try { pm.removeThermalStatusListener(listener) } catch (_: Exception) {}
    }

    fun batteryPercent(): Int =
        try { bm.getIntProperty(BatteryManager.BATTERY_PROPERTY_CAPACITY) } catch (e: Exception) { -1 }

    private fun batteryLoop() {
        while (running) {
            val pct = batteryPercent()
            if (pct in 0..batteryLowThreshold && !batteryWarned) {
                batteryWarned = true
                onBatteryLow(pct)
            } else if (pct > batteryLowThreshold) {
                batteryWarned = false
            }
            try { Thread.sleep(30_000) } catch (e: InterruptedException) { return }
        }
    }

    private fun mapThermal(status: Int): String = when (status) {
        PowerManager.THERMAL_STATUS_NONE -> "none"
        PowerManager.THERMAL_STATUS_LIGHT -> "light"
        PowerManager.THERMAL_STATUS_MODERATE -> "moderate"
        PowerManager.THERMAL_STATUS_SEVERE -> "severe"
        PowerManager.THERMAL_STATUS_CRITICAL,
        PowerManager.THERMAL_STATUS_EMERGENCY,
        PowerManager.THERMAL_STATUS_SHUTDOWN -> "critical"
        else -> "none"
    }

    companion object {
        private const val TAG = "PhoneThermal"
    }
}
