package com.nomoskeeters.sensor.camera

import android.content.Context
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraManager
import android.hardware.camera2.params.StreamConfigurationMap
import android.os.Build
import android.util.Log
import android.util.Size
import com.nomoskeeters.sensor.protocol.Protocol
import org.json.JSONArray
import org.json.JSONObject
import kotlin.math.atan

/**
 * Maps the phone's physical back cameras onto the logical roles the PC
 * addresses ("ultrawide" / "main" / "telephoto") and builds the capabilities
 * manifest returned on `connect`.
 *
 * Role assignment is by focal length: shortest = ultrawide, longest =
 * telephoto, the widest-aperture mid one = main. This is heuristic but matches
 * how every multi-camera Android phone (including the OnePlus 15) lays out its
 * rear array. If enumeration comes up empty (emulator, permission race), we
 * fall back to the OnePlus 15 manifest from PHONE_SENSOR_BOOTSTRAP §5.4 so the
 * PC still gets a sane handshake.
 */
class CameraEnumerator(context: Context) {

    private val cm = context.getSystemService(Context.CAMERA_SERVICE) as CameraManager

    /** role -> physical cameraId, resolved once at construction. */
    val roleToPhysicalId: Map<String, String>
    private val specs: List<CamSpec>

    private data class CamSpec(
        val role: String,
        val physicalId: String,
        val fovHDeg: Double,
        val fovVDeg: Double,
        val maxRes: Size,
        val streamRes: Size,
        val maxFps: Int,
        val hasOpticalZoom: Boolean,
        val opticalZoomFactor: Double,
        val supportsAf: Boolean,
        val supportsLockedFocus: Boolean,
        val supportsHdr: Boolean,
    )

    init {
        val resolved = try { enumerate() } catch (e: Exception) {
            Log.w(TAG, "camera enumeration failed (${e.message}); using fallback")
            emptyList()
        }
        specs = resolved.ifEmpty { fallbackOnePlus15() }
        roleToPhysicalId = specs.associate { it.role to it.physicalId }
        Log.i(TAG, "cameras: " + specs.joinToString { "${it.role}=${it.physicalId}" })
    }

    fun physicalIdFor(role: String): String? = roleToPhysicalId[role]

    /** The manifest entry for one role (fov / zoom / resolution), or null. */
    fun cameraEntry(role: String): JSONObject? {
        val s = specs.firstOrNull { it.role == role } ?: return null
        return JSONObject().apply {
            put("id", s.role)
            put("fov_h_deg", round1(s.fovHDeg))
            put("has_optical_zoom", s.hasOpticalZoom)
            put("optical_zoom_factor", round1(s.opticalZoomFactor))
        }
    }

    fun manifest(): JSONObject {
        val cams = JSONArray()
        for (s in specs) {
            cams.put(JSONObject().apply {
                put("id", s.role)
                put("fov_h_deg", round1(s.fovHDeg))
                put("fov_v_deg", round1(s.fovVDeg))
                put("max_resolution", JSONArray(listOf(s.maxRes.width, s.maxRes.height)))
                put("preferred_streaming_resolution",
                    JSONArray(listOf(s.streamRes.width, s.streamRes.height)))
                put("max_fps_at_streaming_res", s.maxFps)
                put("has_optical_zoom", s.hasOpticalZoom)
                put("optical_zoom_factor", round1(s.opticalZoomFactor))
                put("supports_af", s.supportsAf)
                put("supports_locked_focus", s.supportsLockedFocus)
                put("supports_hdr", s.supportsHdr)
            })
        }
        return JSONObject().apply {
            put("phone_model", "${Build.MANUFACTURER} ${Build.MODEL}")
            put("protocol_version", Protocol.VERSION)
            put("cameras", cams)
        }
    }

    private fun enumerate(): List<CamSpec> {
        data class Raw(
            val id: String, val focal: Float, val sensorW: Float,
            val sensorH: Float, val maxRes: Size, val maxFps: Int,
            val af: Boolean, val hdr: Boolean,
        )

        val raws = mutableListOf<Raw>()
        for (id in cm.cameraIdList) {
            val ch = cm.getCameraCharacteristics(id)
            if (ch.get(CameraCharacteristics.LENS_FACING) !=
                CameraCharacteristics.LENS_FACING_BACK) continue
            val focals = ch.get(
                CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS)
            val size = ch.get(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE)
            val map = ch.get(
                CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)
            if (focals == null || focals.isEmpty() || size == null || map == null) continue
            val maxRes = largestJpeg(map)
            val afModes = ch.get(CameraCharacteristics.CONTROL_AF_AVAILABLE_MODES)
                ?: IntArray(0)
            val hasAf = afModes.any {
                it == CameraCharacteristics.CONTROL_AF_MODE_AUTO ||
                    it == CameraCharacteristics.CONTROL_AF_MODE_CONTINUOUS_VIDEO
            }
            val caps = ch.get(
                CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES) ?: IntArray(0)
            val hdr = Build.VERSION.SDK_INT >= 33 && caps.any { it == 18 } // DYNAMIC_RANGE_TEN_BIT
            raws.add(Raw(id, focals[0], size.width, size.height, maxRes, 60, hasAf, hdr))
        }
        if (raws.isEmpty()) return emptyList()

        raws.sortBy { it.focal }
        val mainFocal = raws[raws.size / 2].focal   // median-ish = main
        val out = mutableListOf<CamSpec>()
        raws.forEachIndexed { i, r ->
            val role = when {
                raws.size == 1 -> Protocol.CAM_MAIN
                i == 0 -> Protocol.CAM_ULTRAWIDE
                i == raws.size - 1 && r.focal > mainFocal * 1.4f -> Protocol.CAM_TELEPHOTO
                else -> Protocol.CAM_MAIN
            }
            // De-dup roles: only one of each, keep the first.
            if (out.any { it.role == role }) return@forEachIndexed
            val fovH = fovDeg(r.sensorW, r.focal)
            val fovV = fovDeg(r.sensorH, r.focal)
            val isTele = role == Protocol.CAM_TELEPHOTO
            out.add(
                CamSpec(
                    role = role, physicalId = r.id,
                    fovHDeg = fovH, fovVDeg = fovV,
                    maxRes = r.maxRes,
                    streamRes = if (isTele) Size(1280, 720) else Size(1920, 1080),
                    maxFps = r.maxFps,
                    hasOpticalZoom = isTele,
                    opticalZoomFactor = if (isTele) (r.focal / mainFocal).toDouble() else 1.0,
                    supportsAf = r.af, supportsLockedFocus = r.af,
                    supportsHdr = r.hdr,
                )
            )
        }
        return out.sortedBy { roleOrder(it.role) }
    }

    private fun largestJpeg(map: StreamConfigurationMap): Size {
        val sizes = map.getOutputSizes(android.graphics.ImageFormat.JPEG)
            ?: return Size(1920, 1080)
        return sizes.maxByOrNull { it.width.toLong() * it.height } ?: Size(1920, 1080)
    }

    private fun fovDeg(sensorDim: Float, focal: Float): Double =
        Math.toDegrees(2.0 * atan((sensorDim / (2.0 * focal))))

    private fun round1(v: Double): Double = Math.round(v * 10.0) / 10.0

    private fun roleOrder(role: String): Int = when (role) {
        Protocol.CAM_ULTRAWIDE -> 0
        Protocol.CAM_MAIN -> 1
        Protocol.CAM_TELEPHOTO -> 2
        else -> 3
    }

    /** PHONE_SENSOR_BOOTSTRAP §5.4 manifest — used only if live enumeration
     *  yields nothing. The physical ids are the conventional OnePlus mapping;
     *  if they're wrong on a given unit, live enumeration (the normal path)
     *  overrides this anyway. */
    private fun fallbackOnePlus15(): List<CamSpec> = listOf(
        CamSpec(Protocol.CAM_ULTRAWIDE, "2", 116.0, 80.0, Size(4000, 3000),
            Size(1920, 1080), 60, false, 1.0, true, true, false),
        CamSpec(Protocol.CAM_MAIN, "0", 75.0, 55.0, Size(8160, 6120),
            Size(1920, 1080), 60, true, 1.0, true, true, true),
        CamSpec(Protocol.CAM_TELEPHOTO, "3", 23.0, 17.0, Size(4080, 3060),
            Size(1280, 720), 60, true, 3.0, true, true, false),
    )

    companion object {
        private const val TAG = "PhoneCameraEnum"
    }
}
