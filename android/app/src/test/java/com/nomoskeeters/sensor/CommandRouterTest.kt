package com.nomoskeeters.sensor

import com.nomoskeeters.sensor.protocol.CommandRouter
import com.nomoskeeters.sensor.protocol.Protocol
import com.nomoskeeters.sensor.protocol.SensorBackend
import org.json.JSONArray
import org.json.JSONException
import org.json.JSONObject
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertThrows
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Command-dispatch logic, exercised on a plain JVM with a fake backend. Mirrors
 * the PC's `_FakePhone` reply expectations: replies are `<cmd>_reply`, echo the
 * `cmd_id`, and carry `ok`.
 */
class CommandRouterTest {

    /** Records calls and returns scriptable results. */
    private class FakeBackend : SensorBackend {
        var lastCamera: String? = null
        var lastStream: List<Any>? = null
        var afRegion: DoubleArray? = null
        var locked = false

        override fun capabilitiesManifest() = JSONObject().apply {
            put("phone_model", "TestPhone")
            put("protocol_version", Protocol.VERSION)
            put("cameras", JSONArray().put(JSONObject().put("id", "main")))
        }
        override fun setActiveCamera(cameraId: String): Boolean { lastCamera = cameraId; return true }
        override fun startStream(mode: String, width: Int, height: Int, fps: Int): String {
            lastStream = listOf(mode, width, height, fps); return mode
        }
        override fun stopStream() = true
        override fun setExposureMode(mode: String) = true
        override fun setExposureValue(shutterUs: Long, iso: Int) = true
        override fun setAfMode(mode: String) = true
        override fun setAfRegion(x: Double, y: Double, w: Double, h: Double): Boolean {
            afRegion = doubleArrayOf(x, y, w, h); return true
        }
        override fun lockFocus(): Boolean { locked = true; return true }
        override fun unlockFocus(): Boolean { locked = false; return true }
        override fun setStreamResolution(width: Int, height: Int) = true
        override fun setTargetBitrate(bitrateBps: Int) = bitrateBps > 0
        override fun setTargetFps(fps: Int) = fps > 0
        override fun status() = JSONObject().put("camera_id", "main").put("streaming", true)
        override fun intrinsics(cameraId: String?) = JSONObject().put("camera_id", cameraId ?: "main")
        override fun recordingStart(filename: String, format: String) = false
        override fun recordingStop() = false
        override fun recordingList() = JSONArray()
        override fun recordingTransfer(filename: String) = false
    }

    private fun route(backend: FakeBackend = FakeBackend()): Pair<CommandRouter, FakeBackend> =
        CommandRouter(backend) to backend

    private fun handle(router: CommandRouter, json: String): JSONObject? =
        router.handle(router.parse(json))

    @Test
    fun connectReturnsCapabilitiesAndEchoesCmdId() {
        val (router, _) = route()
        val reply = handle(router, """{"type":"connect","cmd_id":1,"protocol_version":1}""")!!
        assertEquals("connect_reply", reply.getString("type"))
        assertEquals(1, reply.getInt("cmd_id"))
        assertTrue(reply.getBoolean("ok"))
        assertEquals("TestPhone", reply.getJSONObject("capabilities").getString("phone_model"))
    }

    @Test
    fun pingIsOk() {
        val (router, _) = route()
        val reply = handle(router, """{"type":"ping","cmd_id":9}""")!!
        assertEquals("ping_reply", reply.getString("type"))
        assertEquals(9, reply.getInt("cmd_id"))
        assertTrue(reply.getBoolean("ok"))
    }

    @Test
    fun setActiveCameraForwardsRole() {
        val (router, backend) = route()
        val reply = handle(router, """{"type":"set_active_camera","cmd_id":2,"camera_id":"telephoto"}""")!!
        assertTrue(reply.getBoolean("ok"))
        assertEquals("telephoto", backend.lastCamera)
    }

    @Test
    fun streamStartEchoesNegotiatedMode() {
        val (router, backend) = route()
        val reply = handle(router,
            """{"type":"stream_start","cmd_id":3,"mode":"h264_lowlat","width":1920,"height":1080,"fps":60}""")!!
        assertTrue(reply.getBoolean("ok"))
        assertEquals("h264_lowlat", reply.getString("mode"))
        assertEquals(listOf<Any>("h264_lowlat", 1920, 1080, 60), backend.lastStream)
    }

    @Test
    fun setAfRegionParsesNormalizedCoords() {
        val (router, backend) = route()
        handle(router, """{"type":"set_af_region","cmd_id":4,"x":0.4,"y":0.5,"w":0.2,"h":0.2}""")
        assertNotNull(backend.afRegion)
        assertEquals(0.4, backend.afRegion!![0], 1e-9)
        assertEquals(0.2, backend.afRegion!![3], 1e-9)
    }

    @Test
    fun lockAndUnlockFocus() {
        val (router, backend) = route()
        handle(router, """{"type":"lock_focus","cmd_id":5}""")
        assertTrue(backend.locked)
        handle(router, """{"type":"unlock_focus","cmd_id":6}""")
        assertFalse(backend.locked)
    }

    @Test
    fun getStatusMergesFieldsAlongsideOk() {
        val (router, _) = route()
        val reply = handle(router, """{"type":"get_status","cmd_id":7}""")!!
        assertTrue(reply.getBoolean("ok"))
        assertEquals("main", reply.getString("camera_id"))
        assertTrue(reply.getBoolean("streaming"))
    }

    @Test
    fun unknownCommandIsHonestlyRejected() {
        val (router, _) = route()
        val reply = handle(router, """{"type":"brew_coffee","cmd_id":8}""")!!
        assertFalse(reply.getBoolean("ok"))
        assertTrue(reply.has("error"))
    }

    @Test
    fun disconnectSetsFlag() {
        val (router, _) = route()
        handle(router, """{"type":"disconnect","cmd_id":10}""")
        assertTrue(router.disconnectRequested)
    }

    @Test
    fun parseRejectsMissingType() {
        val (router, _) = route()
        assertThrows(JSONException::class.java) { router.parse("""{"cmd_id":1}""") }
    }

    @Test
    fun eventCarriesNoCmdId() {
        val ev = CommandRouter.event(Protocol.EV_THERMAL_WARNING, mapOf("state" to "light"))
        assertEquals(Protocol.EV_THERMAL_WARNING, ev.getString("type"))
        assertFalse(ev.has("cmd_id"))
        assertEquals("light", ev.getString("state"))
    }
}
