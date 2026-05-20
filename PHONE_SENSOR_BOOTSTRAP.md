# PHONE_SENSOR_BOOTSTRAP.md

**Architecture spec for "phone as remote-controlled smart sensor" path.**

> Status: Design draft, 2026-05-17. Parallel to `BOOTSTRAP.md`. Where this doc and BOOTSTRAP.md disagree on architecture, this doc wins for the phone subsystem only — BOOTSTRAP.md remains canonical for PC orchestration, LaserCube control, Kinect integration, and safety.

> Scope: This describes building a Companion Android app for OnePlus 15 (Snapdragon 8 Elite, Sony LYT-808 main + LYT-600 telephoto, Hasselblad-tuned ISP) that acts as a network-attached camera under PC orchestration. **The PC remains the brain.** The phone is a high-quality, command-able sensor.

> Author: Claude, based on architectural decisions made with Cole 2026-05-17.

---

## 0. Read this first — the architectural commitment

Three things must be true throughout the design or this doesn't work:

1. **The PC is the orchestrator.** Detection, tracking, fire decisions, calibration, Kinect fusion, safety reasoning, session recording, the bus — all of these live on the PC. The phone never decides what to shoot at; it never decides which lens to use autonomously; it never decides anything beyond "did I receive a valid command and execute it."

2. **The phone is a smart sensor, not a brain.** It accepts commands like "switch to telephoto," "set AF region to (x, y, w, h)," "lock exposure," "start streaming." It executes them and reports state. It does not run YOLO. It does not maintain tracks. It does not decide what to do next.

3. **The link between them is a defined protocol, not screen-scraping.** A real protobuf or JSON-over-TCP/UDP message bus with versioned schemas. No `scrcpy`, no virtual webcams. The phone speaks NoMoSkeeters Sensor Protocol v1 and that protocol is the contract.

If you find yourself adding "the phone could just decide..." or "let's run a quick model on the phone for...", stop. That's the path to the 2-month rabbit hole I warned about. The phone is **dumb on purpose**, because that's what makes this architecturally tractable.

---

## 1. Why this architecture wins (and what it doesn't)

### What it wins

| Capability | Other options | Phone-as-smart-sensor |
|---|---|---|
| Multiple focal lengths | Buy multiple cameras | Switch lenses programmatically |
| Optical zoom on demand | Not available | Telephoto on command |
| Hardware autofocus | Manual or fixed | Lock focus on operator-defined region |
| HDR for varying light | None | Phone ISP handles automatically |
| Computational photography | None | Available if needed |
| Night mode / low-light stacking | None | Phone ISP handles |
| Sensor quality | Industrial-grade or webcam-grade | Flagship phone-grade (better than most $500 industrial cameras) |
| Cost | $30-200 for a fixed alternative | $0 (already owned) |

### What it doesn't win

| Capability | Phone-as-smart-sensor | OV9281 industrial |
|---|---|---|
| Raw latency floor | 100-200ms | 25ms |
| Sustained operation | Thermal throttling possible | Designed for 24/7 |
| Mechanical robustness | Hard to mount, fragile | Robust, easy to mount |
| Power draw | Battery + USB charging required | USB-powered |
| Reliability | Phone OS quirks | Plain UVC |
| Setup complexity | Custom Android app | `cv2.VideoCapture(0)` |

The phone path is **the right call when you want capabilities that fixed cameras can't offer** — multi-camera, optical zoom, AF, ISP features. If your operating volume is fixed and your lighting is predictable, the OV9281 is a better choice. If you want to operate across many scenarios, the phone wins.

### The decision

These aren't mutually exclusive. **Best architecture is: build the OV9281 path first as V1 (fast, simple, low-latency), then add the phone as a parallel optional sensor for V2** when you want zoom/AF/multi-camera capabilities. The phone integration becomes a Mode-A-style fusion partner alongside the OV9281 and Kinect.

If you're going to commit to phone-only with no OV9281, accept that V1 will be 100-200ms latency and plan the targeting math around that. With the cone-collapse pattern's breach-and-restart behavior and lead-aim, this is workable on slower targets (resting mosquitoes, larger insects, calibration targets) but marginal for fast-flying mosquitoes.

---

## 2. The phone sensor's contract

What the PC promises to send, what the phone promises to do.

### 2.1 Commands the phone accepts

All commands are JSON messages over a single TCP connection. Each command has a unique `cmd_id` that the phone echoes back in its response. Schema versioned via `protocol_version` in handshake.

**Connection management**
- `connect` — initial handshake, exchanges protocol version + camera capabilities manifest
- `disconnect` — graceful shutdown
- `ping` — heartbeat (PC sends every 1s, phone echoes; if missed 3x, phone enters safe state)

**Camera selection**
- `set_active_camera` → params: `{camera_id: "wide" | "main" | "telephoto" | "ultrawide"}` — switches which physical lens is feeding the stream. Returns after the switch completes (300-800ms typical).
- `get_camera_capabilities` — returns the list of available cameras with their intrinsics (FOV, focal length, max resolution, supported framerates)

**Exposure & focus control**
- `set_exposure_mode` → params: `{mode: "auto" | "manual"}` — locks/unlocks auto-exposure
- `set_exposure_value` → params: `{shutter_us: int, iso: int}` — manual exposure (only valid when mode=manual)
- `set_af_mode` → params: `{mode: "auto" | "manual" | "locked"}` — autofocus behavior
- `set_af_region` → params: `{x: float, y: float, w: float, h: float}` — normalized [0,1] coords for AF region; phone tries to focus on this region
- `lock_focus` — commits current focus state; subsequent frames won't refocus
- `unlock_focus`

**Streaming control**
- `stream_start` → params: `{mode: "raw_yuv" | "h264_lowlat" | "h264_quality"}` — begins frame transport. Returns the chosen mode (may downgrade if hardware doesn't support).
- `stream_stop`
- `stream_set_resolution` → params: `{width: int, height: int}` — for cases where lower res = lower latency
- `stream_set_target_bitrate` → params: `{bitrate_bps: int}` — H.264 quality vs latency tradeoff
- `stream_set_target_fps` → params: `{fps: int}` — request specific framerate

**Status & diagnostics**
- `get_status` — returns current camera, current resolution, current fps, exposure values, focus state, temperature, battery, recording state
- `get_intrinsics` → params: `{camera_id?: string}` — returns the calibrated intrinsics for the current or specified camera (focal length, principal point, distortion coefficients if calibrated)

**Recording (phone-local)**
- `recording_start` → params: `{filename: string, format: "mp4" | "raw"}` — begins recording the active stream to phone storage. Independent of the network stream.
- `recording_stop`
- `recording_list` — returns list of recordings on phone
- `recording_transfer` → params: `{filename: string}` — pushes a recording to PC over the data link

### 2.2 What the phone sends unprompted

**Frame data** — on its own channel (UDP on a separate port). Format depends on the chosen stream mode:
- `raw_yuv`: raw YUV 4:2:0 frames, no compression. Highest bandwidth (~150 Mbps at 1080p60), lowest latency.
- `h264_lowlat`: hardware H.264 with `intra_period=1` (keyframe every frame), zero B-frames, low-latency settings. ~5-15 Mbps, 50-100ms latency.
- `h264_quality`: hardware H.264 with normal GOP. ~3-8 Mbps, 100-300ms latency.

Each frame packet carries:
```
{
  frame_id: uint64,        // monotonic
  capture_ts_us: uint64,   // microseconds, phone monotonic clock
  camera_id: string,       // which lens captured this
  width: uint16,
  height: uint16,
  format: enum,            // YUV420, NV21, etc.
  payload: bytes
}
```

**Status events** — on the command channel, unsolicited:
- `event:af_settled` — autofocus has locked on a target
- `event:exposure_changed` — auto-exposure has adjusted
- `event:thermal_warning` — phone is getting hot, capabilities may degrade
- `event:battery_low` — phone battery dropping below threshold
- `event:camera_unavailable` — requested camera is in use by another app, etc.

### 2.3 What the phone is allowed to do on its own

Almost nothing. Specifically:

- **Auto-exposure**: only when explicitly in `auto` mode. Phone may adjust within that mode.
- **Auto-focus**: only when explicitly in `auto` mode and AF region is set.
- **Thermal management**: phone may reduce framerate or resolution unilaterally to prevent overheating. Emits `thermal_warning` event when it does.
- **Reconnect**: if network drops, phone retries connection. Phone holds last-known-good camera state until reconnect.

That's it. The phone does not decide to switch cameras. It does not decide to start recording. It does not pick exposure values when in manual mode.

---

## 3. The PC's side of the contract

What the PC commits to.

### 3.1 The new subsystem: `phone_sensor/`

```
phone_sensor/
├── __init__.py
├── protocol.py             # message schema, version negotiation
├── client.py               # PhoneSensorClient class — speaks the protocol
├── frame_decoder.py        # H.264/YUV decoding via PyAV or ffmpeg subprocess
├── command_dispatcher.py   # queues + acknowledges commands
└── calibration.py          # per-camera intrinsics calibration
```

### 3.2 How it integrates with the existing bus

`PhoneSensor` is a `Sensor` subclass per `sensors/base.py`. It implements:

- `read() -> SensorFrame` — returns the latest decoded frame, blocking briefly if none ready
- `role` — `SensorRole.TARGETING` by default, configurable
- `sensor_id` — `"phone_main"` or `"phone_telephoto"` depending on active camera (changes when camera switches)
- `capabilities()` — returns dict of what this sensor can do (zoom, AF, HDR, etc.) — used by the orchestrator to decide whether to send certain commands

The PhoneSensor emits frames on the bus exactly like any other sensor. Downstream consumers (detector, tracker, recorder) don't know or care that the source is a phone.

### 3.3 New events the bus carries

In addition to the existing `FrameEvent`, `DetectionEvent`, etc., the phone subsystem emits:

- `PhoneCameraChangedEvent` — emitted when the active camera switches. Carries new camera_id, new intrinsics. Tracker uses this to invalidate or transform existing tracks.
- `PhoneFocusEvent` — AF state change. Recorded for replay; not used in real-time decisions usually.
- `PhoneThermalEvent` — capabilities may degrade. Safety system may decide to halt firing if phone enters thermal protection.

### 3.4 Calibration changes

The phone has multiple cameras with different intrinsics and different extrinsics relative to the laser. This is the **biggest open problem** in this architecture.

Three approaches:

**Approach A — Phone is rigidly mounted, one camera at a time used per session.**
Operator chooses one camera before each session, calibration is per-(camera, mount). Switching cameras mid-session invalidates calibration until re-calibrated. Simple but restrictive.

**Approach B — Phone is rigidly mounted, all cameras pre-calibrated, multi-profile selection.**
Calibrate each camera once during setup. Store as separate profiles. When the active camera switches mid-session, the homography automatically swaps to the matching profile. More work upfront but smooth at runtime.

**Approach C — Phone moves around, calibration runs on-demand.**
Each time the phone is repositioned, run a fast (~10s) calibration. Phone reports its rough pose, PC tracks calibration freshness. Most flexible but slowest setup.

Recommended: **Approach B** for V1. Rigidly mount the phone (3D-printed bracket on the laser tripod), calibrate all available cameras once, store profiles. Per-camera homography selection is automatic.

### 3.5 The lens-switch dance

When the PC wants to switch from wide to telephoto (e.g., target acquired, commit to tracking):

```
1. PC: Tracker confirms high-confidence track on target T at position P_wide
2. PC: Compute predicted target position 1 second in the future (lens switch
       takes 300-800ms, target moves during the switch)
3. PC sends: set_active_camera(telephoto)
4. PC sends: set_af_region(predicted_position)
5. PC sends: lock_focus
6. PC: Stops sending detection updates to tracker during switch (frames will
       be from a different camera with different intrinsics)
7. Phone: switches camera (300-800ms), AF settles, sends event:af_settled
8. PC: Receives event:af_settled. Emits PhoneCameraChangedEvent on bus.
9. Tracker: invalidates existing track, awaits re-detection from new camera
10. Detector: first frame from telephoto comes in; re-detects target
11. Tracker: matches to predicted position via spatial proximity in new camera
        frame, resumes track with new camera's intrinsics
12. Targeting: proceeds with high-zoom precision aim
```

This is intentionally complex because it has to be. The middle of a target engagement is the worst time to fumble a camera handoff. Plan for ~1 second of "tracking pause" during a switch. Don't switch cameras lightly.

### 3.6 The fallback case

When the phone is temporarily unavailable (USB unplugged, network glitch, app crash, thermal protection kicked in), the PC needs to handle it gracefully:

1. Phone sensor stops emitting frames
2. After 500ms of no frames, mark phone sensor as `unhealthy`
3. If another targeting-role sensor exists (OV9281, Kinect RGB), failover
4. If no fallback targeting sensor, mark system as `degraded`, disable fire authorization until phone returns or operator intervenes
5. When phone reconnects, run a quick calibration freshness check; if recent, resume; if stale, prompt for re-calibration

---

## 4. The Android app

This is the actual code you'd write. Scope: ~2000-3000 lines of Kotlin.

### 4.1 Architecture

```
NoMoSkeetersSensorApp/
├── app/src/main/java/com/nomoskeeters/sensor/
│   ├── MainActivity.kt              # status display, connection UI
│   ├── SensorService.kt             # foreground service — keeps app alive
│   ├── camera/
│   │   ├── CameraManager.kt         # CameraX wrapper for lens switching
│   │   ├── FrameProducer.kt         # produces frames at requested resolution/fps
│   │   └── IntrinsicsProvider.kt    # exposes calibration data per camera
│   ├── network/
│   │   ├── CommandServer.kt         # TCP listener for PC commands
│   │   ├── FrameStreamer.kt         # UDP sender for frame data
│   │   ├── Protocol.kt              # message schema, serialization
│   │   └── HeartbeatManager.kt      # ping/pong with PC
│   ├── encoder/
│   │   ├── H264Encoder.kt           # hardware MediaCodec H.264, low-latency profile
│   │   └── YuvPassthrough.kt        # raw YUV streaming for highest performance
│   ├── recording/
│   │   └── LocalRecorder.kt         # MP4 recording to phone storage
│   ├── thermal/
│   │   └── ThermalMonitor.kt        # watches PowerManager thermal state
│   └── ui/
│       ├── StatusView.kt            # main screen: connection, active camera, fps
│       └── DiagnosticView.kt        # debug panel: bandwidth, frame timing, errors
└── app/build.gradle.kts             # CameraX, OkHttp/Netty, ProtoBuf
```

### 4.2 Key library choices

- **CameraX** for camera access. Not Camera2 directly — CameraX abstracts the differences between phone manufacturers and handles the OnePlus quirks. Slight latency overhead but worth it for reliability.
- **MediaCodec** (not FFmpeg or x264) for H.264 encoding. Hardware-accelerated, lowest latency.
- **Netty or Ktor** for the network layer. Both Kotlin-friendly. Ktor is simpler for this scope.
- **Protobuf** for protocol serialization. Smaller than JSON, faster to parse, schema versioned.
- **Foreground service with notification** so Android doesn't kill the app when the phone screen turns off.

### 4.3 The streaming pipeline

```
CameraX ImageAnalysis use case
   ↓ produces ImageProxy (YUV_420_888)
   ↓
For raw_yuv mode: copy YUV planes into one buffer, send via UDP
For h264 mode: feed to MediaCodec, get encoded output, send via UDP
   ↓
UDP socket to PC's frame port
```

Per-frame metadata (capture timestamp, frame ID, camera ID) is small enough to fit in the UDP packet header. Use sequential frame IDs so the PC can detect drops and reorder if needed.

### 4.4 Hardware MediaCodec for low-latency H.264

The key knobs on Android MediaCodec for low-latency streaming:

```kotlin
val format = MediaFormat.createVideoFormat(MIMETYPE_VIDEO_AVC, width, height).apply {
    setInteger(KEY_BIT_RATE, targetBitrate)
    setInteger(KEY_FRAME_RATE, fps)
    setInteger(KEY_COLOR_FORMAT, COLOR_FormatYUV420Flexible)
    setInteger(KEY_I_FRAME_INTERVAL, 0)   // every frame is a keyframe
    setInteger(KEY_LATENCY, 1)            // low-latency mode (Android 12+)
    setInteger(KEY_PROFILE, AVCProfileBaseline)
    setInteger(KEY_LEVEL, AVCLevel31)
    // disable B-frames
    setInteger(KEY_PRIORITY, 0)           // realtime priority
    setInteger(KEY_OPERATING_RATE, fps * 2)
}
```

The `KEY_LATENCY = 1` flag is the big one — added in Android 12, it tells the hardware encoder to prioritize low latency over compression efficiency. OxygenOS on OnePlus 15 should honor this; verify in practice.

### 4.5 Thermal management

Snapdragon 8 Elite handles sustained NPU/encoder load well but isn't immune to thermal throttling. Monitor `PowerManager.currentThermalStatus` in a coroutine; emit `event:thermal_warning` when it goes from `NONE` to `LIGHT`, take protective action when it hits `MODERATE`.

Protective actions in order:
1. Reduce framerate (60 → 30 fps)
2. Reduce resolution (1080p → 720p)
3. Switch from H.264 quality to H.264 lowlat (less encoder load)
4. Last resort: stop streaming, emit error, await operator intervention

Never silently stop sending frames — the PC interprets that as a failure.

### 4.6 The minimum viable app

If you want to start small and verify the architecture before committing fully, here's the MVP:

**Phase 1 (1 week):** TCP command server, single-camera streaming over UDP in raw YUV, no recording, no lens switching, basic UI showing "connected/not connected." Goal: prove PC can receive frames at lower latency than the Hero 13.

**Phase 2 (1 week):** Add H.264 streaming, lens switching via `set_active_camera`, AF region control. Goal: prove the smart-sensor protocol actually works.

**Phase 3 (1 week):** Add thermal monitoring, local recording, robust reconnect handling, calibration support. Goal: production-grade reliability.

**Phase 4 (ongoing):** Integration testing with the full NoMoSkeeters pipeline, calibration tooling, multi-camera profile management.

3-4 weeks of focused work to reach Phase 3.

---

## 5. Calibration tooling specifically for the phone

This is where the PC-side rewrite needs new code beyond what BOOTSTRAP.md plans for.

### 5.1 The challenge

The phone has multiple cameras. Each has its own intrinsics (focal length, principal point, distortion) AND its own extrinsics relative to the laser galvo. So instead of one homography `phone → galvo`, you need:

```
phone_wide → galvo
phone_main → galvo
phone_telephoto → galvo
phone_ultrawide → galvo
```

Each calibrated independently.

### 5.2 The calibration procedure (per camera)

```
1. PC: Mount phone rigidly. Confirm with operator.
2. PC: command set_active_camera(target_camera)
3. PC: command lock_focus at infinity (or known calibration distance)
4. PC: confirm camera state stable, AF locked
5. PC: For each calibration point in the pattern (grid, halton):
     a. PC commands cube: galvo to (gx, gy), enable output low power
     b. PC waits for laser dot to be stable
     c. PC asks PhoneSensor to capture single frame at current state
     d. PC runs dot detector on the captured frame, gets (px, py)
     e. PC stores correspondence (gx, gy) ↔ (px_normalized, py_normalized)
     f. PC disables laser, moves to next point
6. PC: cube to OFF
7. PC: fit homography H_norm_to_galvo from collected correspondences
8. PC: validate at multiple depths if possible
9. PC: store profile to user_data/calibrations/phone_<camera_id>/<scene>.json
```

The slow part is step 5 — each point takes ~1-2 seconds (laser settle + frame capture + dot detect). 25-point grid = ~30-50 seconds per camera. If you calibrate all four cameras, ~3-4 minutes of setup time per session.

### 5.3 Calibration storage schema

```json
{
  "schema_version": 2,
  "sensor_id": "phone",
  "camera_id": "telephoto",
  "phone_model": "OnePlus 15",
  "scene": "garage_default",
  "mount_config": "tripod_below_laser",
  "calibration_distance_m": 2.0,
  "focus_locked": true,
  "n_points": 25,
  "pattern": "halton",
  "intrinsics": {
    "focal_length_px": [3210.5, 3211.2],
    "principal_point_px": [960, 540],
    "distortion": [0.012, -0.034, 0.001, 0.002, 0.0]
  },
  "H_norm_to_galvo": [[...], [...], [...]],
  "H_galvo_to_norm": [[...], [...], [...]],
  "residual_norm": 0.0058,
  "residual_galvo": 19.7,
  "validation_depths_m": [1.0, 2.5, 4.0],
  "validation_residuals_norm": [0.012, 0.005, 0.007]
}
```

### 5.4 Camera capabilities manifest

On `connect`, the phone reports what cameras it has. PC stores this manifest to know what's available:

```json
{
  "phone_model": "OnePlus 15",
  "cameras": [
    {
      "id": "ultrawide",
      "fov_h_deg": 116,
      "fov_v_deg": 80,
      "max_resolution": [4000, 3000],
      "preferred_streaming_resolution": [1920, 1080],
      "max_fps_at_streaming_res": 60,
      "has_optical_zoom": false,
      "supports_af": true,
      "supports_locked_focus": true
    },
    {
      "id": "main",
      "fov_h_deg": 75,
      "fov_v_deg": 55,
      "max_resolution": [8160, 6120],
      "preferred_streaming_resolution": [1920, 1080],
      "max_fps_at_streaming_res": 60,
      "has_optical_zoom": false,
      "supports_af": true,
      "supports_locked_focus": true,
      "supports_hdr": true
    },
    {
      "id": "telephoto",
      "fov_h_deg": 23,
      "fov_v_deg": 17,
      "max_resolution": [4080, 3060],
      "preferred_streaming_resolution": [1280, 720],
      "max_fps_at_streaming_res": 60,
      "has_optical_zoom": true,
      "optical_zoom_factor": 3.0,
      "supports_af": true,
      "supports_locked_focus": true
    }
  ]
}
```

This drives PC-side decisions about when to switch cameras and what to expect from each.

---

## 6. Mounting and physical setup

Often skipped in software docs but critical here.

### 6.1 The mounting problem

You need to:

- Hold the phone in a known orientation relative to the laser
- Allow USB-C connection to PC for command/frame transport (and power)
- Allow the phone screen to be operator-readable (for debug and manual override)
- Not block any of the cameras (the OnePlus 15 has cameras on the back; mount must expose them)
- Survive sustained use without slipping or heating up

### 6.2 Recommended approach

3D-print a bracket that mounts to the same tripod as the laser. The bracket holds the phone in landscape orientation, cameras facing the operating volume, screen facing the operator. USB-C port accessible from the bottom for cable management.

Critical detail: **the phone's camera and the laser aperture should be co-located within ~10cm if possible.** Same logic as the GoPro mount — small baseline minimizes parallax error and makes the homography approximation valid across the operating volume.

For thermal management, leave the back of the phone exposed (not in a case). The phone needs to radiate heat during sustained operation. A small fan (the kind used for cooling smartphones during gaming) is overkill but reliable.

### 6.3 Power

USB-C from PC may not provide enough power for sustained operation while the phone is encoding video and running cameras. Two solutions:

- **Y-cable**: data to PC, power from a separate USB-C charger
- **Powered USB hub**: PC → hub → phone, with hub providing power

OnePlus 15 supports 100W charging so any modern USB-C PD source works.

---

## 7. Latency budget

Honest accounting. Build this in:

| Stage | Best case | Realistic | Worst case |
|---|---|---|---|
| Phone sensor exposure | 10ms (1080p60) | 16ms | 33ms (30fps) |
| Phone ISP processing | 10ms | 20ms | 40ms |
| MediaCodec encode (h264_lowlat) | 15ms | 30ms | 60ms |
| USB transport | 5ms | 10ms | 20ms |
| PC decode (PyAV/ffmpeg) | 5ms | 15ms | 30ms |
| Numpy conversion | 1ms | 3ms | 5ms |
| **Total to first detection-ready frame** | **46ms** | **94ms** | **188ms** |

For raw YUV mode (skip encode/decode):

| Stage | Best case | Realistic | Worst case |
|---|---|---|---|
| Phone sensor exposure | 10ms | 16ms | 33ms |
| Phone ISP processing | 10ms | 20ms | 40ms |
| USB transport (raw, big) | 15ms | 30ms | 50ms |
| PC numpy convert | 1ms | 3ms | 5ms |
| **Total** | **36ms** | **69ms** | **128ms** |

Both better than the Hero 13 by a wide margin. Neither matches the OV9281's 25ms.

### Targeting math at 100ms latency

Mosquito at 0.5 m/s, 100ms latency = 5cm extrapolation needed for lead aim. The cone-collapse pattern with breach-and-restart handles this well — the cone starts wide enough to accommodate the extrapolation error, shrinks as the tracker gains confidence, restarts if the prediction proves wrong.

### Targeting math at 200ms latency

10cm extrapolation. Pushing the limits but still workable for slower targets (resting mosquitoes, larger insects). Fast-flying mosquitoes get harder — you'd want the OV9281 for those.

---

## 8. Open questions specific to this architecture

| Question | How to answer |
|---|---|
| Does OxygenOS on OnePlus 15 honor `KEY_LATENCY=1`? | Build minimal MediaCodec test, measure encode time with flag on/off |
| What's the actual end-to-end latency of the phone path? | Build Phase 1 MVP, measure photon-to-PC-numpy with timestamped LED |
| Does USB-C provide enough power for sustained operation? | Run for 2 hours, watch battery percentage trend |
| How long do camera switches actually take on OnePlus 15? | Build a test that switches between all cameras, measures `af_settled` time |
| Does the telephoto have enough resolution for mosquito detection at 4m? | Calculate target size in pixels given lens specs, validate with test footage |
| What's the thermal sustain? | Run 30-minute sessions at 1080p60 H.264, log thermal state events |
| Does the IR-pass advantage apply to phone cameras? | Test in dark room with Kinect IR illumination — does the phone see anything? |

---

## 9. The decision matrix

Should you build this? Depends on what you value.

**Build it if:**
- You enjoy Android development or want to learn
- You want multi-camera/optical-zoom capabilities you can't get elsewhere
- You don't want to spend even $40 on an OV9281
- The project is as much about the engineering journey as the mosquito-killing
- You're okay with a 3-4 week detour

**Don't build it if:**
- You want NoMoSkeeters working ASAP
- You're satisfied with the OV9281's capabilities
- You don't want to maintain an Android app long-term
- The phone might get repurposed/upgraded (project becomes brittle)

**Honest read**: For Cole specifically — this is project material that fits the "weaponizing ADHD through deliberate project-splitting for novelty reward" pattern. The phone path is more interesting than buying a camera. It teaches Android dev. It uses hardware you already have. It produces a more impressive end result.

But it's also a sidequest off the main NoMoSkeeters timeline. If the goal is "kill mosquitoes by end of month," buy the OV9281. If the goal is "build the most architecturally interesting version of NoMoSkeeters," the phone path is genuinely the best.

You're allowed to do both. OV9281 for V1 (works in a weekend), phone-as-sensor for V2 (an interesting upgrade path that adds zoom/AF/multi-camera capabilities).

---

## 10. What to do this week if you commit to the phone path

In order:

1. **Buy the OV9281 anyway.** $30-40, becomes your "ground truth" reference camera for comparing phone latency and quality. Also gives you a working V1 fallback while the phone app is in development.

2. **Build the latency measurement harness on PC.** A small tool that flashes an LED visible to the camera and measures the time between LED command and first frame showing the LED. Run it against the GoPro, the Kinect, the OV9281. Establishes baseline numbers and the measurement methodology before you have a phone app to measure.

3. **Start the Android app at Phase 1.** Just streaming a single camera over UDP. Use Android Studio's templates. Don't overthink the architecture yet — get raw YUV flowing.

4. **Measure phone Phase 1 latency** with the harness from step 2. This tells you whether to continue investing.

5. **If Phase 1 latency is < 200ms**, commit to Phases 2-4. If > 200ms, the phone path isn't going to compete with the OV9281; either fix the bottleneck (likely in MediaCodec settings) or pivot back to OV9281.

---

## End

This is an architecture spec, not a plan to execute today. Use it to decide whether to commit to this path and as a reference once you do. The TLDR: it's the best architecture you could pick for NoMoSkeeters if you want maximum capability and don't mind ~3-4 weeks of focused Android development. It's not necessary, but it's interesting.
