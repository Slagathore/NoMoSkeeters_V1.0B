# NoMoSkeeters Sensor — Android companion app

The phone half of **NoMoSkeeters Sensor Protocol v1**. This app turns an
Android phone (target: OnePlus 15) into a dumb-on-purpose, command-able smart
camera for the NoMoSkeeters PC. The PC is the brain; this app accepts commands
(switch lens, set AF region, lock focus, start streaming) and ships frames.

It is the counterpart to the PC-side `phone_sensor/` package + `sensors/phone.py`
already in this repo. **The PC code is the canonical contract** — this app was
written to match the bytes that `phone_sensor/protocol.py`,
`phone_sensor/client.py`, and `phone_sensor/frame_decoder.py` actually parse,
not just the design draft in `PHONE_SENSOR_BOOTSTRAP.md`.

---

## How it maps to the PC contract

| PC side (`phone_sensor/…`)                | App side                                   |
|-------------------------------------------|--------------------------------------------|
| `PhoneSensorClient` dials TCP `PHONE_IP:45470` | `net/CommandServer` — phone is the **server** on `:45470` |
| Line-delimited JSON `{type,cmd_id,…}` cmds | `protocol/CommandRouter` parses, replies `{"type":"<cmd>_reply","cmd_id":n,"ok":…}` |
| `PhoneFrameDecoder` binds UDP `:45471`, parses `<4sQQHHHHI` header | `protocol/FramePacket` packs identical bytes; `net/FrameStreamer` sends to the PC |
| Reads decoded `w*h*3` BGR for h264 / per-packet dims for raw | `encoder/H264Encoder` (Annex-B) / `encoder/RawYuvReader` (NV21) |
| Capabilities manifest on `connect`        | `camera/CameraEnumerator` builds it from Camera2 characteristics |
| `event:camera_changed / af_settled / thermal_warning / battery_low` | `SensorEngine` emits them (no `cmd_id`, so the PC never mistakes one for a reply) |
| Heartbeat `ping` every 1s, 3.5s timeout   | `CommandServer` watchdog → safe state (stops stream) on silence |

The frame-packet byte layout is locked by a cross-language test: the Python
`pack_frame_packet` output for a known input is embedded as a golden hex string
in `FramePacketTest`, and the Kotlin packer must reproduce it exactly.

### Who connects to whom

- **TCP commands:** the **PC connects to the phone.** The app listens on
  `0.0.0.0:45470`. So the phone must be reachable at whatever you set `PHONE_IP`
  to on the PC (`config/settings.py`).
- **UDP frames:** the **phone sends to the PC.** The app learns the PC's IP from
  the TCP peer address and streams frames to `<pc-ip>:45471`.

---

## Architecture (`app/src/main/java/com/nomoskeeters/sensor/`)

```
protocol/        pure Kotlin, no Android — JVM-unit-testable
  Protocol.kt        ports, command/event/mode/format constants
  FramePacket.kt     <4sQQHHHHI binary header packer (byte-identical to PC)
  SensorBackend.kt   interface the router delegates device work to
  CommandRouter.kt   JSON command → reply; event builder
net/
  CommandServer.kt   TCP server, JSON framing, dispatch, heartbeat watchdog
  FrameStreamer.kt   UDP frame sender (+ single-datagram size guard)
camera/
  CameraEnumerator.kt  physical lenses → ultrawide/main/telephoto + manifest
  CameraController.kt   Camera2 session, lens switch, AF region, focus lock,
                        manual/auto exposure, AF-settle detection
encoder/
  H264Encoder.kt     MediaCodec surface encoder, low-latency, SPS/PPS prepend
  RawYuvReader.kt    ImageReader YUV_420_888 → NV21
stream/
  StreamPipeline.kt  camera output → packetize → FrameStreamer
thermal/
  ThermalMonitor.kt  PowerManager thermal + battery → events
SensorEngine.kt    the brain: implements SensorBackend, wires everything
SensorService.kt   foreground service (type=camera) hosting the engine
MainActivity.kt    status UI (shows IP/ports/connection/stream/thermal)
```

---

## Two deliberate deviations from `PHONE_SENSOR_BOOTSTRAP.md`

The bootstrap is a design draft; the PC code is the binding contract. Two of its
suggestions were overridden, on purpose:

1. **Camera2, not CameraX** (§4.2 suggested CameraX). The smart-sensor control
   surface — physical-lens switching, AF metering rectangles, manual ISO/shutter
   — is what Camera2 exposes directly and at lower latency. CameraX would have
   abstracted exactly the controls the protocol needs.

2. **Periodic intra-refresh, not every-frame-keyframe** (§4.4's MediaCodec
   snippet set `KEY_I_FRAME_INTERVAL=0`). The PC's UDP frame channel carries
   **one frame per datagram with no reassembly** (`parse_frame_packet` reads a
   single `recvfrom`). A full 1080p IDR every frame would blow the 65507-byte
   IPv4 UDP ceiling and be dropped. Instead the encoder uses `KEY_LATENCY=1`,
   CBR, a long GOP, and cyclic intra-refresh so every packet stays small.

---

## Transport constraints (consequences of the fixed PC protocol)

- **One frame ≤ 65507 bytes.** `FrameStreamer` drops anything larger (with a
  logged warning) rather than fragment into something the PC can't parse. Keep
  the h264 bitrate moderate (default 6 Mbps @ 1080p). If you see "exceeds UDP
  ceiling" warnings, lower the bitrate (`stream_set_target_bitrate`) or drop to
  720p.
- **raw_yuv is small-resolution only.** A 1080p NV21 frame is ~3.1 MB — it can
  never fit one datagram. Use raw_yuv only at ≈256×170 or below; use h264 for
  full resolution. (The PC defaults to `h264_lowlat`, so this is rarely an issue.)
- **h264 resolution must equal what the PC requested.** The PC reads fixed-size
  `w*h*3` BGR frames from ffmpeg, so the encoder streams exactly the `width×height`
  from `stream_start`. Don't change resolution mid-stream unless the PC resizes
  its decoder too.

---

## Build & install

No Gradle wrapper jar is committed (it's a binary). Easiest path:

1. Open the `android/` folder in **Android Studio** (Koala or newer). It will
   sync, download the SDK/build tools, and generate the Gradle wrapper.
2. Plug in the phone (USB debugging on), pick it as the deploy target, Run.

Command line (after the wrapper exists / `gradle wrapper` once):

```
cd android
./gradlew :app:assembleDebug          # build the APK
./gradlew :app:installDebug           # install to a connected device
./gradlew :app:testDebugUnitTest      # run the JVM unit tests
```

Requires JDK 17 and Android SDK 35. **This repo's CI machine has only JDK 8 and
no Android SDK, so the app has not been compiled here** — it was written against
the verified protocol and unit-tested at the protocol layer; first compile +
on-device bring-up happens on your machine.

---

## On-device bring-up checklist

1. Mount the phone per `PHONE_SENSOR_BOOTSTRAP §6` (camera co-located with the
   laser aperture, back exposed for cooling, USB-C for power+data).
2. Launch the app, press **Start**, grant Camera + notification permissions.
3. Read the phone's IP off the top line of the app. Set `PHONE_IP` in
   `config/settings.py` (or pass `--phone-ip`).
4. From the PC: `python tools/phone_probe.py` — should print the capabilities
   manifest and an fps figure. Then `--view` to eyeball the stream.
5. Latency: `python tools/phone_latency.py` (laser + eye protection — Class 3B).
6. Calibrate a lens: `python scripts/step11_calibration.py --camera phone
   --phone-camera main`.

---

## What's deferred

- **Local recording** (`recording_start/stop/list/transfer`) returns `ok:false`
  in v1. Reason: the PC's `phone_sensor` package has **no file-receive path** for
  `recording_transfer`, so there's no consumer to build against, and no current
  PC code path invokes recording. The extension point is a `MediaMuxer` tee off
  the H.264 output in `StreamPipeline` (the encoded NAL units already flow
  through `emit()`); add a PC-side receiver before wiring it.
- **Launcher icon:** uses the default system icon to avoid committing binary
  mipmap assets. Add one in Android Studio if you want branding.
