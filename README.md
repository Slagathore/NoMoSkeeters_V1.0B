# NoMoSkeeters v2

A vision-and-laser system that detects, tracks, and tags mosquitoes with a
laser dot. Operator-supervised, safety-gated, runs as a desktop application on
Windows.

> ⚠️ **Class 3B laser hardware.** This project drives a real laser. Eye
> protection and a safety lens are required for any bench work. The laser is
> never enabled without an explicit operator confirmation step, and every
> live script disables output on exit. Do not run the `scripts/step11_*`
> tools without reading them first.

## Status

This is a **ground-up rewrite**. The v1 codebase had a wholly fictional
LaserCube network protocol — command bytes, packet layouts and sample formats
invented by an earlier coding pass working from memory. v2 exists to replace
it with a protocol verified against the manufacturer source
(`Wickedlasers/libLaserdockCore`) and cross-checked against real hardware.

Steps 0–11 of the amended implementation order are built and tested; the
LaserCube control path and the GoPro video feed are both verified on real
hardware. ~190 tests pass offline. Remaining work is mostly empirical —
bench calibration, the GUI, and the web monitor.

## Hardware & network reference

| Device | Connection | Notes |
|---|---|---|
| WiFi LaserCube 2.5W (FW 0.23) | Direct Ethernet (APIPA) | Laser output + galvo |
| GoPro Hero 13 Black | USB-tethered | Primary targeting camera |
| Kinect v2 | USB (+ Kinect SDK) | Safety / 3D scene |

### LaserCube

- Cube APIPA address: `169.254.40.83`
- PC must bind its source IP on the direct link (`169.254.25.216` in the
  reference rig) — multi-NIC binding is mandatory or UDP replies are lost.
- UDP ports: command `45457`, data `45458`, alive/heartbeat `45456`
- Ring buffer 6000 samples, DAC 30 000 sps, no SHA204 handshake required.

### GoPro Hero 13 (USB)

- The USB network addresses are derived from the camera serial: camera at
  `172.2X.1YZ.51`, the PC lands on the same `/24`. Reference session:
  camera `172.27.109.51`, PC `172.27.109.55`. The third octet changes per
  session — discover it with `tools/gopro_stream_helper.ps1` → `Find-GoPro`.
- HTTP control API: `http://<camera>:8080`
- Video: **webcam mode** (`/gopro/webcam/start`) → H.264 1080p30 MPEG-TS on
  **UDP 8554**. The receiving socket must bind the PC's GoPro-interface IP,
  not `0.0.0.0` (a wildcard bind misses it on a multi-NIC host).

Per-session secrets (GoPro COHN credentials, device serials) live in a
local, untracked `HARDWARE_FINDINGS.md` — they are deliberately not in this
public repo.

## Procedures

### GoPro — connect over USB

1. Power on the GoPro; connect the PC to the GoPro's own WiFi AP.
2. `curl http://10.5.5.9:8080/gopro/camera/control/wired_usb?p=1` — enable
   wired USB mode (may need re-running per power cycle).
3. Plug in the USB-C cable; a `UsbNcm` network adapter enumerates.
4. Disconnect the PC from the GoPro WiFi — USB transport is now active.

### GoPro — see the live feed

```powershell
python tools/gopro_view.py --ip 172.27.109.51        # live preview window
python gp_py_smoke_subprocess.py --ip 172.27.109.51  # headless frame check
```

In code: `GoProSensor(control=GoProInterface(ip="<camera>"), decoder="ffmpeg")`
delivers `SensorFrame`s straight into the pipeline via `SensorManager`.

### Laser bench session

Cube connected, **eye protection + safety lens on**:

```powershell
python scripts/step11_first_light.py                 # safe low-power dot
python scripts/step11_calibration.py --camera gopro --gopro-ip 172.27.109.51 --decoder ffmpeg
```

### Diagnostics

- `tools/gopro_stream_helper.ps1` — PowerShell stream start/stop/find helpers
- `tools/gopro_stream_probe.py` — find which UDP port/IP the camera streams to
- `lasercube_protocol_probe.py` — LaserCube protocol probe
- `sha204_cold_test.py` — confirms the cube emits light without an auth handshake

## Repo layout

```
config/      Settings (single source of truth) + JSON override manager
events/      Bus event dataclasses
sensors/     GoPro, Kinect, local-cam, replay drivers + SensorManager
detection/   Detector + classifier
tracking/    Kalman/IoU tracker, assignment, cross-sensor fusion
targeting/   Calibration, coordinate mapping, sweep patterns
laser/       LaserCube protocol, transports, heartbeat, shot patterns
safety/      No-fire zones, eligibility, decisions
scripts/     Bench runners (first-light, calibration, acceptance)
tools/       Probes, sensor comparison, GoPro stream/view helpers
tests/       Unit + integration tests, golden fixtures
```

The design specification lives in `BOOTSTRAP.md` and its two amendment
documents — those are the canonical reference.

## Quickstart

Requires Python 3.11+.

```powershell
python -m pip install -r requirements.txt
python -m pytest                       # no hardware needed
```

The full pipeline runs offline against a dry-run transport — no cube required
to develop or test.

## License

Not yet licensed — all rights reserved by the author pending a license
decision.
