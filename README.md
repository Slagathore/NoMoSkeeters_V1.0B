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
(`Wickedlasers/libLaserdockCore`) and cross-checked against real hardware with
a probe tool.

Steps 0–11 of the amended implementation order are built and tested:

- **Offline core** — event schemas, settings/config, LaserCube protocol
  parsers, transport ABC + dry-run transport, heartbeat.
- **Live LaserCube** — UDP transport, source-IP auto-detection; the SHA204
  cold-test gate passed on real hardware (no auth handshake required).
- **Targeting** — calibration patterns, coordinate mapper, multi-depth
  validation.
- **Detection / tracking** — background-subtraction detector, heuristic + ML
  classifier, multi-mode Kalman/IoU tracker.
- **Sensors** — GoPro Hero 13 (USB-tethered), Kinect v2, local webcam,
  replay; a threaded `SensorManager`.

178 tests pass offline. Remaining work is mostly empirical — bench validation,
the GUI, and the web monitor.

## Hardware

| Device | Role |
|---|---|
| WiFi LaserCube 2.5W (FW 0.23) | Laser output, galvo steering |
| GoPro Hero 13 Black | Primary targeting camera (USB-tethered preview) |
| Kinect v2 | Safety / 3D scene understanding (and optionally targeting) |

The GoPro is rigidly mounted to the laser body; the Kinect is mobile and
calibrated per session.

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
tools/       Probe, sensor-comparison, GoPro stream helpers
tests/       Unit + integration tests, golden fixtures
```

The design specification lives in `BOOTSTRAP.md` and its two amendment
documents — those are the canonical reference.

## Quickstart

Requires Python 3.11+.

```powershell
python -m pip install -r requirements.txt
python -m pytest                       # 178 tests, no hardware needed
```

The full pipeline runs offline against a dry-run transport — no cube required
to develop or test.

### Bench session (hardware)

With the cube and camera connected, eye protection on, and a safety lens
fitted:

```powershell
python scripts/step11_first_light.py    # safe low-power centre dot
python scripts/step11_calibration.py --camera gopro --gopro-ip 172.X.Y.51
```

For the GoPro USB connection ceremony and stream helpers, see
`tools/gopro_stream_helper.ps1`.

## License

Not yet licensed — all rights reserved by the author pending a license
decision.
