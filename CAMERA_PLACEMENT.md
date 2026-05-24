# Camera placement guide

The system supports four cameras of different roles:

| Sensor   | Role                          | Strength                                                    | Weakness                                          |
|----------|-------------------------------|-------------------------------------------------------------|---------------------------------------------------|
| OV9281   | TARGETING (V1 ground truth)   | 24 ms lag, 188 fps, global shutter, IR-sensitive            | Narrow FOV (5–50 mm varifocal), monochrome        |
| Phone    | TARGETING fallback / wide eye | ~194 ms lag, multi-lens (ultrawide/main/tele), high res     | Lag, less consistent under low light              |
| GoPro    | Spare wide camera             | Excellent wide FOV, well-stabilised image                   | 810 ms USB-webcam lag — never use for live target |
| Kinect v2| SAFETY + secondary targeting  | 512×424 depth at ~30 fps, IR flood for darkness, robust   | Limited to ~4.5 m working range; bulky            |

The OV9281 is the only camera fast enough to actually target a moving
mosquito at close range. Everything else either supplements it or supplies
context.

---

## One-camera setup (OV9281 only)

The minimum viable rig. Use this when you're bench-bringing-up the system
or doing a single-target session in a small, controlled space.

```
                    [ ceiling / sky ]
                          |
                          |
   wall +─────────────────┴──────────────── wall
   (target backstop)
                          ▲
                          │  ~1.0 m
                          │
              ┌───────────┴───────────┐
              │   LaserCube + OV9281  │  (bolted rigidly together)
              │   on a tripod         │
              └───────────────────────┘
                          ▲ operator stands BEHIND the rig
```

Rules:
- The OV9281 must be rigidly bolted to the LaserCube — calibration breaks
  on any relative motion.
- Aim the rig at a matte backdrop (drywall, foamboard) so any missed shot
  hits something controlled. Never aim into open space, mirrors, windows,
  reflective metal, or anything you can't predict the bounce off.
- Set the OV9281's manual lens so the calibration box (`tools/fov_box.py`)
  comfortably fits inside the frame with ~10% margin on all sides.
- Operator stands behind the rig with line of sight to the target volume.

Limitations:
- No safety sensor. The operator IS the safety system. Eye protection on
  everyone in the room, and run `--no-kinect` / `--no-person-rgb` on every
  script (you don't have a person detector, so the gate is operator-only).
- No depth. Cone-collapse shot patterns degrade to "blob in front of a
  background", with no z-discrimination.

---

## Two-camera setup (OV9281 + Kinect v2)

The recommended room-scale rig. Adds a safety eye and depth context.

```
          (ceiling-mounted Kinect, looking down + into room)
                                                            ▲ ~2.0–2.5 m
                          [ wall ]                          │
   ┌──────────────────────────────┐                         │
   │            Kinect            │ ◄── wide depth FOV ◄────┘
   └──────────────────────────────┘
                  │   |   |   |
                  │  depth cone (~70°) covers the targeting volume
                  │
                  ▼
   ──────────────────────────── floor ─────────────────────
                          ▲
                          │
              ┌───────────┴───────────┐
              │   LaserCube + OV9281  │  (on a tripod at adult eye level)
              └───────────────────────┘
```

Rules:
- Kinect mounts so its 70° H × 60° V depth cone covers the entire volume
  between the OV9281 and its backdrop. Anything outside this cone is
  invisible to the safety gate.
- Aim the Kinect *into the room*, not at the backdrop. The backdrop
  doesn't move; what we want to see is a person walking into the laser
  line. Mount above head height looking down.
- The Kinect's RGB camera looks at the same volume the OV9281 does, so it
  doubles as a secondary detection sensor. Calibrate it (Step 11,
  `--camera kinect`) to get a CrossSensorExtrinsic with the OV9281, then
  fusion will agree two-sensor sightings before firing.
- Set `SAFETY_KINECT_DEPTH_*` settings to match: `MIN_M = 0.5`, `MAX_M`
  to the actual depth of your room (default 4.5 m is fine for most rooms).
- During calibration the Kinect is allowed to be bumped (it's mobile);
  during a session it MUST NOT be moved. Use a tripod or wall mount.

Limitations:
- The Kinect's depth plane is 512×424 — fine for detecting *a person*,
  too coarse for detecting a mosquito.
- Kinect IR flood will saturate the OV9281's IR sensitivity if they're
  pointed at the same matte surface up close. Aim the Kinect 30°+ off-axis
  from the OV9281, or put them on opposite sides of the room.

---

## Three-camera setup (OV9281 + Kinect + Phone)

Adds the phone as a wide secondary eye. Useful when:
- The OV9281's narrow lens can't cover the whole defended room.
- You want to log incoming flight paths from outside the OV9281 FOV (the
  phone catches them earlier; the OV9281 takes the shot when they enter
  its narrower bounds).

```
                  (Kinect: ceiling, into room)
                                                            ▲
                          [ wall ]                          │ ~2.5 m
                                                            │
   ── phone (tripod, room-corner, ultrawide) ── ▼ ────────┐ │
                                              wide FOV    │ │
              ┌───────────────────────────┐               │ │
              │   LaserCube + OV9281      │ ◄ narrow FOV──┘ │
              │   (tripod, room-centre)   │                 │
              └───────────────────────────┘                 │
   ── floor ─────────────────────────────────────────────  │
```

Rules:
- Phone goes in a corner of the room, ultrawide camera selected
  (`--phone-camera ultrawide`). It needs an unobstructed view of the
  whole volume the OV9281 might target.
- Calibrate the phone (Step 11 with `--camera phone --phone-camera
  ultrawide`) so the system has a phone→galvo homography too — even
  though we don't fire on phone detections, fusion uses the phone's
  detections to *confirm* the OV9281's at long range.
- Set the phone's `LATENCY_SOFTWARE_LAG_MS` only if you're switching the
  primary targeting camera back to the phone. With OV9281 primary, leave
  `LATENCY_SOFTWARE_LAG_MS=24`.

Limitations:
- The phone's 194 ms lag means its detections lag the OV9281 by ~170 ms;
  fusion's track-level alignment (CrossSensorFusion) handles this, but
  any phone-derived velocity estimate is stale by that much.
- More cameras = more compute. On older laptops, the phone's H.264 decode
  alone can eat a core; consider `PHONE_FFMPEG_HWACCEL = "cuda"`
  (already the default).

---

## Four-camera setup (everything, including GoPro)

The GoPro stays as a recording / wide-context sensor — never as the
targeting sensor. Mount on a tripod 1–2 m to the side of the rig pointing
into the room. Use it for:
- Recording the session for post-hoc review (always nice for safety
  evidence).
- Backup wide-FOV detection if the phone is unavailable (with its 810 ms
  lag, GoPro detections are mostly useful as "was there a thing there 1 s
  ago" context, not for firing).

There's no fundamental new constraint over the 3-camera setup — just one
more thing to calibrate (`--camera gopro`) and one more sensor for the
fusion controller to associate.

---

## Calibrating the rig

Once cameras are mounted:

1. `python tools/fov_box.py --camera ov9281 --cam-index <N>` — adjust the
   OV9281 lens until the laser-drawn box fills the frame comfortably.
2. `python scripts/step11_first_light.py` — laser-only safety gate.
3. `python scripts/step11_calibration.py --camera ov9281 --cam-index <N>
   --laser-power 5 --hold 1.0 --settle 0.3` — primary calibration; defaults
   are sized for slow cameras, the 1s/0.3s override is for the OV9281.
4. Repeat 3 for any other targeting-capable camera (`--camera kinect`,
   `--camera phone`).
5. Calibrate the cross-sensor extrinsic (Kinect → OV9281) using
   `targeting/extrinsics.py` if you want fusion to associate detections.

After calibration:
- `python scripts/spotter_mode.py --cam-index <N>` — full pipeline,
  no laser firing. Use this to confirm safety works before going live.
- `python scripts/live_fire_session.py --dry-fire` — pre-flight without
  opening the cube.
- `python scripts/live_fire_session.py --cam-index <N>` — the real run.

---

## Don'ts

- Don't put the OV9281 in a position where its FOV includes a window,
  mirror, or anyone's face by default. The system fires inside that FOV;
  every pixel needs to be a safe-to-paint pixel.
- Don't share a USB hub between the OV9281 and another high-bandwidth
  device. The OV9281 needs USB 2.0 high-speed bandwidth to sustain MJPG
  at high fps.
- Don't run without a backdrop. A missed shot disappearing into the room
  is a Class 3B beam doing exactly what the safety lens was supposed to
  prevent. Always pose a matte surface behind the target volume.
- Don't run without eye protection.
