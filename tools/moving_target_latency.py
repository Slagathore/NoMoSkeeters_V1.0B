"""moving_target_latency.py — end-to-end latency under motion.

The static-dot probes (`*_latency.py`) measure command->seen time for a
laser that turns on at one position and never moves. That's the right
metric for the calibration pattern, but it understates real-world targeting
lag: the cube's ringbuffer is *one frame* deep, but a moving target keeps
moving through the lag window. This tool measures lag under that motion:
the galvo sweeps a known sinusoid, the camera observes it, and we recover
the time-offset that best aligns commanded position with observed position.

The recovered lag is the "useful" end-to-end lag for lead-aim purposes —
the value `LaserManager.software_lag_ms` should track when the camera and
cube are both at full tilt.

>>> Class 3B laser. Eye protection + safety lens REQUIRED. <<<

    python tools/moving_target_latency.py --camera ov9281 --cam-index 2
    python tools/moving_target_latency.py --camera ov9281 --cam-index 2 --freq-hz 1.5
"""
from __future__ import annotations

import argparse
import math
import statistics
import sys
import time
from pathlib import Path
from typing import Optional

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2                                                       # noqa: E402
import numpy as np                                               # noqa: E402

from config import settings                                       # noqa: E402
from laser.lasercube import LaserCubeInterface                    # noqa: E402
from laser.types import LaserPoint                                # noqa: E402
from sensors.ov9281 import OV9281Sensor                           # noqa: E402
from targeting.dot_detector import LaserDotDetector               # noqa: E402
from targeting.patterns import galvo_norm_to_dac                  # noqa: E402


def _build_sweep_chunk(t0: float, duration_s: float, freq_hz: float,
                       amplitude_norm: float, centre_norm: float,
                       samples_per_chunk: int, dac_rate: int,
                       laser_power: float) -> tuple[list, float, list]:
    """Build one chunk of LaserPoints tracing a horizontal sinusoid plus the
    per-sample (timestamp, commanded x_norm) so the receiver can correlate
    observed dot positions with commanded ones.

    The chunk represents `samples_per_chunk / dac_rate` seconds of motion
    starting at `t0`. Centre is fixed in y; x oscillates as a sinusoid.
    Returns (laser_points, end_time, commanded_log)."""
    lp = max(0, min(0xFFF, int(round(0xFFF * laser_power / 100.0))))
    points: list = []
    log: list = []
    gy = galvo_norm_to_dac(centre_norm)
    omega = 2.0 * math.pi * freq_hz
    for i in range(samples_per_chunk):
        t = t0 + i / dac_rate
        x_n = centre_norm + amplitude_norm * math.sin(omega * t)
        gx = galvo_norm_to_dac(x_n)
        points.append(LaserPoint(x=gx, y=gy, r=lp, g=lp, b=lp))
        log.append((t, x_n))
    return points, t0 + samples_per_chunk / dac_rate, log


def _correlate_delay(observations: list[tuple[float, float]],
                     commands: list[tuple[float, float]],
                     max_lag_ms: float = 1500.0) -> tuple[float, float]:
    """Return (best_lag_s, correlation) of observations vs commands.

    Both inputs are (timestamp, x_norm). We sweep candidate lags in 1ms
    steps and pick the lag that maximises the Pearson correlation between
    the observed x and the commanded x interpolated to (obs_t - lag).
    """
    if len(observations) < 8 or len(commands) < 8:
        return float("nan"), float("nan")
    cmd_t = np.array([t for t, _ in commands])
    cmd_x = np.array([x for _, x in commands])
    obs_t = np.array([t for t, _ in observations])
    obs_x = np.array([x for _, x in observations])

    best_lag = 0.0
    best_corr = -2.0
    for lag_ms in range(0, int(max_lag_ms) + 1):
        lag_s = lag_ms / 1000.0
        sample_t = obs_t - lag_s
        # Clip to the commanded window; otherwise np.interp extrapolates.
        if sample_t.min() < cmd_t[0] or sample_t.max() > cmd_t[-1]:
            continue
        cmd_aligned = np.interp(sample_t, cmd_t, cmd_x)
        if cmd_aligned.std() < 1e-9 or obs_x.std() < 1e-9:
            continue
        corr = float(np.corrcoef(obs_x, cmd_aligned)[0, 1])
        if corr > best_corr:
            best_corr = corr
            best_lag = lag_s
    return best_lag, best_corr


def main(argv=None) -> int:                                  # noqa: C901
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default="169.254.40.83")
    ap.add_argument("--src-ip", default="auto")
    ap.add_argument("--camera", choices=["ov9281"], default="ov9281")
    ap.add_argument("--cam-index", type=int,
                    default=settings.OV9281_DEVICE_INDEX)
    ap.add_argument("--freq-hz", type=float, default=1.0,
                    help="sinusoid frequency in Hz (default 1)")
    ap.add_argument("--amplitude", type=float, default=0.25,
                    help="peak amplitude in galvo-norm units (default 0.25)")
    ap.add_argument("--centre", type=float, default=0.5)
    ap.add_argument("--duration", type=float, default=8.0,
                    help="seconds to sweep + observe (default 8)")
    ap.add_argument("--samples-per-chunk", type=int, default=2000,
                    help="cube samples per send_frame (default 2000)")
    ap.add_argument("--dac-rate", type=int,
                    default=settings.LASERCUBE_DEFAULT_DAC_RATE)
    ap.add_argument("--laser-power", type=float, default=30.0)
    args = ap.parse_args(argv)

    print("=" * 60)
    print("  Moving-target latency probe")
    print("  Class 3B laser. Eye protection + safety lens REQUIRED.")
    print(f"  sinusoid: f={args.freq_hz}Hz  amp={args.amplitude}  "
          f"centre=({args.centre},{args.centre})  duration={args.duration}s")
    print("=" * 60)

    cube = LaserCubeInterface(ip=args.ip, src_ip=args.src_ip)
    if not cube.connect():
        print("FAIL: could not connect to the cube")
        return 1
    print(f"connected (src_ip={cube.src_ip})")

    if args.camera != "ov9281":
        print(f"unsupported camera: {args.camera}")
        cube.disconnect()
        return 1
    sensor = OV9281Sensor(device_index=args.cam_index, auto_exposure=False)
    if not sensor.open():
        cube.disconnect()
        print(f"FAIL: could not open OV9281 idx {args.cam_index}")
        return 1
    print(f"camera: {sensor.sensor_id} opened")

    if input("Fire the laser for the moving-target probe? [type 'yes'] "
             ).strip().lower() != "yes":
        sensor.close(); cube.disconnect()
        print("aborted"); return 0

    detector = LaserDotDetector()

    # Capture a 1-second background reference with the laser blanked at the
    # centre — used by detect_diff to ignore ambient lighting.
    blank = [LaserPoint(x=galvo_norm_to_dac(args.centre),
                        y=galvo_norm_to_dac(args.centre),
                        r=0, g=0, b=0)] * 1000
    print("capturing background reference...")
    bg_stack: list = []
    t_bg = time.monotonic()
    while time.monotonic() - t_bg < 1.0:
        cube.send_frame(blank, frame_num=0)
        f = sensor.read()
        if f is not None and f.rgb is not None:
            bg_stack.append(f.rgb.astype(np.float32))
    background = (np.mean(bg_stack, axis=0).astype(np.uint8)
                  if bg_stack else None)
    if background is None:
        print("FAIL: no background captured"); sensor.close(); cube.disconnect()
        return 1

    cube.enable_output()
    print("LASER ON — running sinusoidal sweep")

    commands: list[tuple[float, float]] = []     # (sample_t, x_norm)
    observations: list[tuple[float, float]] = []  # (frame_t, x_norm)

    try:
        t_start = time.monotonic()
        # Run the sinusoid by streaming chunks; in parallel read frames.
        t_cursor = t_start
        chunk_dt = args.samples_per_chunk / args.dac_rate
        while time.monotonic() - t_start < args.duration:
            chunk, t_cursor, log = _build_sweep_chunk(
                t_cursor - t_start,    # cube time-origin: t_start
                duration_s=chunk_dt,
                freq_hz=args.freq_hz, amplitude_norm=args.amplitude,
                centre_norm=args.centre,
                samples_per_chunk=args.samples_per_chunk,
                dac_rate=args.dac_rate,
                laser_power=args.laser_power)
            cube.send_frame(chunk, frame_num=0)
            commands.extend((t + t_start, x_n) for t, x_n in log)
            # Read whatever frames came in during this chunk.
            t_chunk_start = time.monotonic()
            while time.monotonic() - t_chunk_start < chunk_dt:
                f = sensor.read()
                if f is None or f.rgb is None:
                    continue
                obs = detector.detect_diff(f.rgb, background)
                if obs is not None:
                    observations.append((time.monotonic(), obs.x_norm))
    finally:
        cube.disable_output()
        sensor.close()
        cube.disconnect()
        print("LASER OFF; disconnected")

    print(f"  commanded samples: {len(commands)}")
    print(f"  observed dots    : {len(observations)}")
    if len(observations) < 16:
        print("not enough observations to fit lag — abort")
        return 1

    # The commanded log is at the *sample emission time*; the cube buffers
    # them, so subtract a buffer offset from cmd_t to get the photons-leave
    # time. For correlation we don't need this — both sides have the same
    # offset, so the recovered lag is photons->numpy-pixel, not photons->DAC.
    lag_s, corr = _correlate_delay(observations, commands)
    print("-" * 60)
    if math.isnan(lag_s):
        print("correlation failed (insufficient overlap?)")
        return 1
    lag_ms = lag_s * 1000.0
    print(f"  best-fit lag (cmd -> observed): {lag_ms:.1f} ms")
    print(f"  Pearson correlation at that lag: {corr:.3f}")
    if corr < 0.5:
        print("  WARNING: low correlation — sinusoid may be aliasing with "
              "the camera frame rate; try --freq-hz 0.5 or 2.0")
    # Useful derived figures.
    omega = 2.0 * math.pi * args.freq_hz
    peak_v_norm_per_s = args.amplitude * omega
    miss_norm = peak_v_norm_per_s * lag_s
    print(f"  peak velocity     : {peak_v_norm_per_s:.3f} norm/s")
    print(f"  miss without lead : {miss_norm:.3f} norm  "
          f"(set LATENCY_SOFTWARE_LAG_MS≈{lag_ms:.0f} to nullify)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
