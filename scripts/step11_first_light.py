"""Step 11a — first light. The first time the laser actually emits.

Run this BEFORE step11_calibration.py. It connects the cube, runs a pre-flight
check, asks for an explicit operator go-ahead, then emits a single steady
LOW-POWER dot at galvo centre for a few seconds so you can confirm the beam
is real, aimed sanely, and at a safe power. It always disables output on exit.

    python scripts/step11_first_light.py [--power-pct 5] [--duration 5]

SAFETY: Class 3B laser. Eye protection on. Safety lens fitted. Beam pointed at
a matte surface, never toward people, windows, or reflective objects.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from laser.lasercube import LaserCubeInterface          # noqa: E402
from laser.types import LaserPoint                      # noqa: E402

_CENTRE_DAC = 0x800          # ~galvo centre (2048 of 4095)
_DWELL_SAMPLES = 3000        # ~0.1 s of scan at 30 kpps


def _rgb_level(power_pct: int) -> int:
    return max(0, min(0xFFF, (power_pct * 0xFFF) // 100))


def _preflight(cube: LaserCubeInterface) -> bool:
    info = cube.get_full_info()
    if info is None:
        print("FAIL: no GET_FULL_INFO response")
        return False
    print(f"  firmware     : {info.fw_major}.{info.fw_minor}")
    print(f"  interlock    : {info.interlock}")
    print(f"  output now   : {info.output_enabled}")
    print(f"  over_temp    : {info.over_temp}")
    print(f"  temp_warn    : {info.temp_warn}")
    print(f"  temperature  : {info.temperature_c} degC")
    if not info.interlock:
        print("FAIL: interlock not satisfied")
        return False
    if info.over_temp:
        print("FAIL: cube reports over-temperature")
        return False
    return True


def _confirm(prompt: str) -> bool:
    return input(f"{prompt} [type 'yes' to proceed] ").strip().lower() == "yes"


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="LaserCube first-light test.")
    parser.add_argument("--ip", default="169.254.40.83")
    parser.add_argument("--src-ip", default="auto")
    parser.add_argument("--power-pct", type=int, default=5,
                        help="laser power 0-100 (keep LOW for first light)")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="seconds to hold the test dot")
    args = parser.parse_args(argv)

    print("=" * 60)
    print("  Step 11a — LaserCube FIRST LIGHT")
    print("  Class 3B laser. Eye protection + safety lens REQUIRED.")
    print("=" * 60)

    cube = LaserCubeInterface(ip=args.ip, src_ip=args.src_ip)
    if not cube.connect():
        print("FAIL: could not connect to the cube")
        return 1
    print(f"connected (src_ip={cube.src_ip})\n")

    try:
        print("Pre-flight:")
        if not _preflight(cube):
            return 1
        print()
        if not _confirm(f"Emit a {args.power_pct}%-power dot for "
                        f"{args.duration:.0f}s?"):
            print("aborted by operator")
            return 0

        rgb = _rgb_level(args.power_pct)
        dot = [LaserPoint(x=_CENTRE_DAC, y=_CENTRE_DAC, r=rgb, g=rgb, b=rgb)]
        dwell = dot * _DWELL_SAMPLES

        print(f"\nLASER ON — {args.power_pct}% power at galvo centre")
        cube.enable_output()
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            cube.send_frame(dwell, frame_num=0)
            time.sleep(0.05)
    finally:
        cube.disable_output()
        print("LASER OFF")
        cube.disconnect()
        print("disconnected")

    print()
    if _confirm("Did you see a single steady dot at a safe power?"):
        print("PASS — first light confirmed. Next: step11_calibration.py")
        return 0
    print("NOT confirmed — investigate before calibrating "
          "(power, aim, interlock, beam path).")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
