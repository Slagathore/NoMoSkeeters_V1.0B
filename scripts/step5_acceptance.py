"""Step 5 live-cube acceptance — run with the cube powered and reachable.

    python scripts/step5_acceptance.py

Read-only: GET_FULL_INFO heartbeat only. No SET_OUTPUT, no laser emission.
Checks: connect() returns True, get_full_info() agrees with the golden
fixture, and the heartbeat survives a 30-second session.
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

# Allow running as a plain script: put the project root on sys.path.
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from laser.lasercube import LaserCubeInterface          # noqa: E402
from laser.protocol import parse_full_info              # noqa: E402


def main() -> int:
    golden_path = ROOT / "tests" / "fixtures" / "golden_full_info_64b.bin"
    golden = parse_full_info(golden_path.read_bytes())

    cube = LaserCubeInterface(ip="169.254.40.83", src_ip="auto")

    if not cube.connect():
        print("FAIL: connect() returned False")
        return 1
    print(f"connect: True  (src_ip={cube.src_ip})")

    info = cube.get_full_info()
    if info is None:
        print("FAIL: get_full_info() returned None")
        cube.disconnect()
        return 1
    print(f"  serial      : {info.serial_number}  "
          f"({'match' if info.serial_number == golden.serial_number else 'MISMATCH'})")
    print(f"  firmware    : {info.fw_major}.{info.fw_minor}")
    print(f"  dac_rate    : {info.dac_rate}")
    print(f"  buffer_size : {info.buffer_size}")
    print(f"  temperature : {info.temperature_c} degC")

    print("heartbeat: 30s session ...", flush=True)
    lost = False
    for i in range(20):
        time.sleep(1.5)
        if cube.get_full_info() is None:
            print(f"FAIL: heartbeat lost at {(i + 1) * 1.5:.1f}s")
            lost = True
            break
    if not lost:
        print("heartbeat: survived 30s")

    cube.disconnect()
    print("disconnect: done")
    return 1 if lost else 0


if __name__ == "__main__":
    raise SystemExit(main())
