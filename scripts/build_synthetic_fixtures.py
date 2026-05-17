"""Build synthetic LaserCube response fixtures from the verified probe output.

Run: python scripts/build_synthetic_fixtures.py

Writes:
  tests/fixtures/synth_full_info_64b.bin       (matches Cole's probe @ 2026-05-09)
  tests/fixtures/synth_ringbuf_empty_4b.bin    (full buffer, idle)
  tests/fixtures/synth_alive_2b.bin            (canonical [0x27, 0x00])

These are NOT a substitute for `lasercube_protocol_probe.py --save-raw <dir>`
captured from the actual cube. They unblock parser tests until a real capture
exists; once you have real bytes, replace these files (keeping the same name)
or rename the real captures to `golden_*.bin` and update tests accordingly.
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from laser.protocol import synth_full_info, synth_ringbuf_empty, synth_alive


FIXTURES_DIR = ROOT / "tests" / "fixtures"


def main() -> None:
    FIXTURES_DIR.mkdir(parents=True, exist_ok=True)
    pairs = [
        ("synth_full_info_64b.bin",     synth_full_info()),
        ("synth_ringbuf_empty_4b.bin",  synth_ringbuf_empty(empty_count=6000)),
        ("synth_alive_2b.bin",          synth_alive()),
    ]
    for name, blob in pairs:
        path = FIXTURES_DIR / name
        path.write_bytes(blob)
        print(f"wrote {len(blob):>3}B -> {path}")


if __name__ == "__main__":
    main()
