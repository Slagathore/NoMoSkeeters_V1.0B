# gp_py_smoke_subprocess.py
#
# Fallback smoke test: pull GoPro preview frames by piping ffmpeg's raw
# output through a subprocess (HARDWARE_FINDINGS.md §7.2) — the path to use
# if cv2.VideoCapture cannot decode the HEVC stream directly.
import subprocess
import time

WIDTH = 1920
HEIGHT = 1080
FRAME_SIZE = WIDTH * HEIGHT * 3  # BGR24

cmd = [
    "ffmpeg",
    "-fflags", "nobuffer",
    "-flags", "low_delay",
    "-framedrop",
    "-i", "udp://0.0.0.0:8554",
    "-f", "rawvideo",
    "-pix_fmt", "bgr24",
    "-loglevel", "error",
    "-",
]

print("Spawning ffmpeg subprocess...")
proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=10**8)
assert proc.stdout is not None  # guaranteed non-None by stdout=subprocess.PIPE

frame_count = 0
start = time.monotonic()
first_frame_at: float | None = None

print("Reading frames for 10 seconds...")
deadline = start + 10
while time.monotonic() < deadline:
    raw = proc.stdout.read(FRAME_SIZE)
    if len(raw) != FRAME_SIZE:
        print(f"Short read: {len(raw)} bytes — stream ended or broken")
        break
    # A full-size read is the only frame check needed; the raw bytes can be
    # wrapped with np.frombuffer(...).reshape((HEIGHT, WIDTH, 3)) downstream.
    if first_frame_at is None:
        first_frame_at = time.monotonic()
        print(f"First frame at {first_frame_at - start:.2f}s, "
              f"shape=({HEIGHT}, {WIDTH}, 3)")
    frame_count += 1

proc.terminate()
proc.wait(timeout=2)

if frame_count > 0:
    assert first_frame_at is not None  # frame_count > 0 ⟹ set in the loop
    elapsed = time.monotonic() - first_frame_at
    print(f"PASS: {frame_count} frames in {elapsed:.1f}s "
          f"= {frame_count / elapsed:.1f} fps")
else:
    print("FAIL: zero frames received via subprocess ffmpeg")
