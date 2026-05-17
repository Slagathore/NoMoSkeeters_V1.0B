"""Step 4 acceptance: heartbeat continues to tick while the main thread is
deliberately blocked. Uses DryRunTransport so no hardware is required."""
import time

from laser.heartbeat import CubeHeartbeat
from laser.transports.dry_run import DryRunTransport


def test_heartbeat_ticks_while_main_thread_blocks():
    t = DryRunTransport()
    t.connect()
    hb = CubeHeartbeat(transport=t, interval_s=0.05)
    hb.start()
    try:
        # Block the main thread for ~0.5s; heartbeat should still tick.
        time.sleep(0.5)
    finally:
        hb.stop(timeout_s=1.0)
    # At 0.05s interval over 0.5s we expect roughly 8-10 ticks; assert ≥ 5
    # to leave headroom for scheduler jitter on a busy CI box.
    assert hb.tick_count >= 5
    assert hb.last_info is not None
    assert hb.last_info.serial_number == "c4:5b:be:88:53:24"


def test_heartbeat_status_callback_invoked_per_tick():
    t = DryRunTransport()
    t.connect()
    seen = []
    hb = CubeHeartbeat(transport=t, interval_s=0.04,
                      on_status=lambda info: seen.append(info))
    hb.start()
    try:
        time.sleep(0.25)
    finally:
        hb.stop(timeout_s=1.0)
    assert len(seen) >= 3
    assert all(info.fw_minor == 23 for info in seen)


def test_heartbeat_swallows_callback_exceptions():
    """A misbehaving status callback must not stop the heartbeat or crash
    the host process."""
    t = DryRunTransport()
    t.connect()
    raised = [0]
    def bad_cb(_info):
        raised[0] += 1
        raise RuntimeError("boom")
    hb = CubeHeartbeat(transport=t, interval_s=0.04, on_status=bad_cb)
    hb.start()
    try:
        time.sleep(0.25)
    finally:
        hb.stop(timeout_s=1.0)
    assert raised[0] >= 3                 # callback was invoked repeatedly
    assert hb.tick_count >= 3             # ticks continued despite exceptions


def test_heartbeat_stop_is_idempotent():
    t = DryRunTransport()
    t.connect()
    hb = CubeHeartbeat(transport=t, interval_s=0.05)
    hb.start()
    time.sleep(0.1)
    hb.stop()
    hb.stop()                              # second stop must not raise
