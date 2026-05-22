"""KillCam — fire detection, debounce, per-shot record + continuous hilight."""
from __future__ import annotations

from capture.kill_cam import MODE_CONTINUOUS, KillCam


class FakeGoPro:
    def __init__(self):
        self.calls: list = []

    def load_preset(self, pid):
        self.calls.append(("load_preset", pid)); return True

    def start_recording(self):
        self.calls.append(("start",)); return True

    def stop_recording(self):
        self.calls.append(("stop",)); return True

    def hilight(self):
        self.calls.append(("hilight",)); return True


class _Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


def _kc(gp, **over):
    over.setdefault("record_async", False)        # run inline for tests
    over.setdefault("debounce_seconds", 2.0)
    over.setdefault("sleeper", lambda s: None)
    return KillCam(gp, **over)


def test_per_shot_records_a_clip():
    gp = FakeGoPro()
    slept: list = []
    kc = _kc(gp, mode="per_shot", slomo_preset_id=0x1234,
             record_seconds=3.0, sleeper=slept.append)
    kc.on_laser_event({"op": "shoot", "track_id": 1})
    assert gp.calls == [("load_preset", 0x1234), ("start",), ("stop",)]
    assert slept == [3.0]
    assert kc.shots_witnessed == 1


def test_debounce_collapses_a_burst():
    gp = FakeGoPro()
    clk = _Clock()
    kc = _kc(gp, mode="per_shot", clock=clk, debounce_seconds=2.0)
    kc.on_laser_event({"op": "shoot"})
    clk.t += 0.5
    kc.on_laser_event({"op": "shoot"})            # within debounce → ignored
    assert kc.shots_witnessed == 1
    clk.t += 2.0
    kc.on_laser_event({"op": "shoot"})            # debounce elapsed → fires
    assert kc.shots_witnessed == 2


def test_non_fire_ops_ignored():
    gp = FakeGoPro()
    kc = _kc(gp)
    kc.on_laser_event({"op": "cone_reacquire", "n": 1})
    kc.on_laser_event({"op": "cone_abort", "reason": "breach"})
    assert gp.calls == []
    assert kc.shots_witnessed == 0


def test_fired_mode_respects_fired_flag():
    gp = FakeGoPro()
    kc = _kc(gp, trigger_on="fired")
    kc.on_laser_event({"op": "cone_done", "fired": False})
    assert kc.shots_witnessed == 0                # cone aborted before bzzt
    kc.on_laser_event({"op": "cone_done", "fired": True})
    assert kc.shots_witnessed == 1


def test_any_shot_mode_triggers_on_cone_regardless():
    gp = FakeGoPro()
    kc = _kc(gp, trigger_on="any_shot")
    kc.on_laser_event({"op": "cone_done", "fired": False})
    assert kc.shots_witnessed == 1


def test_continuous_mode_records_once_and_hilights_each_shot():
    gp = FakeGoPro()
    kc = _kc(gp, mode=MODE_CONTINUOUS, slomo_preset_id=7)
    kc.start()
    assert ("load_preset", 7) in gp.calls and ("start",) in gp.calls
    kc.on_laser_event({"op": "shoot"})
    assert ("hilight",) in gp.calls
    kc.stop()
    assert gp.calls[-1] == ("stop",)


def test_record_failure_does_not_raise():
    class BrokenGoPro(FakeGoPro):
        def start_recording(self):
            raise OSError("camera offline")
    kc = _kc(BrokenGoPro(), mode="per_shot")
    kc.on_laser_event({"op": "shoot"})            # must not propagate
    assert kc.shots_witnessed == 1
