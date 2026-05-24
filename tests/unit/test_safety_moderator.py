"""SafetyModerator: gate semantics, stale-detection, cooldown, transitions.

The moderator is the only thing that may call cube.enable_output(), so its
state machine has to be airtight. The tests use a fake clock so cooldown +
stale-detection are deterministic.
"""
from __future__ import annotations

import numpy as np
import pytest

from sensors.base import SensorFrame, SensorRole
from safety.safety_moderator import (
    SafetyModerator, SafetyState, make_kinect_person_check,
    make_person_detector_check,
)


class FakeClock:
    def __init__(self) -> None:
        self.t = 1000.0

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


def _frame(sensor_id: str, *, depth=None, rgb=None, role=SensorRole.SAFETY):
    """Construct a SensorFrame with a deterministic timestamp (it isn't read
    by the moderator — `now_fn` drives the clock for stale detection)."""
    f = SensorFrame(timestamp=0.0, sensor_id=sensor_id,
                    sensor_role=role.value)
    f.depth = depth
    f.rgb = rgb
    f.width = 100
    f.height = 100
    return f


def test_unarmed_is_unsafe_by_default() -> None:
    clk = FakeClock()
    mod = SafetyModerator(now_fn=clk)
    v = mod.verdict()
    assert not v.safe
    assert v.state == SafetyState.UNARMED


def test_arming_without_checks_goes_safe_when_no_checks_registered() -> None:
    clk = FakeClock()
    mod = SafetyModerator(now_fn=clk, safe_cooldown_s=0.0)
    v = mod.arm()
    # No checks registered = no SAFETY sensors required = safe after arming.
    assert v.safe
    assert v.state == SafetyState.SAFE


def test_safety_sensor_without_frames_keeps_gate_closed() -> None:
    clk = FakeClock()
    mod = SafetyModerator(now_fn=clk)
    mod.add_check("kinect_v2", lambda f: (True, ""))
    mod.arm()
    v = mod.verdict()
    assert not v.safe
    assert any("no frames yet" in r for r in v.reasons)


def test_first_clean_frame_opens_the_gate() -> None:
    clk = FakeClock()
    mod = SafetyModerator(now_fn=clk, safe_cooldown_s=0.0)
    mod.add_check("kinect_v2", lambda f: (True, ""))
    mod.arm()
    mod.on_frame(_frame("kinect_v2", depth=np.zeros((4, 4), dtype=np.float32)))
    assert mod.verdict().safe


def test_unsafe_check_closes_the_gate_and_lists_reason() -> None:
    clk = FakeClock()
    mod = SafetyModerator(now_fn=clk, safe_cooldown_s=0.0)
    mod.add_check("kinect_v2", lambda f: (False, "person detected"))
    mod.arm()
    mod.on_frame(_frame("kinect_v2", depth=np.zeros((4, 4))))
    v = mod.verdict()
    assert not v.safe
    assert any("person detected" in r for r in v.reasons)
    assert v.state == SafetyState.BLOCKED


def test_unsafe_to_safe_transition_enforces_cooldown() -> None:
    clk = FakeClock()
    mod = SafetyModerator(now_fn=clk, safe_cooldown_s=1.0)

    flag = {"unsafe": True}
    def check(_f):
        return (False, "stub") if flag["unsafe"] else (True, "")
    mod.add_check("kinect_v2", check)
    mod.arm()

    # First frame: unsafe.
    mod.on_frame(_frame("kinect_v2", depth=np.zeros((4, 4))))
    assert mod.verdict().state == SafetyState.BLOCKED

    # Condition clears. Cooldown still holds for <1s.
    flag["unsafe"] = False
    clk.advance(0.4)
    mod.on_frame(_frame("kinect_v2", depth=np.zeros((4, 4))))
    v = mod.verdict()
    assert not v.safe
    assert v.state == SafetyState.COOLDOWN

    # After the cooldown window we transition to safe.
    clk.advance(1.1)
    mod.on_frame(_frame("kinect_v2", depth=np.zeros((4, 4))))
    assert mod.verdict().safe


def test_stale_safety_sensor_closes_the_gate() -> None:
    clk = FakeClock()
    mod = SafetyModerator(now_fn=clk, stale_after_s=0.2, safe_cooldown_s=0.0)
    mod.add_check("kinect_v2", lambda f: (True, ""))
    mod.arm()
    mod.on_frame(_frame("kinect_v2", depth=np.zeros((4, 4))))
    assert mod.verdict().safe

    # Time advances past stale_after_s with no new frame.
    clk.advance(0.5)
    v = mod.verdict()
    assert not v.safe
    assert any("stale" in r for r in v.reasons)


def test_disarm_resets_to_unarmed_and_closes_gate() -> None:
    clk = FakeClock()
    mod = SafetyModerator(now_fn=clk, safe_cooldown_s=0.0)
    mod.add_check("kinect_v2", lambda f: (True, ""))
    mod.arm()
    mod.on_frame(_frame("kinect_v2", depth=np.zeros((4, 4))))
    assert mod.verdict().safe
    mod.disarm()
    v = mod.verdict()
    assert not v.safe
    assert v.state == SafetyState.UNARMED


def test_enable_disable_callbacks_fire_on_transitions() -> None:
    clk = FakeClock()
    mod = SafetyModerator(now_fn=clk, safe_cooldown_s=0.0)
    enable_calls = {"n": 0}
    disable_calls = {"n": 0}
    mod.set_output_callbacks(
        enable=lambda: (enable_calls.__setitem__("n", enable_calls["n"] + 1) or True),
        disable=lambda: (disable_calls.__setitem__("n", disable_calls["n"] + 1) or True),
    )
    flag = {"unsafe": False}
    mod.add_check("kinect_v2", lambda f: (False, "x") if flag["unsafe"] else (True, ""))
    mod.arm()
    mod.on_frame(_frame("kinect_v2", depth=np.zeros((4, 4))))
    assert enable_calls["n"] == 1
    flag["unsafe"] = True
    mod.on_frame(_frame("kinect_v2", depth=np.zeros((4, 4))))
    assert disable_calls["n"] == 1


def test_non_safety_role_frames_are_ignored() -> None:
    clk = FakeClock()
    mod = SafetyModerator(now_fn=clk)
    mod.add_check("ov9281", lambda f: (False, "would block if checked"))
    mod.arm()
    # TARGETING role — moderator should not run the check
    mod.on_frame(_frame("ov9281", role=SensorRole.TARGETING,
                        rgb=np.zeros((4, 4, 3), dtype=np.uint8)))
    # Stale check on the registered sensor still closes the gate, BUT the
    # check itself must not have run (else it would have set "would block").
    v = mod.verdict()
    # We can't observe the "didn't run" directly, but the reason list must
    # NOT contain the BLOCKED-by-check text.
    assert not any("would block" in r for r in v.reasons)


def test_kinect_person_check_blocks_when_person_present() -> None:
    class FakeKinectCheck:
        def is_person_present(self, _depth):
            return True
    chk = make_kinect_person_check(FakeKinectCheck())
    ok, reason = chk(_frame("kinect_v2", depth=np.zeros((4, 4))))
    assert not ok
    assert "person" in reason.lower()


def test_kinect_person_check_unsafe_on_missing_depth() -> None:
    class FakeKinectCheck:
        def is_person_present(self, _depth):
            return False
    chk = make_kinect_person_check(FakeKinectCheck())
    ok, reason = chk(_frame("kinect_v2", depth=None))
    assert not ok
    assert "depth missing" in reason


def test_person_detector_check_blocks_when_detector_returns_hit() -> None:
    class StubDetector:
        def detect(self, _img):
            return [object()]
    chk = make_person_detector_check(StubDetector())
    ok, reason = chk(_frame("ov9281", rgb=np.zeros((4, 4, 3), dtype=np.uint8)))
    assert not ok
    assert "1" in reason


def test_verdict_sink_receives_state_change() -> None:
    clk = FakeClock()
    mod = SafetyModerator(now_fn=clk, safe_cooldown_s=0.0)
    mod.add_check("kinect_v2", lambda f: (True, ""))
    seen = []
    mod.set_verdict_sink(lambda v: seen.append(v))
    mod.arm()
    mod.on_frame(_frame("kinect_v2", depth=np.zeros((4, 4))))
    # At minimum we should have seen the SAFE transition.
    assert any(v.safe for v in seen)
