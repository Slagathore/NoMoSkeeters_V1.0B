"""SafetyModerator — owns the laser-output fire gate.

Per §9.11 / Step 12 the safety layer is the only thing that should ever
call `LaserCubeInterface.enable_output()`. `LaserManager` streams sample
data unconditionally; the cube only emits light if this gate has said so.

The moderator consumes frames from sensors that carry the SAFETY role
(typically the Kinect; an RGB camera with `HOGPersonDetector` may also
be wired in). It runs each configured check on every incoming frame,
combines the results, and exposes a single `is_safe_to_fire()` plus a
verdict feed for logging / GUI.

Architecture goals:
  - The gate is *fail-safe*: if a SAFETY-role sensor goes stale (no
    frames for `stale_after_s`) the gate closes. A broken safety sensor
    must not silently permit firing.
  - Re-arm has a *cooldown*: when an unsafe condition clears, firing
    stays inhibited for `safe_cooldown_s` so a person who half-stepped
    out doesn't immediately re-enable the beam.
  - Voice warnings are optional and cooldown-throttled
    (see `safety.voice_warning`).
  - The actual call to `cube.enable_output()` / `cube.disable_output()`
    is delegated via callbacks — that keeps this module free of laser
    imports and easy to unit-test.

Reference: BOOTSTRAP.md §5 (sensor roles), §12 (safety),
laser/laser_manager.py docstring.
"""
from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Optional

import numpy as np

from sensors.base import SensorFrame, SensorRole

_log = logging.getLogger(__name__)

_Now = Callable[[], float]


class SafetyState(str, Enum):
    """Coarse-grained moderator state. Drives the verdict feed + UI."""
    UNARMED = "unarmed"          # gate explicitly closed by operator
    BLOCKED = "blocked"          # an unsafe condition is current
    COOLDOWN = "cooldown"        # condition cleared, waiting out re-arm timer
    SAFE = "safe"                # all checks clear, gate open


@dataclass
class SafetyVerdict:
    """Snapshot of the moderator's decision at one instant.

    `safe` is the bit the laser-side cares about. `reasons` is the
    operator-facing list of why we're (un)safe. `state` is the
    finer-grained state for UI/logging."""
    ts: float
    safe: bool
    state: SafetyState
    reasons: list[str] = field(default_factory=list)


# A safety check takes a frame and returns (ok, reason). Reason is empty
# when ok=True. Checks may also return ok=True with a non-empty reason for
# informational notes (e.g. "person detected outside no-fire volume" —
# we still fire but the note shows up in the verdict).
SafetyCheck = Callable[[SensorFrame], tuple[bool, str]]


class SafetyModerator:
    """Combines safety checks across SAFETY-role sensors into a single gate.

    Wire it up by:
      1. `add_check(sensor_id, check_fn)` for each per-sensor check.
      2. Subscribe `on_frame` as the SensorManager frame sink.
      3. `set_output_callbacks(enable, disable)` so the moderator can flip
         the cube's emission gate when state transitions.
      4. Optionally `set_voice_warning(VoiceWarning())` for spoken alerts.
      5. `arm()` when the operator confirms the area is clear.

    The gate is `is_safe_to_fire()`. Code that *fires* — i.e. that streams
    samples expecting they will leave the aperture — must check this
    before every shot, not just at session start.
    """

    def __init__(self,
                 stale_after_s: float = 0.5,
                 safe_cooldown_s: float = 1.0,
                 require_explicit_arm: bool = True,
                 allow_no_checks: bool = False,
                 now_fn: _Now = time.monotonic):
        self._checks: dict[str, list[SafetyCheck]] = {}
        self._stale_after_s = max(0.01, stale_after_s)
        self._stale_after_overrides: dict[str, float] = {}
        self._safe_cooldown_s = max(0.0, safe_cooldown_s)
        self._require_arm = require_explicit_arm
        # A moderator with zero registered checks holds the gate closed —
        # arming with no automated person check is exactly the hole this
        # class exists to plug. Laser-free callers (spotter mode, dry-fire
        # smoke tests) may opt out with allow_no_checks=True.
        self._allow_no_checks = allow_no_checks
        self._now = now_fn

        # Operator arm switch.
        self._armed = not require_explicit_arm

        # Internal state.
        self._lock = threading.Lock()
        self._latest_safe_event_ts: float = 0.0
        self._verdict = SafetyVerdict(
            ts=now_fn(), safe=False, state=SafetyState.UNARMED,
            reasons=["unarmed"] if require_explicit_arm else ["no frames yet"])
        # `_last_frame_ts[sensor_id]` = monotonic ts of the last frame seen
        # from that SAFETY sensor. Stale-detection uses this.
        self._last_frame_ts: dict[str, float] = {}
        # `_check_results[sensor_id]` = the most recent (ok, reason) tuple
        # the checks produced for that sensor. The aggregate verdict is
        # rebuilt from these on every update.
        self._check_results: dict[str, list[tuple[bool, str]]] = {}

        self._enable_output_cb: Optional[Callable[[], bool]] = None
        self._disable_output_cb: Optional[Callable[[], bool]] = None
        self._voice: Optional["VoiceWarningProto"] = None
        self._on_verdict: Optional[Callable[[SafetyVerdict], None]] = None

    # ── Configuration ────────────────────────────────────────────────────

    def add_check(self, sensor_id: str, check: SafetyCheck, *,
                  stale_after_s: Optional[float] = None) -> None:
        """Register a check tied to one SAFETY-role sensor.

        `stale_after_s` overrides the moderator-wide staleness limit for
        this sensor only — use it for slow-cadence sources (e.g. the cube
        heartbeat at 1.5s) that would otherwise always read as stale
        against the camera-rate default."""
        self._checks.setdefault(sensor_id, []).append(check)
        if stale_after_s is not None:
            self._stale_after_overrides[sensor_id] = max(0.01, stale_after_s)
        # A sensor that hasn't produced a frame yet is treated as stale —
        # the gate is closed until the first frame proves the sensor is
        # alive. Seed an entry so stale-detection sees it.
        self._last_frame_ts.setdefault(sensor_id, 0.0)
        self._check_results.setdefault(sensor_id, [])

    @property
    def has_checks(self) -> bool:
        """True when at least one safety check is registered."""
        return bool(self._checks)

    def set_output_callbacks(self,
                              enable: Optional[Callable[[], bool]],
                              disable: Optional[Callable[[], bool]]) -> None:
        """Wire in cube.enable_output / cube.disable_output (or test stubs).

        The moderator calls `enable` on SAFE-state entry and `disable` on
        any transition out of SAFE. Either can be None to skip the side
        effect (useful when running the moderator headless for logging)."""
        self._enable_output_cb = enable
        self._disable_output_cb = disable

    def set_voice_warning(self, voice: Optional["VoiceWarningProto"]) -> None:
        """Wire in a VoiceWarning instance. None disables voice output."""
        self._voice = voice

    def set_verdict_sink(self,
                         sink: Optional[Callable[[SafetyVerdict], None]]) -> None:
        """Register a callback that receives every new verdict."""
        self._on_verdict = sink

    # ── Operator controls ────────────────────────────────────────────────

    def arm(self) -> SafetyVerdict:
        """Operator says: I have visually confirmed the area is clear.
        The gate may now open IF all checks also agree."""
        with self._lock:
            self._armed = True
            return self._reevaluate_locked()

    def disarm(self) -> SafetyVerdict:
        """Operator says: shut it down. The gate stays closed until
        `arm()` is called again."""
        with self._lock:
            self._armed = False
            self._verdict = SafetyVerdict(
                ts=self._now(), safe=False, state=SafetyState.UNARMED,
                reasons=["disarmed by operator"])
        self._on_state_exit_safe()
        self._dispatch_verdict()
        return self._verdict

    # ── Frame intake ─────────────────────────────────────────────────────

    def on_frame(self, frame: SensorFrame) -> None:
        """Wire as the SensorManager frame sink.

        Frames from non-SAFETY sensors are ignored — only SAFETY-role
        frames feed the gate. Each registered check is run; the verdict
        is rebuilt and dispatched if it changed."""
        if frame.sensor_role != SensorRole.SAFETY.value:
            return
        if frame.sensor_id not in self._checks:
            # No checks registered for this sensor → it's a SAFETY sensor
            # being recorded for redundancy but not contributing to the
            # gate. Still update its liveness so stale-detection doesn't
            # close the gate on it.
            with self._lock:
                self._last_frame_ts[frame.sensor_id] = self._now()
            return

        results: list[tuple[bool, str]] = []
        for check in self._checks[frame.sensor_id]:
            try:
                results.append(check(frame))
            except Exception as exc:                       # pragma: no cover
                _log.exception("safety check raised on %s; treating as unsafe",
                               frame.sensor_id)
                results.append((False, f"check error: {exc!r}"))

        with self._lock:
            self._last_frame_ts[frame.sensor_id] = self._now()
            self._check_results[frame.sensor_id] = results
            self._reevaluate_locked()

    # ── External queries ─────────────────────────────────────────────────

    def is_safe_to_fire(self) -> bool:
        """Cheap query — uses the moderator's cached verdict.

        Callers MUST treat the result as a snapshot; the moderator may
        flip it on the next frame. A burst that fires multiple shots
        should poll between shots."""
        # Re-evaluate (mostly to apply stale-detection without needing a
        # new frame to trigger it).
        with self._lock:
            self._reevaluate_locked()
            return self._verdict.safe

    def verdict(self) -> SafetyVerdict:
        """Latest verdict, with stale-detection re-applied."""
        with self._lock:
            self._reevaluate_locked()
            return self._verdict

    # ── Internal ─────────────────────────────────────────────────────────

    def _reevaluate_locked(self) -> SafetyVerdict:
        """Recompute the verdict. MUST be called with `self._lock` held."""
        now = self._now()
        reasons: list[str] = []
        all_ok = True

        if self._require_arm and not self._armed:
            reasons.append("unarmed")
            all_ok = False

        # Zero registered checks keeps the gate closed unless the caller
        # explicitly opted out (laser-free contexts only).
        if not self._checks and not self._allow_no_checks:
            reasons.append("no safety checks registered")
            all_ok = False

        # Stale-sensor detection: any registered SAFETY sensor whose last
        # frame is older than its staleness limit closes the gate.
        for sid in self._checks:
            last = self._last_frame_ts.get(sid, 0.0)
            limit = self._stale_after_overrides.get(sid, self._stale_after_s)
            if last <= 0.0:
                reasons.append(f"{sid}: no frames yet")
                all_ok = False
            elif (now - last) > limit:
                age_ms = (now - last) * 1000.0
                reasons.append(f"{sid}: stale ({age_ms:.0f}ms)")
                all_ok = False

        # Per-check results.
        for sid, results in self._check_results.items():
            for ok, reason in results:
                if not ok:
                    all_ok = False
                if reason:
                    reasons.append(f"{sid}: {reason}")

        was_safe = self._verdict.safe
        was_state = self._verdict.state

        if all_ok:
            # Apply cooldown: if we just transitioned out of an unsafe
            # state, hold off the SAFE state until the cooldown clears.
            since_unsafe = now - self._latest_safe_event_ts
            if (self._safe_cooldown_s > 0
                    and was_state in (SafetyState.BLOCKED, SafetyState.UNARMED)
                    and since_unsafe < self._safe_cooldown_s):
                state = SafetyState.COOLDOWN
                safe = False
                reasons.append(f"cooldown {self._safe_cooldown_s:.1f}s")
            elif (was_state == SafetyState.COOLDOWN
                  and since_unsafe < self._safe_cooldown_s):
                state = SafetyState.COOLDOWN
                safe = False
                reasons.append(f"cooldown {self._safe_cooldown_s:.1f}s")
            else:
                state = SafetyState.SAFE
                safe = True
        else:
            self._latest_safe_event_ts = now
            state = (SafetyState.UNARMED if (self._require_arm
                                              and not self._armed)
                     else SafetyState.BLOCKED)
            safe = False

        verdict = SafetyVerdict(ts=now, safe=safe, state=state,
                                reasons=reasons)
        self._verdict = verdict

        # Side effects (output-gate + voice) happen outside the lock to
        # avoid holding it across slow I/O — but we need to capture the
        # transition while the lock is still held to decide what to do.
        transitioned_to_safe = (not was_safe) and safe
        transitioned_from_safe = was_safe and (not safe)

        if transitioned_to_safe:
            self._defer_call(self._on_state_enter_safe, verdict)
        elif transitioned_from_safe:
            self._defer_call(self._on_state_exit_safe_with_reasons, verdict)
        if verdict.state == SafetyState.BLOCKED and was_state != SafetyState.BLOCKED:
            self._defer_call(self._announce_block, verdict)
        if self._on_verdict is not None:
            self._defer_call(self._dispatch_verdict_for, verdict)

        return verdict

    # `_defer_call`: in production we want the side-effect to fire right
    # away — releasing the lock first is sufficient. The pattern keeps the
    # code symmetric in case we later want to queue these for a separate
    # thread (e.g. so a slow TTS doesn't stall the safety loop).
    def _defer_call(self, fn: Callable, *args) -> None:
        try:
            fn(*args)
        except Exception:                                   # pragma: no cover
            _log.exception("safety moderator side effect raised; suppressing")

    def _on_state_enter_safe(self, verdict: SafetyVerdict) -> None:
        if self._enable_output_cb is not None:
            self._enable_output_cb()

    def _on_state_exit_safe(self) -> None:
        if self._disable_output_cb is not None:
            self._disable_output_cb()

    def _on_state_exit_safe_with_reasons(self, verdict: SafetyVerdict) -> None:
        self._on_state_exit_safe()

    def _announce_block(self, verdict: SafetyVerdict) -> None:
        if self._voice is None:
            return
        # Prefer the most informative reason: a person-related reason
        # before a stale-sensor reason. The moderator's reason strings are
        # `<sensor>: <text>`; we look for "person" first.
        text = None
        for r in verdict.reasons:
            if "person" in r.lower():
                text = ("Person detected in beam path. "
                        "Please move out of the targeting area.")
                break
        if text is None:
            text = "Safety check failed. Hold position; targeting paused."
        self._voice.announce(text)

    def _dispatch_verdict(self) -> None:
        sink = self._on_verdict
        if sink is not None:
            try:
                sink(self._verdict)
            except Exception:                              # pragma: no cover
                _log.exception("verdict sink raised; suppressing")

    def _dispatch_verdict_for(self, verdict: SafetyVerdict) -> None:
        sink = self._on_verdict
        if sink is not None:
            try:
                sink(verdict)
            except Exception:                              # pragma: no cover
                _log.exception("verdict sink raised; suppressing")


# Avoid a hard import of VoiceWarning so this module can be used with a
# stub in tests. The protocol is just `announce(message: str, *,
# force: bool = False) -> bool`.
class VoiceWarningProto:                                   # pragma: no cover
    def announce(self, message: str, *, force: bool = False) -> bool: ...


# ── Helpers for common checks ────────────────────────────────────────────

def make_kinect_person_check(check) -> SafetyCheck:
    """Wrap a `KinectSafetyCheck.is_person_present` into a SafetyCheck.

    `check` is a `KinectSafetyCheck` instance. The returned closure
    expects a SensorFrame carrying a `depth` ndarray (the Kinect's depth
    plane in metres). If depth is missing, the result is unsafe-fail
    with reason `depth missing` — a safety sensor that has no data is a
    broken safety sensor.
    """
    def _check(frame: SensorFrame) -> tuple[bool, str]:
        depth = frame.depth
        if depth is None:
            return False, "depth missing"
        if isinstance(depth, np.ndarray) and depth.size == 0:
            return False, "depth empty"
        if check.is_person_present(depth):
            return False, "person detected in safety volume"
        return True, ""
    return _check


def make_cube_health_check(status_source) -> SafetyCheck:
    """Wrap a cube status source (anything with a `.last_info` returning a
    `LaserInfo` or None — in practice `laser.heartbeat.CubeHeartbeat`) into
    a SafetyCheck.

    The frame that triggers the check is just a liveness tick; the actual
    health data comes from `status_source.last_info`. Unsafe when the
    interlock is open or the cube reports over-temperature. No cube status
    yet is unsafe-fail — same doctrine as a safety sensor with no frames.
    Duck-typed so this module stays free of laser imports."""
    def _check(frame: SensorFrame) -> tuple[bool, str]:
        info = status_source.last_info
        if info is None:
            return False, "no cube status yet"
        if not info.interlock:
            return False, "cube interlock open"
        if info.over_temp:
            return False, "cube over-temperature"
        return True, ""
    return _check


def make_person_detector_check(detector) -> SafetyCheck:
    """Wrap a `PersonDetector` into a SafetyCheck.

    `detector` is any `PersonDetector`. Triggers unsafe when at least
    one detection is returned. Suitable for an RGB SAFETY sensor (e.g.
    the OV9281 used in a wide-FOV "scene watch" role); not appropriate
    for the targeting feed (which is narrow and already scrutinized).
    """
    def _check(frame: SensorFrame) -> tuple[bool, str]:
        rgb = frame.rgb
        if rgb is None:
            return False, "rgb missing"
        dets = detector.detect(rgb)
        if dets:
            return False, f"person detected ({len(dets)})"
        return True, ""
    return _check
