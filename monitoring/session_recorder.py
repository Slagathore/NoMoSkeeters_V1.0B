"""SessionRecorder — append-only JSONL log of session events.

Per BOOTSTRAP §14, every live session leaves a record on disk: when did
the operator arm, which tracks were detected, which were fired on, which
safety verdicts blocked which shots, and when did the cube get told to
emit. That record is the audit trail for any safety incident, and the
debugging input for any "why did it miss?" question after the fact.

The recorder is *append-only*: no fancy structure, no rotating buffers,
no remote shipping. Just write a JSONL file at
`SESSIONS_DIR/session_<utc>/events.jsonl` and a summary at
`session_meta.json`. Tools to read these are separate (a Jupyter notebook
opening pandas.read_json(..., lines=True) covers most needs).

Calling sites:
  - SafetyModerator → `record_verdict()`
  - LaserManager event sink → `record_event()`
  - SensorManager (optional) → `record_frame_meta()` (frames themselves
    are NOT recorded — too heavy; record the timestamp + shape only)

Reference: BOOTSTRAP.md §14, settings.SESSION_RECORDING_*.
"""
from __future__ import annotations

import json
import logging
import time
from dataclasses import asdict, is_dataclass
from datetime import datetime, timezone
from pathlib import Path
from threading import Lock
from typing import Any, Optional

from config import settings

_log = logging.getLogger(__name__)


def _serialise(obj: Any) -> Any:
    """Best-effort JSON-friendly serialisation. Falls back to repr() so a
    serialisation bug never breaks the recorder."""
    if obj is None or isinstance(obj, (bool, int, float, str)):
        return obj
    if isinstance(obj, (list, tuple)):
        return [_serialise(x) for x in obj]
    if isinstance(obj, dict):
        return {str(k): _serialise(v) for k, v in obj.items()}
    if is_dataclass(obj):
        return _serialise(asdict(obj))
    if hasattr(obj, "value"):       # Enum.value, GoProStatusEvent.flags, ...
        try:
            return _serialise(obj.value)
        except Exception:           # pragma: no cover
            pass
    return repr(obj)


class SessionRecorder:
    """Append-only JSONL session log.

    Open one per live-fire session. The recorder creates its own directory
    under `settings.SESSIONS_DIR`. Call `close()` to flush + finalise the
    meta file.
    """

    def __init__(self,
                 session_dir: Optional[Path] = None,
                 *,
                 tag: str = "session",
                 enabled: Optional[bool] = None):
        if enabled is None:
            enabled = settings.SESSION_RECORDING_ENABLED
        self._enabled = bool(enabled)
        self._lock = Lock()
        self._counts: dict[str, int] = {}
        self._start_ts = time.time()
        self._start_mono = time.monotonic()

        if not self._enabled:
            self.path: Optional[Path] = None
            self._events_fp = None
            return

        stamp = datetime.fromtimestamp(self._start_ts, tz=timezone.utc).strftime(
            "%Y%m%dT%H%M%SZ")
        base = (session_dir if session_dir is not None
                else settings.SESSIONS_DIR / f"{tag}_{stamp}")
        base.mkdir(parents=True, exist_ok=True)
        self.path = base
        try:
            self._events_fp = (base / "events.jsonl").open("a",
                                                              encoding="utf-8")
        except OSError:                                       # pragma: no cover
            _log.exception("session_recorder: failed to open events.jsonl")
            self._enabled = False
            self._events_fp = None
            return
        # Pre-write a marker so a session that crashes still has a meta record.
        self._raw_record({"op": "session_start", "tag": tag,
                          "ts_utc": stamp})

    # ── Specific record types (helpers around _raw_record) ───────────────

    def record_verdict(self, verdict) -> None:
        """Record a SafetyVerdict (or any dataclass / dict with the same
        fields)."""
        if not self._enabled:
            return
        self._raw_record({"op": "safety_verdict",
                          "data": _serialise(verdict)})

    def record_event(self, event: dict) -> None:
        """Generic event recorder (`LaserManager` event sink shape)."""
        if not self._enabled:
            return
        self._raw_record({"op": "laser_event", "data": _serialise(event)})

    def record_detection(self, det) -> None:
        if not self._enabled:
            return
        self._raw_record({"op": "detection", "data": _serialise(det)})

    def record_track(self, track) -> None:
        if not self._enabled:
            return
        self._raw_record({"op": "track", "data": _serialise(track)})

    def record_frame_meta(self, sensor_id: str, ts: float,
                          width: int, height: int) -> None:
        """Frame metadata only — the bytes themselves are not recorded
        (videos are too heavy to inline in JSONL)."""
        if not self._enabled:
            return
        self._raw_record({"op": "frame_meta", "sensor_id": sensor_id,
                          "ts": ts, "w": width, "h": height})

    def record_note(self, text: str) -> None:
        """Free-text annotation; useful when an operator types something."""
        if not self._enabled:
            return
        self._raw_record({"op": "note", "text": text})

    # ── Internal ─────────────────────────────────────────────────────────

    def _raw_record(self, record: dict) -> None:
        # Wrap with our timing fields so the consumer can sort regardless
        # of which call site produced it.
        wrapped = {
            "t_mono": time.monotonic() - self._start_mono,
            "t_wall": time.time(),
            **record,
        }
        line = json.dumps(wrapped, default=repr)
        with self._lock:
            self._counts[record.get("op", "?")] = self._counts.get(
                record.get("op", "?"), 0) + 1
            try:
                assert self._events_fp is not None
                self._events_fp.write(line + "\n")
                self._events_fp.flush()
            except (OSError, ValueError):                     # pragma: no cover
                _log.exception("session_recorder: write failed")

    def close(self) -> None:
        if not self._enabled or self.path is None:
            return
        meta = {
            "started_utc": datetime.fromtimestamp(
                self._start_ts, tz=timezone.utc).isoformat(),
            "ended_utc": datetime.now(tz=timezone.utc).isoformat(),
            "duration_s": time.monotonic() - self._start_mono,
            "event_counts": dict(self._counts),
        }
        try:
            (self.path / "session_meta.json").write_text(
                json.dumps(meta, indent=2), encoding="utf-8")
            if self._events_fp is not None:
                self._events_fp.flush()
                self._events_fp.close()
        except OSError:                                       # pragma: no cover
            _log.exception("session_recorder: close() failed")
        self._enabled = False
        self._events_fp = None
