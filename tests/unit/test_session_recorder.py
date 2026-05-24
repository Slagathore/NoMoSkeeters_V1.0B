"""SessionRecorder: file shape, counters, dataclass serialisation."""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from monitoring.session_recorder import SessionRecorder


@dataclass
class FakeVerdict:
    safe: bool
    state: str
    reasons: list


def test_disabled_recorder_no_writes(tmp_path: Path) -> None:
    r = SessionRecorder(session_dir=tmp_path, enabled=False)
    r.record_event({"op": "x"})
    r.close()
    assert not any(tmp_path.iterdir())


def test_records_session_start_marker(tmp_path: Path) -> None:
    r = SessionRecorder(session_dir=tmp_path / "s", enabled=True)
    r.close()
    events = (tmp_path / "s" / "events.jsonl").read_text(encoding="utf-8")
    first = json.loads(events.splitlines()[0])
    assert first["op"] == "session_start"


def test_records_typed_events_with_dataclass(tmp_path: Path) -> None:
    r = SessionRecorder(session_dir=tmp_path / "s", enabled=True)
    v = FakeVerdict(safe=True, state="safe", reasons=["a", "b"])
    r.record_verdict(v)
    r.record_event({"op": "shoot", "track_id": 7})
    r.record_note("operator walked in")
    r.close()
    lines = (tmp_path / "s" / "events.jsonl").read_text(
        encoding="utf-8").splitlines()
    assert len(lines) >= 4   # start + verdict + event + note
    rec = next(json.loads(line) for line in lines
               if json.loads(line)["op"] == "safety_verdict")
    assert rec["data"]["safe"] is True
    assert rec["data"]["state"] == "safe"
    assert rec["data"]["reasons"] == ["a", "b"]


def test_meta_includes_event_counts(tmp_path: Path) -> None:
    r = SessionRecorder(session_dir=tmp_path / "s", enabled=True)
    r.record_event({"op": "x"})
    r.record_event({"op": "y"})
    r.close()
    meta = json.loads((tmp_path / "s" / "session_meta.json").read_text())
    assert meta["event_counts"]["session_start"] == 1
    assert meta["event_counts"]["laser_event"] == 2
    assert meta["duration_s"] >= 0.0


def test_unserialisable_objects_fall_back_to_repr(tmp_path: Path) -> None:
    class Weird:
        def __repr__(self):
            return "Weird()"
    r = SessionRecorder(session_dir=tmp_path / "s", enabled=True)
    r.record_event({"obj": Weird()})
    r.close()
    text = (tmp_path / "s" / "events.jsonl").read_text(encoding="utf-8")
    assert "Weird()" in text
