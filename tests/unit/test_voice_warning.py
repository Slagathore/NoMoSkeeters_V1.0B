"""VoiceWarning: cooldown + disabled behaviour. No actual speech in tests."""
from __future__ import annotations

import sys

from safety.voice_warning import VoiceWarning


def test_disabled_voice_never_dispatches(monkeypatch) -> None:
    v = VoiceWarning(enabled=False)
    assert v.enabled is False or sys.platform != "win32"
    called = {"n": 0}
    monkeypatch.setattr(v, "_speak_async",
                        lambda _msg: called.__setitem__("n", called["n"] + 1))
    assert not v.announce("ignored")
    assert called["n"] == 0


def test_enabled_voice_dispatches_once_then_cools_down(monkeypatch) -> None:
    v = VoiceWarning(cooldown_s=10.0, enabled=True)
    v._enabled = True            # bypass the win32-only auto-disable for the test
    calls: list[str] = []
    monkeypatch.setattr(v, "_speak_async", lambda msg: calls.append(msg))
    assert v.announce("hello")
    assert calls == ["hello"]
    # Second call within cooldown is suppressed.
    assert not v.announce("hello")
    assert calls == ["hello"]


def test_force_bypasses_cooldown(monkeypatch) -> None:
    v = VoiceWarning(cooldown_s=10.0, enabled=True)
    v._enabled = True
    calls: list[str] = []
    monkeypatch.setattr(v, "_speak_async", lambda msg: calls.append(msg))
    assert v.announce("x")
    assert v.announce("x", force=True)
    assert len(calls) == 2


def test_different_messages_each_have_their_own_cooldown(monkeypatch) -> None:
    v = VoiceWarning(cooldown_s=10.0, enabled=True)
    v._enabled = True
    calls: list[str] = []
    monkeypatch.setattr(v, "_speak_async", lambda msg: calls.append(msg))
    assert v.announce("a")
    assert v.announce("b")
    assert calls == ["a", "b"]


def test_reset_clears_cooldown(monkeypatch) -> None:
    v = VoiceWarning(cooldown_s=10.0, enabled=True)
    v._enabled = True
    calls: list[str] = []
    monkeypatch.setattr(v, "_speak_async", lambda msg: calls.append(msg))
    assert v.announce("x")
    assert not v.announce("x")
    v.reset("x")
    assert v.announce("x")
    assert calls == ["x", "x"]


def test_powershell_message_quote_is_escaped(monkeypatch) -> None:
    v = VoiceWarning(enabled=True)
    v._enabled = True
    captured: list[str] = []
    monkeypatch.setattr("subprocess.Popen",
                        lambda *a, **kw: captured.append(a[0]) or None)
    v.announce("don't fire")
    # The script should contain don''t (PowerShell-escaped), not a literal '.
    assert any("don''t fire" in arg for arg in captured[0])
