"""Voice warnings for the safety moderator.

Uses Windows' built-in System.Speech via a short PowerShell subprocess so
there's zero Python dependency. SpeakAsync is used so warnings never block
the safety loop. Cooldown-throttled — the moderator can call announce()
repeatedly without producing a stuttering loop.

Linux/Mac fall back to a no-op + logger warning; for those platforms wire
in pyttsx3 or similar by subclassing.

Reference: BOOTSTRAP.md §12 (safety UX).
"""
from __future__ import annotations

import logging
import os
import subprocess
import sys
import threading
import time
from typing import Optional

_log = logging.getLogger(__name__)


class VoiceWarning:
    """Spoken safety warnings with a per-message cooldown.

    Cooldown is *per-message*: two different warnings (e.g. "person in
    beam path" vs "interlock open") can speak back to back, but the same
    message won't replay within `cooldown_s`. Set `cooldown_s=0` to disable.
    """

    def __init__(self, cooldown_s: float = 4.0,
                 enabled: bool = True,
                 voice_rate: int = 0):
        self._cooldown_s = max(0.0, cooldown_s)
        self._enabled = enabled and sys.platform == "win32"
        self._voice_rate = max(-10, min(10, voice_rate))
        self._last_said: dict[str, float] = {}
        self._lock = threading.Lock()

    @property
    def enabled(self) -> bool:
        return self._enabled

    def set_enabled(self, enabled: bool) -> None:
        self._enabled = enabled and sys.platform == "win32"

    def announce(self, message: str, *, force: bool = False) -> bool:
        """Speak `message`. Returns True if a speak was dispatched, False
        if suppressed (cooldown or disabled). `force=True` bypasses the
        cooldown for emergencies."""
        if not self._enabled or not message:
            return False
        now = time.monotonic()
        with self._lock:
            last = self._last_said.get(message, 0.0)
            if not force and (now - last) < self._cooldown_s:
                return False
            self._last_said[message] = now
        self._speak_async(message)
        return True

    def reset(self, message: Optional[str] = None) -> None:
        """Clear cooldown for one message (or all)."""
        with self._lock:
            if message is None:
                self._last_said.clear()
            else:
                self._last_said.pop(message, None)

    def _speak_async(self, message: str) -> None:
        # Escape single quotes for the PowerShell here-string. NOT shelling
        # out the message as an argv element so an attacker-controlled
        # message string (e.g. echoed back through some external surface)
        # can't break out of the cmdlet boundary.
        sanitized = message.replace("'", "''")
        rate = self._voice_rate
        script = (
            "Add-Type -AssemblyName System.Speech;"
            "$s = New-Object System.Speech.Synthesis.SpeechSynthesizer;"
            f"$s.Rate = {rate};"
            f"$s.SpeakAsync('{sanitized}') | Out-Null;"
            # Brief sleep so the async hand-off completes before the host
            # process exits — without this the .NET runtime tears the
            # synthesizer down before it pipes audio out.
            "Start-Sleep -Milliseconds 200;"
        )
        try:
            subprocess.Popen(
                ["powershell", "-NoLogo", "-NoProfile", "-NonInteractive",
                 "-Command", script],
                stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
            )
        except (OSError, FileNotFoundError):
            _log.exception("voice_warning: failed to launch powershell")
            self._enabled = False
