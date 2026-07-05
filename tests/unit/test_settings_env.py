"""NMS_<NAME> environment overrides in config/settings.py.

These tests reload the settings module, so they restore it with a clean
reload afterwards — other modules hold `from config import settings`
references to the same module object, which survives reload.
"""
from __future__ import annotations

import importlib
from pathlib import Path

import pytest

from config import settings


@pytest.fixture()
def reload_settings(monkeypatch: pytest.MonkeyPatch):
    """Yields a reloader; restores pristine settings afterwards."""
    def _reload():
        importlib.reload(settings)
    yield _reload
    monkeypatch.undo()          # drop env vars BEFORE the restore reload
    importlib.reload(settings)


def test_env_overrides_by_type(monkeypatch, reload_settings) -> None:
    monkeypatch.setenv("NMS_SAFETY_DWELL_LIMIT_MS", "120")
    monkeypatch.setenv("NMS_SAFETY_DWELL_LIMIT_ENABLED", "false")
    monkeypatch.setenv("NMS_SAFETY_MODERATOR_STALE_AFTER_S", "0.75")
    monkeypatch.setenv("NMS_LOG_LEVEL", "DEBUG")
    monkeypatch.setenv("NMS_SESSIONS_DIR", r"C:\tmp\nms_sessions")
    reload_settings()
    assert settings.SAFETY_DWELL_LIMIT_MS == 120
    assert settings.SAFETY_DWELL_LIMIT_ENABLED is False
    assert settings.SAFETY_MODERATOR_STALE_AFTER_S == 0.75
    assert settings.LOG_LEVEL == "DEBUG"
    assert settings.SESSIONS_DIR == Path(r"C:\tmp\nms_sessions")


def test_unset_env_leaves_defaults(reload_settings) -> None:
    before = settings.SAFETY_DWELL_LIMIT_MS
    reload_settings()
    assert settings.SAFETY_DWELL_LIMIT_MS == before


def test_bad_value_fails_loudly(monkeypatch, reload_settings) -> None:
    monkeypatch.setenv("NMS_SAFETY_DWELL_LIMIT_MS", "eighty")
    with pytest.raises(SystemExit):
        reload_settings()
