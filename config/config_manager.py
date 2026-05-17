"""Layered settings loader.

Resolution order, lowest to highest priority:
  1. Defaults from `config.settings` module
  2. JSON overrides from USER_CONFIG_PATH
  3. Environment variables prefixed with `NMS_<KEY>`
  4. In-process `set()` calls (runtime-mutable settings only)

Reference: BOOTSTRAP.md §13.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
from threading import RLock
from typing import Any, Optional, Union

from config import settings as _settings


_ENV_PREFIX = "NMS_"
_SENTINEL = object()


def _public_keys() -> list[str]:
    return [k for k in vars(_settings).keys() if k.isupper() and not k.startswith("_")]


def _coerce_to_default_type(default: Any, raw: str) -> Any:
    """Coerce a string (env var) to the type of the default value.
    Lists/dicts/Paths come through json or as Path; bool accepts truthy strings."""
    if isinstance(default, bool):
        return raw.strip().lower() in ("1", "true", "yes", "on")
    if isinstance(default, int) and not isinstance(default, bool):
        return int(raw)
    if isinstance(default, float):
        return float(raw)
    if isinstance(default, Path):
        return Path(raw)
    if isinstance(default, (list, dict, set)):
        return json.loads(raw)
    return raw  # str fallthrough


class ConfigManager:
    """Thread-safe layered settings access. Treat the result of get() as the
    effective value at this moment; callers should not cache it."""

    def __init__(self,
                 user_config_path: Optional[Union[str, Path]] = None,
                 read_env: bool = True):
        self._lock = RLock()
        self._user_path = Path(user_config_path) if user_config_path else _settings.USER_CONFIG_PATH
        self._read_env = read_env
        self._json_overrides: dict[str, Any] = {}
        self._runtime_overrides: dict[str, Any] = {}
        self._reload_json()

    # ── Public API ───────────────────────────────────────────────────────

    def get(self, key: str, default: Any = _SENTINEL) -> Any:
        with self._lock:
            if key in self._runtime_overrides:
                return self._runtime_overrides[key]
            if self._read_env:
                env_val = os.environ.get(_ENV_PREFIX + key)
                if env_val is not None:
                    base = getattr(_settings, key, None)
                    try:
                        return _coerce_to_default_type(base, env_val) if base is not None else env_val
                    except (ValueError, json.JSONDecodeError):
                        return env_val
            if key in self._json_overrides:
                return self._json_overrides[key]
            if hasattr(_settings, key):
                return getattr(_settings, key)
            if default is _SENTINEL:
                raise KeyError(f"unknown setting: {key}")
            return default

    def set(self, key: str, value: Any) -> None:
        """Set a runtime override. Refuses for keys in RESTART_REQUIRED."""
        with self._lock:
            if key in _settings.RESTART_REQUIRED:
                raise ValueError(
                    f"{key} requires a restart to change; edit user config or env var")
            if not hasattr(_settings, key):
                raise KeyError(f"unknown setting: {key}")
            self._runtime_overrides[key] = value

    def is_runtime_mutable(self, key: str) -> bool:
        return hasattr(_settings, key) and key not in _settings.RESTART_REQUIRED

    def reload(self) -> None:
        """Re-read user JSON. Runtime overrides survive."""
        with self._lock:
            self._reload_json()

    def save_user_config(self, overrides: dict[str, Any]) -> None:
        """Write a JSON override file at USER_CONFIG_PATH. Pass an empty dict
        to clear all JSON overrides."""
        with self._lock:
            self._user_path.parent.mkdir(parents=True, exist_ok=True)
            serialized = {k: _to_jsonable(v) for k, v in overrides.items()}
            self._user_path.write_text(json.dumps(serialized, indent=2))
            self._json_overrides = dict(overrides)

    def all(self) -> dict[str, Any]:
        """Snapshot of effective values for every public setting."""
        return {k: self.get(k) for k in _public_keys()}

    # ── Internal ─────────────────────────────────────────────────────────

    def _reload_json(self) -> None:
        if not self._user_path.exists():
            self._json_overrides = {}
            return
        try:
            raw = json.loads(self._user_path.read_text())
        except (OSError, json.JSONDecodeError):
            self._json_overrides = {}
            return
        if not isinstance(raw, dict):
            self._json_overrides = {}
            return
        # Only keep keys that exist in settings.py — silent ignore for unknown.
        self._json_overrides = {k: v for k, v in raw.items() if hasattr(_settings, k)}


def _to_jsonable(v: Any) -> Any:
    if isinstance(v, Path):
        return str(v)
    if isinstance(v, set):
        return sorted(v)
    return v
