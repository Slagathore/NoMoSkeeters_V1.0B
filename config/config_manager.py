"""
Iron Dome Anti-Mosquito System
================================
Mission: Automatically detect and neutralize mosquitos in real-time using a
         GoPro camera for vision and a LaserCube for precision targeting.

Config Manager Module
---------------------
Handles reading and writing of user-specific configuration overrides stored in
a JSON file on disk.  On first run the file is created with an empty dict;
subsequent runs merge the persisted overrides on top of the defaults from
settings.py so that unset keys always fall back to the default.

Modules imported:
  json, pathlib     — standard library I/O
  config.settings   — default constant values (all-caps names)

Classes:
  ConfigManager     — Singleton-style manager for runtime configuration state.

Functions (ConfigManager):
  load()            — Load JSON file, merge with defaults, return merged dict.
  save(overrides)   — Write only the override dict (not full defaults) to JSON.
  get(key, default) — Retrieve a single config value with optional fallback.
  set(key, value)   — Update a key in memory and persist immediately.
  all_settings()    — Return full dict (defaults + overrides merged).
  reset()           — Delete the JSON file and reload defaults.

Variables:
  _config_path      — pathlib.Path to the JSON config file.
  _overrides        — Dict of user-provided overrides loaded from disk.
  _defaults         — Dict of all default values pulled from settings module.

#todo Add config validation (jsonschema or manual) to catch invalid user edits.
#todo Support config profiles (e.g. "indoor", "outdoor") with quick switching.
#todo Watch config file for external edits and hot-reload.
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any

import config.settings as _settings

logger = logging.getLogger(__name__)


class ConfigManager:
    """
    Manages persistent user configuration overrides stored as JSON on disk.

    All values not explicitly overridden fall back to the constants defined in
    config/settings.py.  Only user overrides are written to disk so the file
    stays readable and compact.

    Attributes:
        _config_path (Path): Path to the JSON config file on disk.
        _overrides   (dict): Currently loaded user override values.
        _defaults    (dict): Default values extracted from settings.py module.
    """

    def __init__(self, config_path: Path | None = None) -> None:
        """
        Initialise the ConfigManager.

        Args:
            config_path: Optional explicit path to the JSON config file.
                         Defaults to CALIBRATION_FILE from settings.
        """
        self._config_path: Path = config_path or _settings.CONFIG_FILE
        self._overrides: dict[str, Any] = {}
        self._defaults: dict[str, Any] = self._extract_defaults()
        self.load()

    # ──────────────────────────────────────────────────────────────────────────
    # Private helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _extract_defaults(self) -> dict[str, Any]:
        """
        Extract all upper-case names from settings.py as the default dict.

        Returns:
            dict mapping setting name → default value for every constant
            defined in config/settings.py.
        """
        return {
            k: v
            for k, v in vars(_settings).items()
            if k.isupper() and not k.startswith("_")
        }

    # ──────────────────────────────────────────────────────────────────────────
    # Public API
    # ──────────────────────────────────────────────────────────────────────────

    def load(self) -> dict[str, Any]:
        """
        Load user overrides from the JSON config file on disk.

        Creates the parent directory and an empty JSON file if they do not
        exist yet.  Merges overrides on top of defaults.

        Returns:
            Merged configuration dict (defaults overridden by user values).
        """
        self._config_path.parent.mkdir(parents=True, exist_ok=True)
        if self._config_path.exists():
            try:
                with self._config_path.open("r", encoding="utf-8") as fh:
                    self._overrides = json.load(fh)
                logger.debug("Config loaded from %s", self._config_path)
            except (json.JSONDecodeError, OSError) as exc:
                logger.warning("Could not read config file (%s); using defaults.", exc)
                self._overrides = {}
        else:
            self._overrides = {}
            self._persist()
        return self.all_settings()

    def save(self, overrides: dict[str, Any]) -> None:
        """
        Replace the current overrides with the supplied dict and persist.

        Args:
            overrides: Dict of setting keys → override values.
        """
        self._overrides = overrides
        self._persist()

    def get(self, key: str, default: Any = None) -> Any:
        """
        Return the value for *key*, checking overrides first then defaults.

        Args:
            key:     Setting name (typically an upper-case constant name).
            default: Fallback if key is present in neither overrides nor defaults.

        Returns:
            The resolved value for the key.
        """
        if key in self._overrides:
            return self._overrides[key]
        return self._defaults.get(key, default)

    def set(self, key: str, value: Any) -> None:
        """
        Update a single key in the overrides dict and immediately persist.

        Args:
            key:   Setting name.
            value: New value to store.
        """
        self._overrides[key] = value
        self._persist()
        logger.debug("Config updated: %s = %r", key, value)

    def all_settings(self) -> dict[str, Any]:
        """
        Return the fully merged configuration (defaults + overrides).

        Returns:
            Dict with all settings, user overrides taking precedence.
        """
        merged = dict(self._defaults)
        merged.update(self._overrides)
        return merged

    def reset(self) -> None:
        """
        Delete the JSON config file and reload pure defaults.
        """
        if self._config_path.exists():
            self._config_path.unlink()
            logger.info("Config reset — deleted %s", self._config_path)
        self._overrides = {}

    # ──────────────────────────────────────────────────────────────────────────
    # Internal persistence
    # ──────────────────────────────────────────────────────────────────────────

    def _persist(self) -> None:
        """Write the current overrides dict (not full defaults) to JSON."""
        try:
            self._config_path.parent.mkdir(parents=True, exist_ok=True)
            with self._config_path.open("w", encoding="utf-8") as fh:
                # Convert Path objects to strings for JSON serialisation
                serialisable = {
                    k: (str(v) if isinstance(v, Path) else v)
                    for k, v in self._overrides.items()
                }
                json.dump(serialisable, fh, indent=2)
        except OSError as exc:
            logger.error("Failed to write config: %s", exc)
