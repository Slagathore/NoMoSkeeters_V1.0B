"""
Iron Dome Anti-Mosquito System
================================
Mission: Automatically detect and neutralize mosquitos in real-time using a
         GoPro camera for vision and a LaserCube for precision targeting.

Logging Utils Module
--------------------
Configures the Python logging subsystem for the entire application.  Should be
called once at application startup (before any other modules are imported that
use logging).

Sets up:
  - A rotating file handler (so logs do not grow indefinitely).
  - A coloured console handler for interactive use.
  - A root logger at the level specified in config/settings.py.

Modules imported:
  logging, logging.handlers  — standard library logging
  pathlib                    — standard library path handling
  config.settings            — LOG_LEVEL, LOG_FILE, LOG_MAX_BYTES, LOG_BACKUP_COUNT

Functions:
  setup_logging(level, log_file, max_bytes, backup_count)
      Initialise the root logger with file + console handlers.

  get_logger(name)
      Convenience wrapper around logging.getLogger(name).

Variables:
  _COLOUR_CODES  — ANSI escape code mapping from log level to terminal colour.

#todo Add a Qt-compatible log handler that forwards records to the GUI log panel.
#todo Support structured JSON logging for easier log aggregation (e.g. via ELK).
#todo Add optional Sentry/Datadog integration for remote error reporting.
"""

from __future__ import annotations

import logging
import logging.handlers
import sys
from pathlib import Path

import config.settings as _s

# ─── ANSI colour codes for console output ─────────────────────────────────────
_COLOUR_CODES: dict[int, str] = {
    logging.DEBUG:    "\033[36m",   # Cyan
    logging.INFO:     "\033[32m",   # Green
    logging.WARNING:  "\033[33m",   # Yellow
    logging.ERROR:    "\033[31m",   # Red
    logging.CRITICAL: "\033[35m",   # Magenta
}
_RESET: str = "\033[0m"

_LOG_FORMAT = "%(asctime)s [%(levelname)-8s] %(name)s — %(message)s"
_DATE_FORMAT = "%Y-%m-%d %H:%M:%S"


class _ColouredFormatter(logging.Formatter):
    """Logging formatter that adds ANSI colour codes to console output."""

    def format(self, record: logging.LogRecord) -> str:  # noqa: A003
        colour = _COLOUR_CODES.get(record.levelno, "")
        message = super().format(record)
        return f"{colour}{message}{_RESET}"


def setup_logging(
    level: str | None = None,
    log_file: Path | None = None,
    max_bytes: int | None = None,
    backup_count: int | None = None,
) -> None:
    """
    Initialise the application-wide logging configuration.

    Should be called ONCE at the very start of main.py before any other
    module creates loggers.

    Args:
        level:        Logging level string (e.g. "DEBUG", "INFO"). Defaults to
                      LOG_LEVEL from settings.
        log_file:     Path to the rotating log file.  Defaults to LOG_FILE.
        max_bytes:    Max file size in bytes before rotation.
        backup_count: Number of rotated backup files to keep.
    """
    level = level or _s.LOG_LEVEL
    log_file = log_file or _s.LOG_FILE
    max_bytes = max_bytes or _s.LOG_MAX_BYTES
    backup_count = backup_count if backup_count is not None else _s.LOG_BACKUP_COUNT

    numeric_level = getattr(logging, level.upper(), logging.INFO)

    root_logger = logging.getLogger()
    root_logger.setLevel(numeric_level)

    # Avoid adding duplicate handlers on repeated calls
    if root_logger.handlers:
        return

    # ── Console handler ────────────────────────────────────────────────────────
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(numeric_level)
    console_handler.setFormatter(
        _ColouredFormatter(fmt=_LOG_FORMAT, datefmt=_DATE_FORMAT)
    )
    root_logger.addHandler(console_handler)

    # ── File handler (rotating) ────────────────────────────────────────────────
    try:
        log_file.parent.mkdir(parents=True, exist_ok=True)
        file_handler = logging.handlers.RotatingFileHandler(
            filename=log_file,
            maxBytes=max_bytes,
            backupCount=backup_count,
            encoding="utf-8",
        )
        file_handler.setLevel(numeric_level)
        file_handler.setFormatter(
            logging.Formatter(fmt=_LOG_FORMAT, datefmt=_DATE_FORMAT)
        )
        root_logger.addHandler(file_handler)
    except OSError as exc:
        root_logger.warning("Could not create log file at %s: %s", log_file, exc)

    root_logger.info("Logging initialised at level=%s file=%s", level, log_file)


def get_logger(name: str) -> logging.Logger:
    """
    Return a named logger.

    Simple wrapper so other modules can do:
        from utils.logging_utils import get_logger
        logger = get_logger(__name__)

    Args:
        name: Logger name — conventionally use __name__.

    Returns:
        logging.Logger instance for *name*.
    """
    return logging.getLogger(name)
