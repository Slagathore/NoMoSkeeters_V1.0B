"""
Iron Dome Anti-Mosquito System
================================
Mission: Automatically detect and neutralize mosquitos in real-time by fusing live
         video from a GoPro action camera with precision galvo targeting via a
         LaserCube ILDA laser projector.  The system runs a continuous pipeline of
         background-subtraction (or YOLO) detection, Kalman-filter multi-object
         tracking, homography-based coordinate mapping, and safety-gated laser firing,
         all presented through a single PySide6 GUI with real-time video overlay.

Goal: Provide a fully autonomous, operator-supervised "iron dome" against mosquitos
      that is configurable, safe-by-default, and operable by a non-technical user.

Entry Point Module (main.py)
-----------------------------
This is the single entry point for the application.  It is intentionally minimal.
Responsibilities:
  1. Parse optional command-line arguments.
  2. Initialise application-wide logging (before any other module runs).
  3. Instantiate QApplication and ConfigManager.
  4. Load any persisted user configuration from disk.
  5. Create and show MainWindow.
  6. Enter the Qt event loop and forward the exit code to the OS.

No business logic lives here — everything is delegated to the GUI and subsystem
modules below it in the dependency tree.

Modules imported:
  sys                      — exit-code forwarding.
  argparse                 — optional CLI flags.
  PySide6.QtWidgets        — QApplication.
  PySide6.QtCore           — Qt (high-DPI policy constant).
  utils.logging_utils      — setup_logging().
  config.config_manager    — ConfigManager (persistent user settings).
  gui.main_window          — MainWindow (top-level application window).

Classes: (none — procedural entry point)

Functions:
  main() — Parses CLI args, wires everything up, and runs the Qt event loop.

Variables:
  None at module scope beyond the standard ``if __name__ == "__main__"`` guard.

#todo Add --debug flag that sets log level to DEBUG and maximises all log verbosity.
#todo Add --no-laser flag for demo/testing mode (detector + tracker run, laser is mocked).
#todo Add --record <path> flag to automatically start a session recording on launch.
#todo Add --config <path> flag to specify a custom config file location.
#todo Show a splash screen while modules initialize on slower machines.
#todo Support --fullscreen flag for kiosk-style deployments.
"""

from __future__ import annotations

import argparse
import sys

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QApplication

from config.config_manager import ConfigManager
from gui.main_window import MainWindow
from utils.logging_utils import setup_logging


def main() -> int:
    """
    Application entry point.

    Parses command-line flags, initialises logging, creates the Qt application,
    and runs the event loop.

    Returns:
        int: Exit code forwarded from QApplication.exec().
    """
    # ── CLI ───────────────────────────────────────────────────────────────────
    parser = argparse.ArgumentParser(
        prog="iron_dome",
        description=(
            "Iron Dome Anti-Mosquito System — "
            "real-time laser targeting via GoPro + LaserCube."
        ),
    )
    parser.add_argument(
        "--log-level",
        default=None,
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
        help="Override the logging level from config (default: INFO).",
    )
    parser.add_argument(
        "--config",
        default=None,
        metavar="PATH",
        help="Path to a custom JSON configuration file.",
    )
    args = parser.parse_args()

    # ── Logging ───────────────────────────────────────────────────────────────
    # Must be first — all other modules call get_logger() at import time.
    setup_logging(level=args.log_level)

    # ── Config ────────────────────────────────────────────────────────────────
    from pathlib import Path  # local import to keep top-level imports fast

    config_path = Path(args.config) if args.config else None
    config = ConfigManager(config_path=config_path)

    # ── Qt application ────────────────────────────────────────────────────────
    # Enable high-DPI scaling before creating QApplication.
    QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough
    )

    app = QApplication(sys.argv)
    app.setApplicationName("Iron Dome Anti-Mosquito System")
    app.setOrganizationName("IronDome")
    app.setApplicationVersion("1.0.0")

    # ── Main window ────────────────────────────────────────────────────────────
    window = MainWindow(config)
    window.show()

    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
