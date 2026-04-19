"""
Iron Dome Anti-Mosquito System
================================
GUI Package
-----------
PySide6-based graphical user interface for the Iron Dome system.

Modules:
  camera_view.py        — Video display widget with detection overlays.
  control_panel.py      — Side dock: connection controls, arm/disarm, stats.
  calibration_dialog.py — Step-by-step calibration wizard dialog.
  main_window.py        — Root QMainWindow assembling all GUI components.

Classes (re-exported):
  CameraView         — QWidget that renders camera frames with overlays.
  ControlPanel       — QWidget dock panel for system control.
  CalibrationDialog  — QDialog calibration wizard.
  MainWindow         — Root application window.

#todo Add a settings dialog (QDialog) for editing config values.
#todo Add a log viewer dock that streams the application log to a QTextEdit.
#todo Add a heatmap overlay showing accumulated detection density.
"""

from .calibration_dialog import CalibrationDialog
from .camera_view import CameraView
from .control_panel import ControlPanel
from .main_window import MainWindow

__all__ = ["CameraView", "ControlPanel", "CalibrationDialog", "MainWindow"]
