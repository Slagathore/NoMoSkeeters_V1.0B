"""
Iron Dome Anti-Mosquito System
================================
Mission: Automatically detect and neutralize mosquitos in real-time using a
         GoPro camera for vision and a LaserCube for precision targeting.

Control Panel Module
--------------------
Provides ControlPanel: a QWidget housing all system controls and status
indicators.  Designed to be used as a floating dock widget in MainWindow.

Sections:
  1. Connection — Connect/Disconnect buttons for GoPro and LaserCube,
                  with coloured status indicators (red/yellow/green).
  2. Safety     — ARM / DISARM toggle and EMERGENCY STOP button.
  3. Detection  — Mode selector (BGSub / YOLO), sensitivity slider,
                  reset background model button.
  4. Stats      — Live counters: camera FPS, detection FPS, active tracks,
                  total shots fired.
  5. No-Fire Zones — Add/remove zone entries, visible list.

Modules imported:
  PySide6.QtWidgets — QWidget, QVBoxLayout, QGroupBox, QPushButton,
                       QLabel, QSlider, QComboBox, QListWidget
  PySide6.QtCore    — Qt, Signal
  PySide6.QtGui     — QColor, QPalette
  utils.safety      — SafetySystem
  detection.detector — DetectionMode
  config.settings   — GUI colour constants
  utils.logging_utils — get_logger

Classes:
  StatusLight     — Small QLabel circle showing red/yellow/green.
  ControlPanel    — Main control dock widget.

Signals (ControlPanel):
  gopro_connect_requested    ()          — User clicked Connect GoPro.
  gopro_disconnect_requested ()          — User clicked Disconnect GoPro.
  laser_connect_requested    ()          — User clicked Connect Laser.
  laser_disconnect_requested ()          — User clicked Disconnect Laser.
  arm_requested              ()          — User clicked ARM.
  disarm_requested           ()          — User clicked DISARM.
  estop_requested            ()          — User clicked EMERGENCY STOP.
  detection_mode_changed (str)           — User selected a detection mode.
  sensitivity_changed    (int)           — User moved the sensitivity slider.
  reset_background_requested ()          — User clicked Reset Background.
  calibrate_requested        ()          — User clicked Calibrate.
  add_zone_requested         ()          — User clicked Add No-Fire Zone.
  remove_zone_requested  (int)           — User clicked Remove Zone (zone_id).

Methods (ControlPanel):
  set_gopro_status(connected)            — Update GoPro indicator light.
  set_laser_status(connected)            — Update laser indicator light.
  set_armed_state(armed, estop)          — Update ARM/DISARM button appearance.
  update_stats(cam_fps, det_fps, tracks, shots) — Refresh stat labels.
  refresh_zones(zones)                   — Rebuild the zone list widget.

Variables (instance):
  _safety              (SafetySystem | None): For reading state directly.
  _status_gopro        (StatusLight): GoPro connection indicator.
  _status_laser        (StatusLight): Laser connection indicator.
  _btn_arm             (QPushButton): ARM button.
  _btn_disarm          (QPushButton): DISARM button.
  _btn_estop           (QPushButton): EMERGENCY STOP button.
  _combo_mode          (QComboBox): Detection mode selector.
  _slider_sensitivity  (QSlider): Sensitivity adjustment.
  _lbl_cam_fps, _lbl_det_fps, _lbl_tracks, _lbl_shots (QLabel): Stat labels.
  _list_zones          (QListWidget): No-fire zone list.

#todo Add a laser power slider (per-channel RGB).
#todo Add a "test fire" button that sends a single calibration dot.
#todo Show a mini system log (last 10 lines) directly in the control panel.
#todo Allow drag-to-reorder no-fire zones by priority.
"""

from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QColor, QPalette
from PySide6.QtWidgets import (QComboBox, QGroupBox, QHBoxLayout, QLabel,
                               QListWidget, QPushButton, QScrollArea,
                               QSizePolicy, QSlider, QVBoxLayout, QWidget)

import config.settings as _s
from utils.logging_utils import get_logger

logger = get_logger(__name__)


# ─── StatusLight ──────────────────────────────────────────────────────────────

class StatusLight(QLabel):
    """
    A small circular status indicator label.

    Colours:
      green  — connected / armed / OK
      yellow — connecting / transitioning
      red    — disconnected / error

    Attributes:
        _colour (str): Current CSS colour name.
    """

    def __init__(self, label: str = "", parent=None) -> None:
        super().__init__(parent)
        self._label_text = label
        self._colour = "red"
        self._update_style()

    def set_state(self, state: str) -> None:
        """
        Update colour/state.

        Args:
            state: "green", "yellow", or "red".
        """
        self._colour = state
        self._update_style()

    def _update_style(self) -> None:
        colour_map = {
            "green":  "#22cc44",
            "yellow": "#ffcc00",
            "red":    "#cc2222",
        }
        hex_color = colour_map.get(self._colour, "#888")
        self.setStyleSheet(
            f"QLabel {{ "
            f"  background-color: {hex_color}; "
            f"  border-radius: 6px; "
            f"  min-width: 12px; max-width: 12px; "
            f"  min-height: 12px; max-height: 12px; "
            f"}}"
        )
        self.setToolTip(f"{self._label_text}: {self._colour}")


# ─── ControlPanel ─────────────────────────────────────────────────────────────

class ControlPanel(QWidget):
    """
    Dock panel containing all system controls and status indicators.

    Signals:
        gopro_connect_requested    (): User wants to connect GoPro.
        gopro_disconnect_requested (): User wants to disconnect GoPro.
        laser_connect_requested    (): User wants to connect laser.
        laser_disconnect_requested (): User wants to disconnect laser.
        arm_requested              (): User pressed ARM.
        disarm_requested           (): User pressed DISARM.
        estop_requested            (): User pressed EMERGENCY STOP.
        detection_mode_changed (str): New detection mode name.
        sensitivity_changed    (int): New sensitivity value 0–100.
        reset_background_requested (): Reset MOG2 background model.
        calibrate_requested        (): Open calibration wizard.
        add_zone_requested         (): Start add-zone flow.
        remove_zone_requested  (int): Remove zone with given ID.
    """

    gopro_connect_requested = Signal()
    gopro_disconnect_requested = Signal()
    laser_connect_requested = Signal()
    laser_disconnect_requested = Signal()
    arm_requested = Signal()
    disarm_requested = Signal()
    estop_requested = Signal()
    detection_mode_changed = Signal(str)
    sensitivity_changed = Signal(int)
    reset_background_requested = Signal()
    calibrate_requested = Signal()
    add_zone_requested = Signal()
    remove_zone_requested = Signal(int)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._safety = None
        self._zones_data: list = []  # cached NoFireZone list
        self._armed: bool = False
        self._estop: bool = False
        self._zone_ids: list[int] = []  # parallel list of zone IDs in list widget

        self.setMinimumWidth(240)
        self.setMaximumWidth(320)
        self.setStyleSheet("""
            QWidget { background-color: #1e1e2e; color: #cdd6f4; font-family: Consolas; }
            QGroupBox { border: 1px solid #45475a; border-radius: 4px; margin-top: 8px;
                        padding-top: 4px; font-weight: bold; color: #cba6f7; }
            QGroupBox::title { subcontrol-origin: margin; left: 8px; }
            QPushButton { border-radius: 4px; padding: 5px 10px; font-weight: bold; }
            QPushButton:disabled { opacity: 0.4; }
        """)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        layout.addWidget(self._build_connection_group())
        layout.addWidget(self._build_safety_group())
        layout.addWidget(self._build_detection_group())
        layout.addWidget(self._build_stats_group())
        layout.addWidget(self._build_zones_group())
        layout.addStretch()

    # ──────────────────────────────────────────────────────────────────────────
    # Group builders
    # ──────────────────────────────────────────────────────────────────────────

    def _build_connection_group(self) -> QGroupBox:
        grp = QGroupBox("Connections")
        lay = QVBoxLayout(grp)
        lay.setSpacing(4)

        # GoPro row
        gopro_row = QHBoxLayout()
        self._status_gopro = StatusLight("GoPro")
        self._btn_gopro_connect = QPushButton("Connect GoPro")
        self._btn_gopro_connect.setStyleSheet(
            "QPushButton { background-color: #313244; color: #a6e3a1; border: 1px solid #a6e3a1; }"
            "QPushButton:hover { background-color: #45475a; }"
        )
        self._btn_gopro_disconnect = QPushButton("Disconnect")
        self._btn_gopro_disconnect.setEnabled(False)
        self._btn_gopro_disconnect.setStyleSheet(
            "QPushButton { background-color: #313244; color: #f38ba8; border: 1px solid #f38ba8; }"
        )
        gopro_row.addWidget(self._status_gopro)
        gopro_row.addWidget(self._btn_gopro_connect)
        gopro_row.addWidget(self._btn_gopro_disconnect)
        lay.addLayout(gopro_row)

        # Laser row
        laser_row = QHBoxLayout()
        self._status_laser = StatusLight("Laser")
        self._btn_laser_connect = QPushButton("Connect Laser")
        self._btn_laser_connect.setStyleSheet(
            "QPushButton { background-color: #313244; color: #a6e3a1; border: 1px solid #a6e3a1; }"
            "QPushButton:hover { background-color: #45475a; }"
        )
        self._btn_laser_disconnect = QPushButton("Disconnect")
        self._btn_laser_disconnect.setEnabled(False)
        self._btn_laser_disconnect.setStyleSheet(
            "QPushButton { background-color: #313244; color: #f38ba8; border: 1px solid #f38ba8; }"
        )
        laser_row.addWidget(self._status_laser)
        laser_row.addWidget(self._btn_laser_connect)
        laser_row.addWidget(self._btn_laser_disconnect)
        lay.addLayout(laser_row)

        # Calibrate button
        self._btn_calibrate = QPushButton("⚙ Calibrate Laser ↔ Camera")
        self._btn_calibrate.setStyleSheet(
            "QPushButton { background-color: #313244; color: #89dceb; border: 1px solid #89dceb; }"
            "QPushButton:hover { background-color: #45475a; }"
        )
        lay.addWidget(self._btn_calibrate)

        # Signals
        self._btn_gopro_connect.clicked.connect(self.gopro_connect_requested)
        self._btn_gopro_disconnect.clicked.connect(self.gopro_disconnect_requested)
        self._btn_laser_connect.clicked.connect(self.laser_connect_requested)
        self._btn_laser_disconnect.clicked.connect(self.laser_disconnect_requested)
        self._btn_calibrate.clicked.connect(self.calibrate_requested)

        return grp

    def _build_safety_group(self) -> QGroupBox:
        grp = QGroupBox("⚡ Safety")
        lay = QVBoxLayout(grp)
        lay.setSpacing(4)

        arm_row = QHBoxLayout()
        self._btn_arm = QPushButton("🔓 ARM")
        self._btn_arm.setStyleSheet(
            "QPushButton { background-color: #1e1e2e; color: #a6e3a1; "
            "border: 2px solid #a6e3a1; font-size: 13px; padding: 8px; }"
            "QPushButton:hover { background-color: #313244; }"
        )
        self._btn_disarm = QPushButton("🔒 DISARM")
        self._btn_disarm.setEnabled(False)
        self._btn_disarm.setStyleSheet(
            "QPushButton { background-color: #1e1e2e; color: #f9e2af; "
            "border: 2px solid #f9e2af; font-size: 13px; padding: 8px; }"
        )
        arm_row.addWidget(self._btn_arm)
        arm_row.addWidget(self._btn_disarm)
        lay.addLayout(arm_row)

        self._btn_estop = QPushButton("🚨 EMERGENCY STOP")
        self._btn_estop.setStyleSheet(
            "QPushButton { background-color: #f38ba8; color: #1e1e2e; "
            "border: none; font-size: 14px; font-weight: bold; padding: 10px; "
            "border-radius: 6px; }"
            "QPushButton:hover { background-color: #ff5555; }"
            "QPushButton:pressed { background-color: #cc0000; }"
        )

        self._lbl_safety_status = QLabel("Status: DISARMED")
        self._lbl_safety_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._lbl_safety_status.setStyleSheet("color: #f9e2af; font-weight: bold;")

        lay.addWidget(self._btn_estop)
        lay.addWidget(self._lbl_safety_status)

        self._btn_arm.clicked.connect(self.arm_requested)
        self._btn_disarm.clicked.connect(self.disarm_requested)
        self._btn_estop.clicked.connect(self.estop_requested)

        return grp

    def _build_detection_group(self) -> QGroupBox:
        grp = QGroupBox("Detection")
        lay = QVBoxLayout(grp)
        lay.setSpacing(4)

        mode_row = QHBoxLayout()
        mode_row.addWidget(QLabel("Mode:"))
        self._combo_mode = QComboBox()
        self._combo_mode.addItem("Background Subtraction", "bgsub")
        self._combo_mode.addItem("YOLO (AI)", "yolo")
        self._combo_mode.setStyleSheet("QComboBox { background-color: #313244; color: #cdd6f4; }")
        mode_row.addWidget(self._combo_mode)
        lay.addLayout(mode_row)

        sens_row = QHBoxLayout()
        sens_row.addWidget(QLabel("Sensitivity:"))
        self._slider_sensitivity = QSlider(Qt.Orientation.Horizontal)
        self._slider_sensitivity.setRange(0, 100)
        self._slider_sensitivity.setValue(50)
        self._slider_sensitivity.setTickPosition(QSlider.TickPosition.TicksBelow)
        self._slider_sensitivity.setTickInterval(25)
        self._lbl_sensitivity_val = QLabel("50")
        self._lbl_sensitivity_val.setMinimumWidth(28)
        sens_row.addWidget(self._slider_sensitivity)
        sens_row.addWidget(self._lbl_sensitivity_val)
        lay.addLayout(sens_row)

        self._btn_reset_bg = QPushButton("↺ Reset Background Model")
        self._btn_reset_bg.setStyleSheet(
            "QPushButton { background-color: #313244; color: #cba6f7; "
            "border: 1px solid #cba6f7; }"
            "QPushButton:hover { background-color: #45475a; }"
        )
        lay.addWidget(self._btn_reset_bg)

        self._combo_mode.currentIndexChanged.connect(
            lambda i: self.detection_mode_changed.emit(self._combo_mode.currentData())
        )
        self._slider_sensitivity.valueChanged.connect(self._on_sensitivity_changed)
        self._btn_reset_bg.clicked.connect(self.reset_background_requested)

        return grp

    def _build_stats_group(self) -> QGroupBox:
        grp = QGroupBox("Stats")
        lay = QVBoxLayout(grp)
        lay.setSpacing(2)

        def make_stat_row(label: str) -> QLabel:
            row = QHBoxLayout()
            row.addWidget(QLabel(label))
            val_lbl = QLabel("—")
            val_lbl.setAlignment(Qt.AlignmentFlag.AlignRight)
            val_lbl.setStyleSheet("color: #89dceb; font-weight: bold;")
            row.addWidget(val_lbl)
            lay.addLayout(row)
            return val_lbl

        self._lbl_cam_fps = make_stat_row("Camera FPS:")
        self._lbl_det_fps = make_stat_row("Detection FPS:")
        self._lbl_tracks = make_stat_row("Active Tracks:")
        self._lbl_shots = make_stat_row("Shots Fired:")

        return grp

    def _build_zones_group(self) -> QGroupBox:
        grp = QGroupBox("No-Fire Zones")
        lay = QVBoxLayout(grp)
        lay.setSpacing(4)

        self._list_zones = QListWidget()
        self._list_zones.setMaximumHeight(90)
        self._list_zones.setStyleSheet(
            "QListWidget { background-color: #181825; color: #f38ba8; "
            "border: 1px solid #45475a; }"
        )
        lay.addWidget(self._list_zones)

        zone_btn_row = QHBoxLayout()
        self._btn_add_zone = QPushButton("+ Add Zone")
        self._btn_add_zone.setStyleSheet(
            "QPushButton { background-color: #313244; color: #fab387; "
            "border: 1px solid #fab387; }"
        )
        self._btn_remove_zone = QPushButton("✕ Remove")
        self._btn_remove_zone.setStyleSheet(
            "QPushButton { background-color: #313244; color: #f38ba8; "
            "border: 1px solid #f38ba8; }"
        )
        zone_btn_row.addWidget(self._btn_add_zone)
        zone_btn_row.addWidget(self._btn_remove_zone)
        lay.addLayout(zone_btn_row)

        self._btn_add_zone.clicked.connect(self.add_zone_requested)
        self._btn_remove_zone.clicked.connect(self._on_remove_zone_clicked)

        return grp

    # ──────────────────────────────────────────────────────────────────────────
    # Public state update methods
    # ──────────────────────────────────────────────────────────────────────────

    def set_gopro_status(self, connected: bool) -> None:
        """
        Update the GoPro connection indicator.

        Args:
            connected: True if GoPro is connected and streaming.
        """
        self._status_gopro.set_state("green" if connected else "red")
        self._btn_gopro_connect.setEnabled(not connected)
        self._btn_gopro_disconnect.setEnabled(connected)

    def set_laser_status(self, connected: bool) -> None:
        """
        Update the LaserCube connection indicator.

        Args:
            connected: True if LaserCube is connected.
        """
        self._status_laser.set_state("green" if connected else "red")
        self._btn_laser_connect.setEnabled(not connected)
        self._btn_laser_disconnect.setEnabled(connected)
        self._btn_arm.setEnabled(connected)

    def set_armed_state(self, armed: bool, estop: bool) -> None:
        """
        Update the ARM/DISARM buttons and status label.

        Args:
            armed: True if system is armed.
            estop: True if emergency stop is active.
        """
        self._armed = armed
        self._estop = estop

        if estop:
            status_text = "Status: ⛔ E-STOP ACTIVE"
            status_colour = "#f38ba8"
        elif armed:
            status_text = "Status: ⚡ ARMED — LASER ACTIVE"
            status_colour = "#a6e3a1"
        else:
            status_text = "Status: 🔒 DISARMED"
            status_colour = "#f9e2af"

        self._lbl_safety_status.setText(status_text)
        self._lbl_safety_status.setStyleSheet(
            f"color: {status_colour}; font-weight: bold;"
        )
        self._btn_arm.setEnabled(not armed and not estop)
        self._btn_disarm.setEnabled(armed)

    def update_stats(
        self,
        cam_fps: float,
        det_fps: float,
        tracks: int,
        shots: int,
    ) -> None:
        """
        Refresh the statistics labels.

        Args:
            cam_fps: Current camera capture FPS.
            det_fps: Current detection processing FPS.
            tracks:  Number of active tracked targets.
            shots:   Total number of laser shots fired.
        """
        self._lbl_cam_fps.setText(f"{cam_fps:.1f}")
        self._lbl_det_fps.setText(f"{det_fps:.1f}")
        self._lbl_tracks.setText(str(tracks))
        self._lbl_shots.setText(str(shots))

    def refresh_zones(self, zones: list) -> None:
        """
        Rebuild the no-fire zone list widget.

        Args:
            zones: List of NoFireZone objects from SafetySystem.
        """
        self._zones_data = zones
        self._zone_ids = []
        self._list_zones.clear()
        for z in zones:
            self._list_zones.addItem(
                f"#{z.zone_id} {z.label} ({z.x1},{z.y1})–({z.x2},{z.y2})"
            )
            self._zone_ids.append(z.zone_id)

    # ──────────────────────────────────────────────────────────────────────────
    # Private slots
    # ──────────────────────────────────────────────────────────────────────────

    def _on_sensitivity_changed(self, value: int) -> None:
        self._lbl_sensitivity_val.setText(str(value))
        self.sensitivity_changed.emit(value)

    def _on_remove_zone_clicked(self) -> None:
        row = self._list_zones.currentRow()
        if row < 0 or row >= len(self._zone_ids):
            return
        self.remove_zone_requested.emit(self._zone_ids[row])
