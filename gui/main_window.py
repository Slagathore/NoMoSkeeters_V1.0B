"""
Iron Dome Anti-Mosquito System
================================
Mission: Automatically detect and neutralize mosquitos in real-time using a
         GoPro camera for vision and a LaserCube for precision targeting.

Main Window Module
------------------
Provides MainWindow: the top-level QMainWindow that assembles and manages
all subsystems.  It owns the lifecycle of every QThread worker, wires all
Qt signals to their slots, and orchestrates the full detection-tracking-
targeting pipeline.

Layout:
  Central widget — CameraView (live video + overlays).
  Right dock     — ControlPanel (connections, safety, detection, stats).
  Menu bar       — File, View, Tools, Help.
  Status bar     — Camera FPS | Detections/tracks | Laser state.

Subsystem ownership:
  _safety     SafetySystem              — arm/disarm/e-stop gatekeeper.
  _mapper     CoordinateMapper          — camera pixel → laser coordinate.
  _tracker    MosquitoTracker           — Kalman multi-object tracker.
  _cal_ctrl   CalibrationController     — calibration wizard driver.
  _interface  LaserCubeInterface        — raw UDP hardware driver.
  _camera     CameraManager (QThread)   — frame capture loop.
  _detector   MosquitoDetector (QThread)— detection loop.
  _laser      LaserManager (QThread)    — targeting loop.

Correct API notes (verified against each module):
  CameraManager:
    set_source_gopro(ip) / set_source_local(index) — configure before start().
    stop()              — signal loop to stop and wait.
    snapshot() → ndarray | None.
    Signals: frame_ready(ndarray), fps_updated(float),
             connection_changed(bool), error_occurred(str).

  MosquitoDetector:
    push_frame(frame)   — queue a frame for processing.
    set_mode(DetectionMode), set_sensitivity(float), reset_background().
    stop()              — signal loop to stop.
    Signals: detections_ready(list), fps_updated(float).

  LaserManager:
    set_laser_interface(LaserCubeInterface) — must call before connect_laser().
    set_safety_system(SafetySystem)         — must call before arm/fire.
    connect_laser() → bool  — opens UDP sockets.
    disconnect_laser()      — blanks and disconnects.
    arm() → bool / disarm() / emergency_stop().
    push_target(laser_x, laser_y, cam_x, cam_y, r, g, b).
    stop()              — signal loop to stop.
    Signals: laser_fired(int,int), connection_changed(bool),
             error_occurred(str), output_changed(bool).

  MosquitoTracker:
    update(detections) → list[TrackedTarget].
    get_best_target()  → TrackedTarget | None.
    live_track_count() → int.

  CoordinateMapper:
    camera_to_laser(cx, cy) → (int, int).

  CameraView:
    update_frame(frame), update_detections(list), update_tracks(list).
    set_target(TrackedTarget | None), set_fps(float), set_zones(list).
    set_show_tracks(bool), set_show_zones(bool), set_show_fps(bool).
    pixel_clicked Signal(float, float).

Modules imported:
  PySide6.QtWidgets — QMainWindow, QDockWidget, QStatusBar, QMessageBox,
                       QLabel, QMenuBar, QApplication
  PySide6.QtCore    — Qt, QTimer
  PySide6.QtGui     — QKeySequence, QCloseEvent, QAction
  camera.camera_manager        — CameraManager
  detection.detector           — MosquitoDetector, DetectionMode
  detection.tracker            — MosquitoTracker
  laser.laser_manager          — LaserManager
  laser.lasercube_interface    — LaserCubeInterface
  targeting.coordinate_mapper  — CoordinateMapper
  targeting.calibration        — CalibrationController
  utils.safety                 — SafetySystem
  gui.camera_view              — CameraView
  gui.control_panel            — ControlPanel
  gui.calibration_dialog       — CalibrationDialog
  config.settings              — all constants
  config.config_manager        — ConfigManager
  utils.logging_utils          — get_logger

Classes:
  MainWindow — Top-level application window.

Methods (MainWindow):
  _apply_theme()                    — Dark stylesheet.
  _build_dock()                     — Right ControlPanel dock.
  _build_menu()                     — Menu bar actions.
  _build_status_bar()               — Status bar labels.
  _wire_signals()                   — Connect all signals/slots.
  _start_threads()                  — Start all QThread workers.
  _stop_threads()                   — Graceful shutdown.
  _on_frame_ready(frame)            — Route frame to view + detector.
  _on_detections_ready(detections)  — Tracker + view + laser targeting.
  _on_laser_fired(lx, ly)           — Increment shot counter.
  _on_laser_error(msg)              — Log + status bar.
  _on_camera_fps(fps)               — Track fps, update view + status.
  _on_gopro_connect_requested()     — Configure camera and start thread.
  _on_gopro_disconnect_requested()  — Stop camera thread.
  _on_laser_connect_requested()     — Create interface, connect, start thread.
  _on_laser_disconnect_requested()  — Disarm then disconnect.
  _on_calibrate_requested()         — Open CalibrationDialog.
  _on_arm_requested()               — Arm safety system + laser.
  _on_disarm_requested()            — Disarm.
  _on_estop_requested()             — Emergency stop.
  _on_add_zone_requested()          — Begin zone-click flow.
  _on_remove_zone(zone_id)          — Remove a no-fire zone.
  _on_camera_clicked(x, y)          — Handle zone drawing clicks.
  _on_safety_changed(armed, estop)  — Safety callback → update UI (via timer).
  _push_overlay_settings()          — Sync view overlay checkboxes.
  _refresh_stats()                  — Periodic stats push to ControlPanel.
  closeEvent(event)                 — Graceful shutdown.
  showEvent(event)                  — Start threads on first show.

Variables (instance):
  _config        (ConfigManager):        Persistent settings.
  _safety        (SafetySystem):         Safety gatekeeper.
  _mapper        (CoordinateMapper):     Coordinate transform.
  _tracker       (MosquitoTracker):      Multi-object tracker.
  _cal_ctrl      (CalibrationController):Calibration driver.
  _interface     (LaserCubeInterface | None): Raw laser hardware.
  _camera        (CameraManager):        Camera QThread.
  _detector      (MosquitoDetector):     Detection QThread.
  _laser         (LaserManager):         Laser QThread.
  _camera_view   (CameraView):           Central widget.
  _panel         (ControlPanel):         Dock panel.
  _shots_fired   (int):                  Running total laser shots.
  _cam_fps       (float):                Last reported camera FPS.
  _det_fps       (float):                Last reported detection FPS.
  _zone_first_pt (tuple | None):         First click for zone drawing.
  _stats_timer   (QTimer):               Periodic stats refresh (500ms).

#todo Add local camera device chooser (QComboBox with cv2.VideoCapture indexes).
#todo Save/restore window geometry and dock state via QSettings.
#todo Add a "session recording" mode that saves detections + video to disk.
#todo Show a heatmap overlay of historical mosquito detections.
#todo Add a notification sound when the laser fires (configurable).
#todo Support multi-laser targeting for larger coverage areas.
"""

from __future__ import annotations

import numpy as np
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QAction, QCloseEvent, QKeySequence
from PySide6.QtWidgets import (QDockWidget, QLabel, QMainWindow, QMenuBar,
                               QMessageBox, QStatusBar)

from camera.camera_manager import CameraManager
from config import settings as _s
from config.config_manager import ConfigManager
from detection.detector import DetectionMode, MosquitoDetector
from detection.tracker import MosquitoTracker
from gui.calibration_dialog import CalibrationDialog
from gui.camera_view import CameraView
from gui.control_panel import ControlPanel
from laser.laser_manager import LaserManager
from laser.lasercube_interface import LaserCubeInterface
from targeting.calibration import CalibrationController
from targeting.coordinate_mapper import CoordinateMapper
from utils.logging_utils import get_logger
from utils.safety import SafetySystem

logger = get_logger(__name__)


class MainWindow(QMainWindow):
    """
    Top-level application window.

    Owns all subsystem lifetimes.  All cross-subsystem communication routes
    through Qt signals and SafetySystem.

    Args:
        config: ConfigManager instance holding run-time settings.
    """

    def __init__(self, config: ConfigManager) -> None:
        super().__init__()
        self._config = config
        self._shots_fired: int = 0
        self._cam_fps: float = 0.0
        self._det_fps: float = 0.0
        self._zone_first_pt: tuple[float, float] | None = None  # zone drawing state
        self._interface: LaserCubeInterface | None = None

        # ── Subsystems ────────────────────────────────────────────────────────
        self._safety = SafetySystem()
        self._mapper = CoordinateMapper()
        self._tracker = MosquitoTracker()
        self._cal_ctrl = CalibrationController(self._mapper)

        self._camera = CameraManager()
        self._detector = MosquitoDetector()
        self._laser = LaserManager()
        self._laser.set_safety_system(self._safety)

        # ── GUI ───────────────────────────────────────────────────────────────
        self.setWindowTitle(_s.GUI_WINDOW_TITLE)
        self.resize(_s.GUI_DEFAULT_WIDTH, _s.GUI_DEFAULT_HEIGHT)
        self._apply_theme()

        self._camera_view = CameraView(self)
        self.setCentralWidget(self._camera_view)

        self._build_dock()
        self._build_menu()
        self._build_status_bar()
        self._wire_signals()

        # ── Stats refresh ─────────────────────────────────────────────────────
        self._stats_timer = QTimer(self)
        self._stats_timer.setInterval(500)
        self._stats_timer.timeout.connect(self._refresh_stats)
        self._stats_timer.start()

        # Safety callback — zero-arg callable; query state inside via status_dict.
        # Scheduled onto main thread via QTimer.singleShot to avoid GUI updates
        # from non-main threads.
        self._safety.register_callback(self._on_safety_state_changed)

        # Initial UI state
        self._panel.set_gopro_status(False)
        self._panel.set_laser_status(False)
        self._panel.set_armed_state(False, False)

        logger.info("MainWindow initialised.")

    # ──────────────────────────────────────────────────────────────────────────
    # UI construction
    # ──────────────────────────────────────────────────────────────────────────

    def _apply_theme(self) -> None:
        """Apply application-wide dark stylesheet."""
        self.setStyleSheet("""
            QMainWindow { background-color: #1e1e2e; }
            QMenuBar { background-color: #181825; color: #cdd6f4; }
            QMenuBar::item:selected { background-color: #313244; }
            QMenu { background-color: #1e1e2e; color: #cdd6f4;
                    border: 1px solid #45475a; }
            QMenu::item:selected { background-color: #313244; }
            QStatusBar { background-color: #181825; color: #cdd6f4;
                          font-family: Consolas; font-size: 11px; }
            QDockWidget::title { background-color: #181825; color: #cba6f7;
                                  padding: 4px; font-weight: bold; }
        """)

    def _build_dock(self) -> None:
        """Create the right-side ControlPanel dock widget."""
        self._panel = ControlPanel(self)
        self._dock = QDockWidget("Control Panel", self)
        self._dock.setObjectName("ControlPanelDock")
        self._dock.setWidget(self._panel)
        self._dock.setFeatures(
            QDockWidget.DockWidgetFeature.DockWidgetMovable
            | QDockWidget.DockWidgetFeature.DockWidgetFloatable
        )
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self._dock)

    def _build_menu(self) -> None:
        """Build the menu bar with File, View, Tools and Help menus."""
        mb: QMenuBar = self.menuBar()

        # ── File ──
        file_menu = mb.addMenu("&File")
        act_quit = QAction("&Quit", self)
        act_quit.setShortcut(QKeySequence.StandardKey.Quit)
        act_quit.triggered.connect(self.close)
        file_menu.addAction(act_quit)

        # ── View ──
        view_menu = mb.addMenu("&View")
        self._act_show_fps = QAction("Show &FPS", self, checkable=True)
        self._act_show_fps.setChecked(True)
        self._act_show_tracks = QAction("Show &Tracks", self, checkable=True)
        self._act_show_tracks.setChecked(True)
        self._act_show_zones = QAction("Show No-Fire &Zones", self, checkable=True)
        self._act_show_zones.setChecked(True)
        self._act_show_panel = QAction("Show Control &Panel", self, checkable=True)
        self._act_show_panel.setChecked(True)
        self._act_show_panel.triggered.connect(
            lambda checked: self._dock.setVisible(checked)
        )
        for act in (
            self._act_show_fps,
            self._act_show_tracks,
            self._act_show_zones,
            None,
            self._act_show_panel,
        ):
            if act is None:
                view_menu.addSeparator()
            else:
                view_menu.addAction(act)
                act.triggered.connect(self._push_overlay_settings)

        # ── Tools ──
        tools_menu = mb.addMenu("&Tools")
        act_calibrate = QAction("&Calibrate Laser ↔ Camera…", self)
        act_calibrate.triggered.connect(self._on_calibrate_requested)
        act_reset_bg = QAction("Reset &Background Model", self)
        act_reset_bg.triggered.connect(self._detector.reset_background)
        act_estop = QAction("⛔ EMERGENCY STOP", self)
        act_estop.setShortcut(QKeySequence("Escape"))
        act_estop.triggered.connect(self._on_estop_requested)
        tools_menu.addAction(act_calibrate)
        tools_menu.addAction(act_reset_bg)
        tools_menu.addSeparator()
        tools_menu.addAction(act_estop)

        # ── Help ──
        help_menu = mb.addMenu("&Help")
        act_about = QAction("&About", self)
        act_about.triggered.connect(self._show_about)
        act_safety = QAction("⚠ Safety Guidelines", self)
        act_safety.triggered.connect(self._show_safety)
        help_menu.addAction(act_about)
        help_menu.addAction(act_safety)

    def _build_status_bar(self) -> None:
        """Build the bottom status bar with live counters."""
        sb: QStatusBar = self.statusBar()

        self._sb_cam = QLabel("Camera: —")
        self._sb_det = QLabel("Detections: 0")
        self._sb_laser = QLabel("Laser: DISCONNECTED")
        self._sb_armed = QLabel("🔒 DISARMED")
        self._sb_armed.setStyleSheet("color: #f9e2af; font-weight: bold;")

        for lbl in (self._sb_cam, self._sb_det, self._sb_laser, self._sb_armed):
            sb.addPermanentWidget(lbl)

    # ──────────────────────────────────────────────────────────────────────────
    # Signal wiring
    # ──────────────────────────────────────────────────────────────────────────

    def _wire_signals(self) -> None:
        """Connect all Qt signals from workers and the control panel to slots."""
        # Camera → view + detector
        self._camera.frame_ready.connect(self._on_frame_ready)
        self._camera.fps_updated.connect(self._on_camera_fps)
        self._camera.connection_changed.connect(self._panel.set_gopro_status)
        self._camera.error_occurred.connect(self._on_camera_error)

        # Detector → tracker + view
        self._detector.detections_ready.connect(self._on_detections_ready)
        self._detector.fps_updated.connect(self._on_detector_fps)

        # Laser
        self._laser.laser_fired.connect(self._on_laser_fired)
        self._laser.error_occurred.connect(self._on_laser_error)
        self._laser.connection_changed.connect(self._panel.set_laser_status)
        self._laser.connection_changed.connect(self._on_laser_connection_changed)

        # Camera view click → zone drawing
        self._camera_view.pixel_clicked.connect(self._on_camera_clicked)

        # Control panel
        self._panel.gopro_connect_requested.connect(self._on_gopro_connect_requested)
        self._panel.gopro_disconnect_requested.connect(self._on_gopro_disconnect_requested)
        self._panel.laser_connect_requested.connect(self._on_laser_connect_requested)
        self._panel.laser_disconnect_requested.connect(self._on_laser_disconnect_requested)
        self._panel.arm_requested.connect(self._on_arm_requested)
        self._panel.disarm_requested.connect(self._on_disarm_requested)
        self._panel.estop_requested.connect(self._on_estop_requested)
        self._panel.detection_mode_changed.connect(self._on_detection_mode_changed)
        self._panel.sensitivity_changed.connect(self._on_sensitivity_changed)
        self._panel.reset_background_requested.connect(self._detector.reset_background)
        self._panel.calibrate_requested.connect(self._on_calibrate_requested)
        self._panel.add_zone_requested.connect(self._on_add_zone_requested)
        self._panel.remove_zone_requested.connect(self._on_remove_zone)

    # ──────────────────────────────────────────────────────────────────────────
    # Thread lifecycle
    # ──────────────────────────────────────────────────────────────────────────

    def _start_threads(self) -> None:
        """Start the detector and laser background threads."""
        logger.info("Starting background threads…")
        self._detector.start()
        self._laser.start()
        # CameraManager is started only after explicit Connect GoPro

    def _stop_threads(self) -> None:
        """Gracefully stop all background threads."""
        logger.info("Stopping all threads…")

        for worker, name in (
            (self._laser, "LaserManager"),
            (self._camera, "CameraManager"),
            (self._detector, "MosquitoDetector"),
        ):
            try:
                worker.stop()
            except Exception as exc:
                logger.warning("%s stop() error: %s", name, exc)

    # ──────────────────────────────────────────────────────────────────────────
    # Core pipeline slots
    # ──────────────────────────────────────────────────────────────────────────

    def _on_frame_ready(self, frame: np.ndarray) -> None:
        """
        Route a captured frame to the camera view and the detector queue.

        Args:
            frame: BGR numpy array from CameraManager.
        """
        self._camera_view.update_frame(frame)
        self._detector.push_frame(frame)

    def _on_detections_ready(self, detections: list) -> None:
        """
        Receive detections, update tracker and view, push target to laser.

        Args:
            detections: List of Detection namedtuples from MosquitoDetector.
        """
        tracks = self._tracker.update(detections)
        self._camera_view.update_detections(detections)
        self._camera_view.update_tracks(tracks)

        best = self._tracker.get_best_target()
        if best is not None:
            # Predict position ahead to account for firing latency
            px, py = best.predicted_position(predict_ms=_s.TARGETING_PREDICT_MS)
            # Map to laser coordinates
            lx, ly = self._mapper.camera_to_laser(px, py)
            self._laser.push_target(lx, ly, px, py)
            self._camera_view.set_target(best)
        else:
            self._camera_view.set_target(None)

        count = len(detections)
        track_count = self._tracker.live_track_count()
        self._sb_det.setText(f"Det: {count} | Tracks: {track_count}")

    # ──────────────────────────────────────────────────────────────────────────
    # Laser slots
    # ──────────────────────────────────────────────────────────────────────────

    def _on_laser_fired(self, lx: int, ly: int) -> None:
        """Increment shot counter each time the laser fires."""
        self._shots_fired += 1

    def _on_laser_error(self, msg: str) -> None:
        logger.error("LaserManager error: %s", msg)
        self.statusBar().showMessage(f"Laser error: {msg}", 4000)

    def _on_laser_connection_changed(self, connected: bool) -> None:
        state = "CONNECTED" if connected else "DISCONNECTED"
        self._sb_laser.setText(f"Laser: {state}")

    # ──────────────────────────────────────────────────────────────────────────
    # Camera slots
    # ──────────────────────────────────────────────────────────────────────────

    def _on_camera_fps(self, fps: float) -> None:
        self._cam_fps = fps
        self._camera_view.set_fps(fps)
        self._sb_cam.setText(f"Camera: {fps:.1f} FPS")

    def _on_camera_error(self, msg: str) -> None:
        logger.warning("CameraManager: %s", msg)
        self.statusBar().showMessage(f"Camera: {msg}", 3000)

    def _on_detector_fps(self, fps: float) -> None:
        self._det_fps = fps

    # ──────────────────────────────────────────────────────────────────────────
    # Control panel action slots
    # ──────────────────────────────────────────────────────────────────────────

    def _on_gopro_connect_requested(self) -> None:
        """Configure camera for GoPro and start the capture thread."""
        ip: str = self._config.get("GOPRO_DEFAULT_IP")
        logger.info("Connecting to GoPro @ %s", ip)
        self._camera.set_source_gopro(ip)
        if not self._camera.isRunning():
            self._camera.start()

    def _on_gopro_disconnect_requested(self) -> None:
        """Stop the camera capture thread."""
        logger.info("Disconnecting GoPro.")
        self._camera.stop()
        self._panel.set_gopro_status(False)

    def _on_laser_connect_requested(self) -> None:
        """Create LaserCubeInterface, inject into LaserManager, connect."""
        ip: str = self._config.get("LASERCUBE_DEFAULT_IP")
        data_port: int = int(self._config.get("LASERCUBE_DATA_PORT"))
        cmd_port: int = int(self._config.get("LASERCUBE_CMD_PORT"))
        logger.info("Connecting to LaserCube @ %s:%d/%d", ip, data_port, cmd_port)

        self._interface = LaserCubeInterface(ip, data_port, cmd_port)
        self._laser.set_laser_interface(self._interface)
        if not self._laser.isRunning():
            self._laser.start()
        self._laser.connect_laser()

    def _on_laser_disconnect_requested(self) -> None:
        """Disarm and disconnect the laser."""
        logger.info("Disconnecting LaserCube.")
        self._on_disarm_requested()
        self._laser.disconnect_laser()

    def _on_arm_requested(self) -> None:
        """Arm the safety system and enable laser output."""
        ok = self._laser.arm()
        if not ok:
            QMessageBox.warning(
                self,
                "Cannot Arm",
                "System cannot arm.\n\n"
                "Possible reasons:\n"
                "• Emergency stop is active — reset it first.\n"
                "• LaserCube is not connected.",
            )
        else:
            logger.info("System ARMED.")

    def _on_disarm_requested(self) -> None:
        """Disarm and blank the laser."""
        self._laser.disarm()
        logger.info("System DISARMED.")

    def _on_estop_requested(self) -> None:
        """Trigger emergency stop immediately."""
        logger.critical("EMERGENCY STOP triggered by user!")
        self._laser.emergency_stop()
        self.statusBar().showMessage("⛔ EMERGENCY STOP ACTIVE — press Disarm to reset.", 0)

    def _on_detection_mode_changed(self, mode_str: str) -> None:
        mode = (
            DetectionMode.YOLO
            if mode_str == "yolo"
            else DetectionMode.BACKGROUND_SUBTRACTION
        )
        self._detector.set_mode(mode)
        logger.info("Detection mode → %s", mode_str)

    def _on_sensitivity_changed(self, value: int) -> None:
        self._detector.set_sensitivity(value / 100.0)

    def _on_calibrate_requested(self) -> None:
        """Open CalibrationDialog. Refuses if laser is not connected."""
        if self._interface is None or not self._interface.is_connected():
            QMessageBox.warning(
                self,
                "Laser Not Connected",
                "Please connect the LaserCube before calibrating.",
            )
            return

        dlg = CalibrationDialog(self._cal_ctrl, self)
        dlg.calibration_complete.connect(self._on_calibration_complete)
        dlg.start(self._camera.snapshot, self._laser)
        dlg.exec()

    def _on_calibration_complete(self, success: bool) -> None:
        if success:
            self.statusBar().showMessage("Calibration complete — saved.", 5000)
            logger.info("Calibration succeeded.")
        else:
            self.statusBar().showMessage("Calibration cancelled.", 3000)

    def _on_add_zone_requested(self) -> None:
        """
        Begin the two-click no-fire zone drawing flow.

        The user clicks two corners on the camera view; _on_camera_clicked
        handles both clicks and calls SafetySystem.add_no_fire_zone().
        """
        self._zone_first_pt = None
        self.statusBar().showMessage(
            "Click two corners of the no-fire zone on the camera view…", 0
        )

    def _on_remove_zone(self, zone_id: int) -> None:
        """Remove a no-fire zone by ID and refresh the UI."""
        self._safety.remove_no_fire_zone(zone_id)
        zones = self._safety.get_zones()
        self._panel.refresh_zones(zones)
        self._camera_view.set_zones(zones)
        logger.info("No-fire zone #%d removed.", zone_id)

    def _on_camera_clicked(self, x: float, y: float) -> None:
        """
        Handle clicks on the camera view for two-corner zone drawing.

        First click stores the corner; second click completes the zone.
        """
        if not (
            self.statusBar().currentMessage().startswith("Click two corners")
        ):
            return

        if self._zone_first_pt is None:
            self._zone_first_pt = (x, y)
            self.statusBar().showMessage(
                f"First corner set at ({x:.0f}, {y:.0f}) — click second corner…", 0
            )
        else:
            x1, y1 = self._zone_first_pt
            x2, y2 = x, y
            self._zone_first_pt = None
            # Normalise so x1,y1 is top-left
            lx, ly = min(x1, x2), min(y1, y2)
            rx, ry = max(x1, x2), max(y1, y2)
            zid = self._safety.add_no_fire_zone(int(lx), int(ly), int(rx), int(ry), label="User Zone")
            zones = self._safety.get_zones()
            self._panel.refresh_zones(zones)
            self._camera_view.set_zones(zones)
            self.statusBar().clearMessage()
            logger.info(
                "No-fire zone #%d added: (%.0f,%.0f)–(%.0f,%.0f)",
                zid, lx, ly, rx, ry,
            )

    # ──────────────────────────────────────────────────────────────────────────
    # Safety callback
    # ──────────────────────────────────────────────────────────────────────────

    def _on_safety_state_changed(self) -> None:
        """
        Zero-argument callback registered with SafetySystem.

        Called from any thread on any state change.  Queries the current state
        and marshals a GUI update to the main thread via QTimer.singleShot(0).
        """
        status = self._safety.status_dict()
        armed: bool = status["armed"]
        estop: bool = status["estop"]

        def _update() -> None:
            self._panel.set_armed_state(armed, estop)
            if estop:
                self._sb_armed.setText("⛔ E-STOP")
                self._sb_armed.setStyleSheet("color: #f38ba8; font-weight: bold;")
            elif armed:
                self._sb_armed.setText("⚡ ARMED")
                self._sb_armed.setStyleSheet("color: #a6e3a1; font-weight: bold;")
            else:
                self._sb_armed.setText("🔒 DISARMED")
                self._sb_armed.setStyleSheet("color: #f9e2af; font-weight: bold;")

        QTimer.singleShot(0, _update)

    # ──────────────────────────────────────────────────────────────────────────
    # Overlay settings sync
    # ──────────────────────────────────────────────────────────────────────────

    def _push_overlay_settings(self) -> None:
        """Push current menu-checkbox states to the CameraView widget."""
        self._camera_view.set_show_fps(self._act_show_fps.isChecked())
        self._camera_view.set_show_tracks(self._act_show_tracks.isChecked())
        self._camera_view.set_show_zones(self._act_show_zones.isChecked())

    # ──────────────────────────────────────────────────────────────────────────
    # Stats refresh
    # ──────────────────────────────────────────────────────────────────────────

    def _refresh_stats(self) -> None:
        """Push latest stats to the ControlPanel every 500ms."""
        self._panel.update_stats(
            cam_fps=self._cam_fps,
            det_fps=self._det_fps,
            tracks=self._tracker.live_track_count(),
            shots=self._shots_fired,
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Info dialogs
    # ──────────────────────────────────────────────────────────────────────────

    def _show_about(self) -> None:
        QMessageBox.about(
            self,
            "About Iron Dome Anti-Mosquito System",
            f"<b>Iron Dome Anti-Mosquito System</b><br>"
            f"Version {_s.VERSION}<br><br>"
            "Real-time mosquito detection and laser neutralisation.<br>"
            "GoPro camera + LaserCube • Python + PySide6 + OpenCV<br><br>"
            "<i>⚠ For research/recreational use only. "
            "Always observe laser safety guidelines.</i>",
        )

    def _show_safety(self) -> None:
        QMessageBox.warning(
            self,
            "⚠ Laser Safety Guidelines",
            "<b>CRITICAL SAFETY RULES:</b><br><br>"
            "1. <b>NEVER</b> aim the laser at people, animals, or reflective surfaces.<br>"
            "2. <b>Always</b> ensure the target area is clear before arming.<br>"
            "3. Configure <b>no-fire zones</b> around any area with occupants.<br>"
            "4. Keep the <b>Emergency Stop</b> accessible and tested.<br>"
            "5. This system uses a high-powered laser — eye exposure is permanently injurious.<br>"
            "6. Never leave the armed system unattended.<br>"
            "7. Comply with all local laser safety and emissions regulations.",
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Window lifecycle
    # ──────────────────────────────────────────────────────────────────────────

    def showEvent(self, event) -> None:
        """Start background threads on first show only."""
        super().showEvent(event)
        if not self._detector.isRunning() and not self._laser.isRunning():
            self._start_threads()

    def closeEvent(self, event: QCloseEvent) -> None:
        """Trigger e-stop, stop all threads, then accept close."""
        logger.info("MainWindow closing — stopping all systems…")
        # Hard-stop laser first
        try:
            self._laser.emergency_stop()
        except Exception:
            pass

        self._stats_timer.stop()
        self._stop_threads()
        event.accept()
