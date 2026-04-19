"""
Iron Dome Anti-Mosquito System
================================
Mission: Automatically detect and neutralize mosquitos in real-time using a
         GoPro camera for vision and a LaserCube for precision targeting.

Calibration Dialog Module
--------------------------
Provides CalibrationDialog: a step-by-step QDialog wizard that guides the
user through the hardware calibration procedure.

The calibration works by iterating CalibrationController.next_point() for
each grid position.  next_point() fires the laser, waits for it to settle
(~150ms sleep internally), captures a frame, and detects the dot in a single
blocking call.  To prevent freezing the GUI, each step is run inside a
QThread (_StepWorker) and results are returned via Qt signals.

Workflow:
  1. Intro page  — explains prerequisites and laser safety.
  2. Steps page  — "next_point" runs in background thread, UI shows live
                   feedback and progress bar.  User clicks "Next" for each step
                   or can tick "Auto" to advance automatically.
  3. Result page — shows success / reprojection error / retry option.

CalibrationController API used:
  controller.start(frame_fn, laser_mgr) — begin session, capture background.
  controller.next_point()               — fire+detect+record one step.
                                          Returns (bool success, str message).
  controller.current_step()             — returns (current_idx, total_steps).
  controller.is_complete()              — True when all steps done.
  controller.finish()                   — compute homography. Returns bool.
  controller.cancel()                   — abort and discard.

Modules imported:
  PySide6.QtCore    — QThread, QTimer, Signal, Slot, Qt
  PySide6.QtWidgets — QDialog, QLabel, QProgressBar, QPushButton,
                       QTextEdit, QCheckBox, QVBoxLayout, QHBoxLayout
  utils.logging_utils — get_logger
  config.settings     — CALIBRATION_GRID_COLS, CALIBRATION_GRID_ROWS

Classes:
  _StepWorker     — QThread that calls controller.next_point() off the GUI thread.
  CalibrationDialog — Main wizard dialog.

Signals (_StepWorker):
  step_done (bool, str) — Delivered with (success, message) after next_point().

Signals (CalibrationDialog):
  calibration_complete (bool) — True = success, False = cancelled/failed.

Methods (CalibrationDialog):
  start(frame_fn, laser_manager) — Set up controller and begin wizard.
  _show_intro_page()             — Render the "before you begin" page.
  _show_step_page()              — Render the step page and fire first point.
  _show_result_page(ok)          — Render results after finish().
  _advance()                     — Kick off a _StepWorker for the next step.
  _on_step_done(ok, msg)         — Slot receiving result from _StepWorker.
  _run_finish()                  — Call controller.finish() and show result.
  _on_action_clicked()           — Route action button click by current page.
  _cancel()                      — Abort calibration and reject dialog.
  _log(msg)                      — Append message to in-dialog log box.

Variables (instance):
  _controller (CalibrationController): Calibration logic driver.
  _laser_mgr  (LaserManager | None):   Laser manager reference.
  _frame_fn   (Callable | None):       Camera snapshot function.
  _page_mode  (str):                   "intro" | "steps" | "result".
  _worker     (_StepWorker | None):    Active step worker thread.
  _total_steps(int):                   Total grid positions.
  _progress   (QProgressBar):          Overall progress bar.
  _lbl_step   (QLabel):                Current step description.
  _lbl_status (QLabel):                Detection result label.
  _log_box    (QTextEdit):             Rolling log of step results.
  _chk_auto   (QCheckBox):             Auto-advance checkbox.
  _btn_action (QPushButton):           Primary action button.
  _btn_cancel (QPushButton):           Cancel button.

#todo Add visual frame capture thumbnail for each detected dot.
#todo Allow user to click the dot manually if auto-detection fails.
#todo Show reprojection error per point on the result page.
#todo Export calibration report to PDF with annotated frame captures.
"""

from __future__ import annotations

from typing import Callable

import numpy as np
from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtWidgets import (QCheckBox, QDialog, QHBoxLayout, QLabel,
                               QProgressBar, QPushButton, QTextEdit,
                               QVBoxLayout, QWidget)

from config import settings as _s
from utils.logging_utils import get_logger

logger = get_logger(__name__)


# ─── Background worker ────────────────────────────────────────────────────────

class _StepWorker(QThread):
    """
    Runs CalibrationController.next_point() in a background thread.

    next_point() contains a ~150ms time.sleep; running it here prevents
    blocking the Qt event loop during that settle time.

    Signals:
        step_done (bool, str): (success, human-readable message).
    """

    step_done = Signal(bool, str)

    def __init__(self, controller, parent=None) -> None:
        super().__init__(parent)
        self._controller = controller

    def run(self) -> None:
        try:
            ok, msg = self._controller.next_point()
        except Exception as exc:
            ok, msg = False, f"Exception in next_point: {exc}"
            logger.error("[StepWorker] %s", msg)
        self.step_done.emit(ok, msg)


# ─── CalibrationDialog ────────────────────────────────────────────────────────

class CalibrationDialog(QDialog):
    """
    Step-by-step calibration wizard.

    Drives CalibrationController through each grid point using background
    QThread workers, showing live feedback and progress.

    Signals:
        calibration_complete (bool): True = success, False = cancelled/failed.
    """

    calibration_complete = Signal(bool)

    def __init__(self, controller, parent=None) -> None:
        """
        Args:
            controller: CalibrationController instance.
            parent:     Parent QWidget.
        """
        super().__init__(parent)
        self._controller = controller
        self._laser_mgr = None
        self._frame_fn: Callable[[], np.ndarray | None] | None = None
        self._total_steps = _s.CALIBRATION_GRID_COLS * _s.CALIBRATION_GRID_ROWS
        self._page_mode: str = "intro"
        self._worker: _StepWorker | None = None

        self.setWindowTitle("Laser ↔ Camera Calibration Wizard")
        self.setMinimumSize(520, 440)
        self.setModal(True)
        self.setStyleSheet("""
            QDialog  { background-color: #1e1e2e; color: #cdd6f4;
                       font-family: Consolas; }
            QPushButton { border-radius: 4px; padding: 6px 14px;
                          font-weight: bold; }
            QLabel   { color: #cdd6f4; }
            QTextEdit { background-color: #181825; color: #a6e3a1;
                        border: 1px solid #45475a; }
            QCheckBox { color: #cdd6f4; }
        """)

        self._build_ui()
        self._show_intro_page()

    # ──────────────────────────────────────────────────────────────────────────
    # Public API
    # ──────────────────────────────────────────────────────────────────────────

    def start(
        self,
        frame_fn: Callable[[], np.ndarray | None],
        laser_manager,
    ) -> None:
        """
        Initialise the calibration controller and transition to the step page.

        Call this BEFORE exec()-ing the dialog.

        Args:
            frame_fn:      Zero-arg callable returning the latest BGR frame.
            laser_manager: LaserManager instance for firing test points.
        """
        self._frame_fn = frame_fn
        self._laser_mgr = laser_manager
        try:
            self._controller.start(frame_fn, laser_manager)
        except Exception as exc:
            logger.error("CalibrationController.start failed: %s", exc)
            return
        self._show_step_page()

    # ──────────────────────────────────────────────────────────────────────────
    # UI construction
    # ──────────────────────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        """Build the static skeleton shared across all pages."""
        self._main_layout = QVBoxLayout(self)
        self._main_layout.setContentsMargins(16, 16, 16, 16)
        self._main_layout.setSpacing(10)

        # Title
        self._lbl_title = QLabel("Calibration Wizard")
        self._lbl_title.setStyleSheet(
            "font-size: 18px; font-weight: bold; color: #cba6f7;"
        )
        self._lbl_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._main_layout.addWidget(self._lbl_title)

        # Swappable page area
        self._page_widget = QWidget()
        self._page_layout = QVBoxLayout(self._page_widget)
        self._page_layout.setContentsMargins(0, 0, 0, 0)
        self._main_layout.addWidget(self._page_widget, stretch=1)

        # Progress bar
        self._progress = QProgressBar()
        self._progress.setRange(0, self._total_steps)
        self._progress.setValue(0)
        self._progress.setVisible(False)
        self._progress.setStyleSheet(
            "QProgressBar { background-color:#313244; border-radius:4px; height:16px; }"
            "QProgressBar::chunk { background-color:#cba6f7; border-radius:4px; }"
        )
        self._main_layout.addWidget(self._progress)

        # Button row
        btn_row = QHBoxLayout()
        self._btn_action = QPushButton("Begin Calibration")
        self._btn_action.setStyleSheet(
            "QPushButton { background-color:#cba6f7; color:#1e1e2e; }"
            "QPushButton:hover { background-color:#b4befe; }"
            "QPushButton:disabled { background-color:#45475a; color:#6c7086; }"
        )
        self._btn_cancel = QPushButton("Cancel")
        self._btn_cancel.setStyleSheet(
            "QPushButton { background-color:#313244; color:#f38ba8;"
            "  border:1px solid #f38ba8; }"
        )
        btn_row.addStretch()
        btn_row.addWidget(self._btn_cancel)
        btn_row.addWidget(self._btn_action)
        self._main_layout.addLayout(btn_row)

        self._btn_action.clicked.connect(self._on_action_clicked)
        self._btn_cancel.clicked.connect(self._cancel)

    def _clear_page(self) -> None:
        while self._page_layout.count():
            item = self._page_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

    # ──────────────────────────────────────────────────────────────────────────
    # Page renderers
    # ──────────────────────────────────────────────────────────────────────────

    def _show_intro_page(self) -> None:
        self._page_mode = "intro"
        self._clear_page()
        self._progress.setVisible(False)

        html = (
            "<h3>Before you begin:</h3>"
            "<ul>"
            "<li>Mount the GoPro so it sees the full target area.</li>"
            "<li>Position the LaserCube so its beam covers the same area.</li>"
            "<li><b>⚠ Ensure NO people or animals are in the target zone.</b></li>"
            f"<li>The system will fire at <b>{self._total_steps} grid positions</b> "
            "and detect the laser dot each time.</li>"
            "<li>The room should be dim enough for the dot to be visible.</li>"
            "</ul>"
            "<p>Press <b>Begin Calibration</b> to start.</p>"
        )
        lbl = QLabel(html)
        lbl.setWordWrap(True)
        lbl.setStyleSheet("color: #cdd6f4; line-height: 1.5;")
        self._page_layout.addWidget(lbl)

        self._btn_action.setText("Begin Calibration")
        self._btn_action.setEnabled(True)

    def _show_step_page(self) -> None:
        self._page_mode = "steps"
        self._clear_page()
        self._progress.setVisible(True)
        self._progress.setValue(0)

        self._lbl_step = QLabel("Preparing…")
        self._lbl_step.setStyleSheet(
            "font-size: 14px; font-weight: bold; color: #89dceb;"
        )
        self._page_layout.addWidget(self._lbl_step)

        self._lbl_status = QLabel("")
        self._lbl_status.setWordWrap(True)
        self._page_layout.addWidget(self._lbl_status)

        self._log_box = QTextEdit()
        self._log_box.setReadOnly(True)
        self._log_box.setMaximumHeight(130)
        self._page_layout.addWidget(self._log_box)

        self._chk_auto = QCheckBox("Auto-advance to next step")
        self._chk_auto.setChecked(False)
        self._page_layout.addWidget(self._chk_auto)

        self._btn_action.setText("Next Point ▶")
        self._btn_action.setEnabled(True)

        # Fire first point immediately
        self._advance()

    def _show_result_page(self, ok: bool) -> None:
        self._page_mode = "result"
        self._clear_page()
        self._progress.setValue(self._total_steps)

        if ok:
            html = (
                "<h3 style='color:#a6e3a1;'>Calibration Successful ✓</h3>"
                "<p>Homography computed and saved to disk.</p>"
                "<p>The coordinate mapping will be loaded automatically on next launch.</p>"
            )
        else:
            html = (
                "<h3 style='color:#f38ba8;'>Calibration Failed ✗</h3>"
                "<p>Not enough valid calibration points were detected (need ≥ 4).</p>"
                "<p>Try again in a darker environment, or ensure the laser dot is "
                "clearly visible on the camera feed.</p>"
            )

        lbl = QLabel(html)
        lbl.setWordWrap(True)
        self._page_layout.addWidget(lbl)

        self._btn_action.setText("Accept ✓" if ok else "Retry")
        self._btn_action.setEnabled(True)
        self._btn_cancel.setText("Discard" if ok else "Cancel")

    # ──────────────────────────────────────────────────────────────────────────
    # Step driver
    # ──────────────────────────────────────────────────────────────────────────

    def _advance(self) -> None:
        """
        Dispatch a _StepWorker to call controller.next_point() off the UI thread.
        Disables the Next button while the step is running.
        """
        if self._controller.is_complete():
            self._run_finish()
            return

        current, total = self._controller.current_step()
        col = current % _s.CALIBRATION_GRID_COLS
        row = current // _s.CALIBRATION_GRID_COLS
        self._lbl_step.setText(
            f"Step {current + 1} / {total} — grid position ({col}, {row})"
        )
        self._lbl_status.setText("Firing laser and detecting dot…")
        self._lbl_status.setStyleSheet("color: #f9e2af;")
        self._btn_action.setEnabled(False)

        self._worker = _StepWorker(self._controller, self)
        self._worker.step_done.connect(self._on_step_done)
        self._worker.start()

    def _on_step_done(self, ok: bool, msg: str) -> None:
        """
        Slot: receive result of one completed step.

        Args:
            ok:  True if laser dot was detected successfully.
            msg: Human-readable description.
        """
        self._log(msg)

        if ok:
            self._lbl_status.setText(f"✓ {msg.split(':')[-1].strip()}")
            self._lbl_status.setStyleSheet("color: #a6e3a1;")
        else:
            self._lbl_status.setText("✗ Dot not detected — step recorded with fallback.")
            self._lbl_status.setStyleSheet("color: #f38ba8;")

        current, total = self._controller.current_step()
        self._progress.setValue(current)

        if self._controller.is_complete():
            self._btn_action.setText("Finish ✓")
            self._btn_action.setEnabled(True)
        else:
            self._btn_action.setEnabled(True)
            # Auto-advance if checkbox ticked
            if hasattr(self, "_chk_auto") and self._chk_auto.isChecked():
                self._advance()

    def _run_finish(self) -> None:
        """Call controller.finish() and transition to the result page."""
        self._btn_action.setEnabled(False)
        self._lbl_step.setText("Computing homography…")
        try:
            ok = self._controller.finish()
        except Exception as exc:
            logger.error("Calibration finish error: %s", exc)
            ok = False
        self._show_result_page(ok)

    # ──────────────────────────────────────────────────────────────────────────
    # Button handlers
    # ──────────────────────────────────────────────────────────────────────────

    def _on_action_clicked(self) -> None:
        if self._page_mode == "intro":
            # start() should already have been called by the launcher
            if self._frame_fn is not None:
                self._show_step_page()

        elif self._page_mode == "steps":
            if self._controller.is_complete():
                self._run_finish()
            else:
                self._advance()

        elif self._page_mode == "result":
            if self._btn_action.text().startswith("Accept"):
                self.calibration_complete.emit(True)
                self.accept()
            else:
                # Retry: restart from scratch
                try:
                    self._controller.cancel()
                    if self._frame_fn is not None:
                        self._controller.start(self._frame_fn, self._laser_mgr)
                except Exception as exc:
                    logger.error("Calibration retry error: %s", exc)
                self._show_step_page()

    def _cancel(self) -> None:
        # Stop any running worker
        if self._worker is not None and self._worker.isRunning():
            self._worker.quit()
            self._worker.wait(2000)
        try:
            self._controller.cancel()
        except Exception:
            pass
        self.calibration_complete.emit(False)
        self.reject()

    # ──────────────────────────────────────────────────────────────────────────
    # Helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _log(self, msg: str) -> None:
        logger.debug("[CalibDlg] %s", msg)
        if hasattr(self, "_log_box"):
            self._log_box.append(msg)

    def closeEvent(self, event) -> None:
        """Ensure the worker thread is cleaned up on close."""
        if self._worker is not None and self._worker.isRunning():
            self._worker.quit()
            self._worker.wait(1000)
        super().closeEvent(event)
