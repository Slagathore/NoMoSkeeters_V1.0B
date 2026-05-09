"""
Iron Dome Anti-Mosquito System
================================
Mission: Automatically detect and neutralize mosquitos in real-time using a
         GoPro camera for vision and a LaserCube for precision targeting.

Camera View Module
------------------
Provides CameraView: a QWidget that renders live camera frames as a video
display and draws detection/targeting overlays on top.

Features:
  - Efficient BGR→RGB conversion and QImage display via QPainter.
  - Detection overlay: green bounding boxes and centroid dots.
  - Tracker overlay: track ID label, velocity arrow.
  - Targeting overlay: red crosshair on the laser's current target.
  - Predicted-position marker: orange dot where the Kalman filter predicts
    the mosquito will be in TARGETING_PREDICT_MS milliseconds.
  - No-fire zones: semi-transparent red rectangles.
  - FPS counter (top-left corner).
  - Click callback: emit camera pixel coordinates on mouse press (for
    manual calibration point selection and no-fire zone drawing).

Modules imported:
  cv2, numpy                — frame data and type
  PySide6.QtWidgets         — QWidget, QLabel
  PySide6.QtGui             — QPainter, QImage, QPixmap, QPen, QColor, QFont
  PySide6.QtCore            — Qt, Signal, QRect, QPoint
  detection.detector        — Detection namedtuple (for type hint)
  detection.tracker         — TrackedTarget (for type hint)
  utils.safety              — NoFireZone (for overlay)
  config.settings           — overlay colour constants

Classes:
  CameraView    — QWidget subclass for live video display with overlays.

Methods (CameraView):
  update_frame(frame)           — Accept a new BGR numpy frame; schedule repaint.
  update_detections(detections) — Update the list of detection boxes to draw.
  update_tracks(tracks)         — Update tracked target state for overlay.
  set_target(track)             — Set the primary targeted track for crosshair.
  set_zones(zones)              — Set no-fire zones for overlay drawing.
  clear_overlays()              — Clear all overlay state.
  set_show_fps(enabled)         — Toggle FPS counter display.
  set_show_tracks(enabled)      — Toggle track overlay.
  set_show_zones(enabled)       — Toggle no-fire zone overlay.

Signals:
  pixel_clicked (float, float)  — Camera pixel (x, y) when user clicks frame.

Variables (instance):
  _frame         (QImage | None): Current frame converted for display.
  _detections    (list):          Current Detection objects.
  _tracks        (list):          Current TrackedTarget objects.
  _target_track  (TrackedTarget | None): Primary target for crosshair.
  _zones         (list[NoFireZone]):      No-fire zones.
  _fps           (float):         Last displayed FPS value.
  _show_fps      (bool):          FPS counter visibility toggle.
  _show_tracks   (bool):          Track overlay visibility toggle.
  _show_zones    (bool):          Zone overlay visibility toggle.

#todo Add a zoom/pan mode for detailed inspection of a small area.
#todo Support recording the annotated video feed to disk.
#todo Add a drawing tool for interactive no-fire zone creation via mouse drag.
"""

from __future__ import annotations

import cv2
import numpy as np
from PySide6.QtCore import QPoint, QRect, QSize, Qt, Signal
from PySide6.QtGui import QColor, QFont, QImage, QPainter, QPen, QPixmap
from PySide6.QtWidgets import QSizePolicy, QWidget

import config.settings as _s
from utils.logging_utils import get_logger

logger = get_logger(__name__)


class CameraView(QWidget):
    """
    QWidget that displays a live BGR camera feed with overlay annotations.

    Accepts frame data and detection/tracking state from the application and
    repaints using QPainter.  Emits pixel_clicked when the user clicks the
    video area (used for calibration point selection and zone drawing).

    Signals:
        pixel_clicked (float, float): Camera-space (x, y) of mouse click.

    Attributes:
        _pixmap        (QPixmap | None): Scaled pixmap for current frame.
        _detections    (list): Detection namedtuples.
        _tracks        (list): TrackedTarget objects.
        _target_track  (TrackedTarget | None): Primary targeted track.
        _zones         (list): NoFireZone objects.
        _fps           (float): FPS value for HUD.
        _show_fps      (bool): Whether to render FPS.
        _show_tracks   (bool): Whether to render track overlays.
        _show_zones    (bool): Whether to render zone overlays.
        _native_w, _native_h (int): Native frame resolution.
    """

    pixel_clicked = Signal(float, float)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setMinimumSize(QSize(480, 270))
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setStyleSheet("background-color: #1a1a1a;")

        self._pixmap: QPixmap | None = None
        self._detections: list = []
        self._tracks: list = []
        self._target_track = None
        self._zones: list = []
        self._fps: float = 0.0
        self._show_fps: bool = True
        self._show_tracks: bool = True
        self._show_zones: bool = True
        self._native_w: int = 1
        self._native_h: int = 1

        # Pre-build pens for performance
        self._pen_detection = QPen(
            QColor(*_s.GUI_OVERLAY_BOX_COLOR[::-1]),  # BGR→RGB
            2,
        )
        self._pen_crosshair = QPen(
            QColor(*_s.GUI_OVERLAY_CROSS_COLOR[::-1]),
            2,
        )
        self._pen_predicted = QPen(
            QColor(*_s.GUI_OVERLAY_PRED_COLOR[::-1]),
            2,
        )
        self._pen_zone = QPen(QColor(180, 0, 0, 180), 2)
        self._brush_zone = QColor(180, 0, 0, 50)  # semi-transparent fill

        self._font_hud = QFont("Consolas", 10, QFont.Weight.Bold)
        self._font_track = QFont("Consolas", 8)

    # ──────────────────────────────────────────────────────────────────────────
    # Data setters (called from main thread via Qt signals)
    # ──────────────────────────────────────────────────────────────────────────

    def update_frame(self, frame: np.ndarray) -> None:
        """
        Accept a new BGR numpy frame and schedule a repaint.

        Converts BGR→RGB, constructs a QImage, scales to widget size, and
        stores as a QPixmap for painting.

        Args:
            frame: BGR numpy array from the camera.
        """
        if frame is None or frame.size == 0:
            return

        h, w, ch = frame.shape
        self._native_w = w
        self._native_h = h

        # BGR → RGB
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        bytes_per_line = ch * w
        qimg = QImage(
            rgb.data, w, h, bytes_per_line, QImage.Format.Format_RGB888
        )
        self._pixmap = QPixmap.fromImage(qimg).scaled(
            self.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.update()

    def update_detections(self, detections: list) -> None:
        """
        Update the detection overlay list.

        Args:
            detections: List of Detection namedtuples from MosquitoDetector.
        """
        self._detections = detections
        self.update()

    def update_tracks(self, tracks: list) -> None:
        """
        Update the tracked target overlay list.

        Args:
            tracks: List of TrackedTarget objects from MosquitoTracker.
        """
        self._tracks = tracks
        self.update()

    def set_target(self, track) -> None:
        """
        Set the primary targeted track for crosshair rendering.

        Args:
            track: TrackedTarget or None to clear the crosshair.
        """
        self._target_track = track
        self.update()

    def set_fps(self, fps: float) -> None:
        """
        Update the FPS value shown in the HUD.

        Args:
            fps: Current frame rate value.
        """
        self._fps = fps
        self.update()

    def set_zones(self, zones: list) -> None:
        """
        Set the no-fire zones to draw as overlays.

        Args:
            zones: List of NoFireZone objects.
        """
        self._zones = zones
        self.update()

    def clear_overlays(self) -> None:
        """Clear all overlay state."""
        self._detections = []
        self._tracks = []
        self._target_track = None
        self.update()

    def set_show_fps(self, enabled: bool) -> None:
        """Toggle FPS counter display."""
        self._show_fps = enabled
        self.update()

    def set_show_tracks(self, enabled: bool) -> None:
        """Toggle track/detection overlay display."""
        self._show_tracks = enabled
        self.update()

    def set_show_zones(self, enabled: bool) -> None:
        """Toggle no-fire zone overlay display."""
        self._show_zones = enabled
        self.update()

    # ──────────────────────────────────────────────────────────────────────────
    # QPainter rendering
    # ──────────────────────────────────────────────────────────────────────────

    def paintEvent(self, event) -> None:  # noqa: N802
        """Override QWidget.paintEvent to render frame + all overlays."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Background fill
        painter.fillRect(self.rect(), QColor("#1a1a1a"))

        if self._pixmap is None:
            # Draw "No Signal" placeholder
            painter.setPen(QColor("#888"))
            painter.setFont(QFont("Consolas", 14))
            painter.drawText(
                self.rect(),
                Qt.AlignmentFlag.AlignCenter,
                "No Camera Signal",
            )
            painter.end()
            return

        # Draw video frame centred
        x_offset = (self.width() - self._pixmap.width()) // 2
        y_offset = (self.height() - self._pixmap.height()) // 2
        painter.drawPixmap(x_offset, y_offset, self._pixmap)

        # Scale factors: native frame coords → widget display coords
        sx = self._pixmap.width() / max(self._native_w, 1)
        sy = self._pixmap.height() / max(self._native_h, 1)

        def tx(val: float) -> int:
            return int(val * sx) + x_offset

        def ty(val: float) -> int:
            return int(val * sy) + y_offset

        # ── No-fire zones ──────────────────────────────────────────────────
        if self._show_zones and self._zones:
            for zone in self._zones:
                x1 = tx(zone.x1)
                y1 = ty(zone.y1)
                x2 = tx(zone.x2)
                y2 = ty(zone.y2)
                painter.setPen(self._pen_zone)
                painter.setBrush(self._brush_zone)
                painter.drawRect(QRect(x1, y1, x2 - x1, y2 - y1))
                painter.setBrush(Qt.BrushStyle.NoBrush)
                painter.setFont(self._font_track)
                painter.setPen(QColor(255, 80, 80))
                painter.drawText(x1 + 4, y1 + 14, f"⛔ {zone.label}")

        # ── Detection boxes ────────────────────────────────────────────────
        if self._show_tracks:
            painter.setPen(self._pen_detection)
            painter.setBrush(Qt.BrushStyle.NoBrush)
            for det in self._detections:
                painter.drawRect(
                    QRect(tx(det.x1), ty(det.y1),
                          int((det.x2 - det.x1) * sx),
                          int((det.y2 - det.y1) * sy))
                )

            # ── Track IDs and velocity arrows ──────────────────────────────
            for track in self._tracks:
                cx = tx(track.cx)
                cy = ty(track.cy)
                painter.setPen(QPen(QColor(0, 200, 255), 1))
                painter.setFont(self._font_track)
                painter.drawText(cx + 5, cy - 5, f"#{track.track_id}")

                # Velocity arrow (scaled for visibility)
                vx_disp = track.vx * sx * 5
                vy_disp = track.vy * sy * 5
                if abs(vx_disp) + abs(vy_disp) > 2:
                    painter.setPen(QPen(QColor(255, 255, 0), 1))
                    painter.drawLine(
                        QPoint(cx, cy),
                        QPoint(int(cx + vx_disp), int(cy + vy_disp)),
                    )

                # Predicted position dot
                pred_cx, pred_cy = track.predicted_position()
                painter.setPen(self._pen_predicted)
                px, py = tx(pred_cx), ty(pred_cy)
                painter.drawEllipse(QPoint(px, py), 4, 4)

        # ── Targeting crosshair ────────────────────────────────────────────
        if self._target_track is not None:
            cx = tx(self._target_track.cx)
            cy = ty(self._target_track.cy)
            size = 14
            painter.setPen(self._pen_crosshair)
            # Crosshair lines
            painter.drawLine(cx - size, cy, cx + size, cy)
            painter.drawLine(cx, cy - size, cx, cy + size)
            # Outer circle
            painter.drawEllipse(QPoint(cx, cy), size, size)

        # ── HUD: FPS + status ──────────────────────────────────────────────
        if self._show_fps:
            painter.setPen(QColor(255, 255, 255))
            painter.setFont(self._font_hud)
            painter.drawText(
                x_offset + 6,
                y_offset + 18,
                f"FPS: {self._fps:.1f}",
            )
            if self._tracks:
                painter.drawText(
                    x_offset + 6,
                    y_offset + 34,
                    f"Tracks: {len(self._tracks)}",
                )

        painter.end()

    # ──────────────────────────────────────────────────────────────────────────
    # Mouse events (for calibration / zone drawing)
    # ──────────────────────────────────────────────────────────────────────────

    def mousePressEvent(self, event) -> None:  # noqa: N802
        """Emit pixel_clicked with camera-space coordinates of the click."""
        if self._pixmap is None or self._native_w == 0:
            return

        x_offset = (self.width() - self._pixmap.width()) // 2
        y_offset = (self.height() - self._pixmap.height()) // 2

        px = event.position().x() - x_offset
        py = event.position().y() - y_offset

        sx = self._native_w / max(self._pixmap.width(), 1)
        sy = self._native_h / max(self._pixmap.height(), 1)

        cam_x = px * sx
        cam_y = py * sy

        if 0 <= cam_x <= self._native_w and 0 <= cam_y <= self._native_h:
            self.pixel_clicked.emit(cam_x, cam_y)

    def resizeEvent(self, event) -> None:  # noqa: N802
        """Re-scale the stored pixmap when the widget is resized."""
        super().resizeEvent(event)
        # Will be naturally re-created on next frame; just trigger a repaint
        self.update()
