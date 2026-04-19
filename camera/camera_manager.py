"""
Iron Dome Anti-Mosquito System
================================
Mission: Automatically detect and neutralize mosquitos in real-time using a
         GoPro camera for vision and a LaserCube for precision targeting.

Camera Manager Module
---------------------
Provides a QThread-based camera capture loop that continuously reads frames
from either a GoPro (via RTSP/UDP stream) or a local webcam, and emits them
as Qt signals for consumption by the detection and GUI subsystems.

Supports two modes:
  - GOPRO: uses GoProInterface to connect, then opens the UDP preview stream
           via OpenCV (requires ffmpeg backend).
  - LOCAL: opens a local device index (0, 1, …) via cv2.VideoCapture directly.

Emitted signals (all emitted from the worker thread, received in GUI thread):
  frame_ready(np.ndarray)  — New BGR frame as a numpy array.
  fps_updated(float)       — Current measured capture FPS.
  connection_changed(bool) — Camera connected/disconnected state transition.
  error_occurred(str)      — Human-readable error description.

Modules imported:
  cv2                — OpenCV video capture
  numpy              — Frame array type
  PySide6.QtCore     — QThread, Signal, QTimer
  camera.gopro_interface — GoProInterface
  config.settings    — GOPRO_PREVIEW_STREAM_URL, GOPRO_DEFAULT_IP,
                        DETECTION_FPS_TARGET, GUI_CAMERA_FPS_DISPLAY
  utils.logging_utils — get_logger

Classes:
  CameraMode         — Enum for GOPRO vs LOCAL source.
  CameraManager      — QThread subclass managing the capture loop.

Methods (CameraManager):
  set_source_gopro(ip)     — Configure for GoPro; will connect on run().
  set_source_local(index)  — Configure for local webcam device index.
  stop()                   — Gracefully stop the capture loop and thread.
  snapshot()               — Return the most recent frame (or None).
  is_running()             — True if the capture loop is active.
  gopro                    — Property: GoProInterface instance (if GOPRO mode).

Variables (instance):
  _mode          — CameraMode enum value.
  _gopro_ip      — IP string for GoPro mode.
  _local_index   — Device index for local mode.
  _cap           — cv2.VideoCapture or None while stopped.
  _running       — bool; set False to stop the capture loop.
  _last_frame    — Most recent numpy frame for snapshot().
  _fps_counter   — Rolling FPS measurement state.

#todo Add automatic reconnect with exponential backoff on stream drop.
#todo Support RTMP stream ingest path (e.g. from IP cameras / action cams).
#todo Add recording to disk (cv2.VideoWriter) with configurable path.
#todo Expose camera settings (resolution, exposure) to the control panel.
"""

from __future__ import annotations

import enum
import time
from typing import Optional

import cv2
import numpy as np
from PySide6.QtCore import QThread, Signal

import config.settings as _s
from camera.gopro_interface import GoProInterface
from utils.logging_utils import get_logger

logger = get_logger(__name__)


class CameraMode(enum.Enum):
    """
    Enum representing the camera input source mode.

    Attributes:
        GOPRO: Capture from GoPro WiFi preview stream.
        LOCAL: Capture from a local USB/CSI camera device.
    """
    GOPRO = "gopro"
    LOCAL = "local"


class CameraManager(QThread):
    """
    QThread that continuously captures frames from a camera source.

    Emits frame_ready(ndarray) for every captured frame so that the GUI
    and detection pipeline can consume them without blocking the main thread.

    Signals:
        frame_ready (np.ndarray): New BGR frame captured.
        fps_updated (float):      Rolling measured FPS.
        connection_changed (bool):Camera went online (True) or offline (False).
        error_occurred (str):     Human-readable error message.

    Attributes:
        _mode       (CameraMode): Current source mode (GOPRO or LOCAL).
        _gopro_ip   (str):        IP for GOPRO mode.
        _local_index(int):        Device index for LOCAL mode.
        _cap        (cv2.VideoCapture | None): Active capture object.
        _running    (bool):       Loop control flag.
        _last_frame (np.ndarray | None): Most recent captured frame.
        _gopro      (GoProInterface | None): GoProInterface instance.
        _fps_times  (list[float]): Timestamps of recent frames for FPS calc.
    """

    frame_ready = Signal(np.ndarray)
    fps_updated = Signal(float)
    connection_changed = Signal(bool)
    error_occurred = Signal(str)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._mode: CameraMode = CameraMode.GOPRO
        self._gopro_ip: str = _s.GOPRO_DEFAULT_IP
        self._local_index: int = 0
        self._cap: cv2.VideoCapture | None = None
        self._running: bool = False
        self._last_frame: np.ndarray | None = None
        self._gopro: GoProInterface | None = None
        self._fps_times: list[float] = []

    # ──────────────────────────────────────────────────────────────────────────
    # Configuration (must be called before start())
    # ──────────────────────────────────────────────────────────────────────────

    def set_source_gopro(self, ip: str = _s.GOPRO_DEFAULT_IP) -> None:
        """
        Configure this manager to capture from a GoPro.

        Args:
            ip: GoPro AP IP address (default 10.5.5.9).
        """
        self._mode = CameraMode.GOPRO
        self._gopro_ip = ip
        logger.info("Camera source set to GOPRO @ %s", ip)

    def set_source_local(self, index: int = 0) -> None:
        """
        Configure this manager to capture from a local camera device.

        Args:
            index: OpenCV device index (0 = first camera, 1 = second, etc.).
        """
        self._mode = CameraMode.LOCAL
        self._local_index = index
        logger.info("Camera source set to LOCAL device #%d", index)

    @property
    def gopro(self) -> GoProInterface | None:
        """Return the GoProInterface instance, if in GOPRO mode."""
        return self._gopro

    # ──────────────────────────────────────────────────────────────────────────
    # QThread run() — the capture loop
    # ──────────────────────────────────────────────────────────────────────────

    def run(self) -> None:
        """
        Main capture loop executed in the worker thread by QThread.start().

        Connects to the configured source, then reads frames in a tight loop
        until stop() is called.  Emits frame_ready for each valid frame.
        Attempts one reconnect on connection loss before quitting.
        """
        self._running = True
        logger.info("CameraManager thread started (mode=%s).", self._mode.value)

        try:
            connected = self._open_source()
        except Exception as exc:  # noqa: BLE001
            logger.error("Camera open failed: %s", exc)
            self.error_occurred.emit(f"Camera open failed: {exc}")
            self._running = False
            return

        if not connected:
            self.connection_changed.emit(False)
            self._running = False
            return

        self.connection_changed.emit(True)
        target_interval = 1.0 / _s.DETECTION_FPS_TARGET
        consecutive_failures = 0
        max_failures = 30  # consecutive read failures before reconnect attempt

        while self._running:
            t0 = time.monotonic()

            if self._cap is None or not self._cap.isOpened():
                logger.warning("Capture closed; attempting reconnect …")
                self.connection_changed.emit(False)
                if not self._open_source():
                    logger.error("Reconnect failed — stopping camera.")
                    self.error_occurred.emit("Camera reconnect failed.")
                    break
                self.connection_changed.emit(True)
                consecutive_failures = 0
                continue

            ret, frame = self._cap.read()
            if not ret or frame is None:
                consecutive_failures += 1
                if consecutive_failures >= max_failures:
                    logger.warning(
                        "%d consecutive read failures — reconnecting.", max_failures
                    )
                    self._close_source()
                    consecutive_failures = 0
                continue

            consecutive_failures = 0
            self._last_frame = frame
            self.frame_ready.emit(frame)
            self._update_fps()

            # Pace to target FPS (sleep if we read faster than needed)
            elapsed = time.monotonic() - t0
            sleep_s = target_interval - elapsed
            if sleep_s > 0:
                self.msleep(int(sleep_s * 1000))

        self._close_source()
        self.connection_changed.emit(False)
        logger.info("CameraManager thread exited.")

    # ──────────────────────────────────────────────────────────────────────────
    # Public control
    # ──────────────────────────────────────────────────────────────────────────

    def stop(self) -> None:
        """
        Signal the capture loop to stop, then wait for the thread to finish.
        """
        logger.info("Stopping CameraManager …")
        self._running = False
        self.wait(5000)  # wait up to 5 s for thread to finish

    def snapshot(self) -> np.ndarray | None:
        """
        Return the most recently captured frame without waiting.

        Returns:
            numpy ndarray (BGR) or None if no frame has been captured yet.
        """
        return self._last_frame.copy() if self._last_frame is not None else None

    def is_running(self) -> bool:
        """Return True if the capture loop is actively running."""
        return self._running

    # ──────────────────────────────────────────────────────────────────────────
    # Internal source management
    # ──────────────────────────────────────────────────────────────────────────

    def _open_source(self) -> bool:
        """
        Open the camera capture source based on the current mode.

        Returns:
            True if the source was opened successfully, False otherwise.
        """
        if self._mode == CameraMode.GOPRO:
            return self._open_gopro()
        return self._open_local()

    def _open_gopro(self) -> bool:
        """
        Connect to GoPro, start preview stream, then open UDP capture.

        Returns:
            True on success.
        """
        self._gopro = GoProInterface(ip=self._gopro_ip)
        if not self._gopro.connect():
            self.error_occurred.emit("Could not connect to GoPro. Check WiFi.")
            return False

        if not self._gopro.start_stream():
            self.error_occurred.emit("GoPro connected but stream start failed.")
            return False

        # Give GoPro ~1 s to begin sending UDP packets before opening capture
        time.sleep(1.0)

        url = _s.GOPRO_PREVIEW_STREAM_URL
        logger.info("Opening GoPro stream: %s", url)
        self._cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)

        if not self._cap.isOpened():
            self.error_occurred.emit(
                f"OpenCV could not open GoPro stream ({url}). "
                "Ensure ffmpeg backend is available."
            )
            return False

        logger.info("GoPro stream opened successfully.")
        return True

    def _open_local(self) -> bool:
        """
        Open a local camera device.

        Returns:
            True if opened successfully.
        """
        logger.info("Opening local camera device #%d …", self._local_index)
        self._cap = cv2.VideoCapture(self._local_index)
        if not self._cap.isOpened():
            self.error_occurred.emit(
                f"Could not open local camera device #{self._local_index}."
            )
            return False
        logger.info("Local camera #%d opened.", self._local_index)
        return True

    def _close_source(self) -> None:
        """Release the OpenCV capture and disconnect GoPro if active."""
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        if self._gopro is not None:
            self._gopro.disconnect()
            self._gopro = None

    # ──────────────────────────────────────────────────────────────────────────
    # FPS measurement
    # ──────────────────────────────────────────────────────────────────────────

    def _update_fps(self) -> None:
        """
        Track frame timestamps and emit fps_updated with a rolling average.
        Emits once per second.
        """
        now = time.monotonic()
        self._fps_times.append(now)
        # Keep only timestamps from the last 2 seconds
        self._fps_times = [t for t in self._fps_times if now - t <= 2.0]
        if len(self._fps_times) >= 2:
            fps = (len(self._fps_times) - 1) / (self._fps_times[-1] - self._fps_times[0])
            # Emit at most once per second to avoid overwhelming the GUI
            if len(self._fps_times) % _s.DETECTION_FPS_TARGET == 0:
                self.fps_updated.emit(fps)
