"""
Iron Dome Anti-Mosquito System
================================
Mission: Automatically detect and neutralize mosquitos in real-time using a
         GoPro camera for vision and a LaserCube for precision targeting.

GoPro Interface Module
----------------------
Provides a Python HTTP client for the GoPro Open GoPro API (v2).

The GoPro must be connected to the host machine over its WiFi AP:
  - SSID pattern:  "GOPRO-XXXXXX"  (shown in GoPro > Connections > Camera Info)
  - Host machine obtains IP via DHCP; GoPro API is at http://10.5.5.9:8080

Key API endpoints used:
  GET  /gopro/camera/info          — Returns model, serial number, etc.
  GET  /gopro/camera/state         — Returns current camera state.
  POST /gopro/camera/keep_alive    — Must be sent every ~5 s or WiFi drops.
  POST /gopro/camera/stream/start  — Tells GoPro to begin streaming preview.
  POST /gopro/camera/stream/stop   — Stops the preview stream.
  GET  /gopro/media/list           — Lists files on SD card (for future use).

Preview stream:
  After calling stream/start the GoPro sends H.264 video as UDP packets to
  this machine on port 8554.  OpenCV can open it with:
    cv2.VideoCapture("udp://0.0.0.0:8554", cv2.CAP_FFMPEG)

Modules imported:
  requests           — HTTP client for GoPro API
  threading          — Background keep-alive timer thread
  logging            — event logging
  config.settings    — GOPRO_DEFAULT_IP, GOPRO_API_PORT, GOPRO_TIMEOUT_S,
                       GOPRO_KEEP_ALIVE_INTERVAL_S, GOPRO_PREVIEW_STREAM_URL
  utils.logging_utils — get_logger

Classes:
  GoProInterface     — Manages HTTP sessions and keep-alive to the GoPro.

Methods (GoProInterface):
  connect()          — Open an HTTP session and verify camera is reachable.
  disconnect()       — Stop keep-alive and start_stream if running.
  start_stream()     — POST stream/start to begin UDP preview output.
  stop_stream()      — POST stream/stop to stop UDP preview output.
  get_camera_info()  — GET camera info; returns dict.
  get_camera_state() — GET camera state; returns dict.
  is_connected()     — True if the last HTTP call succeeded.
  stream_url         — Property: OpenCV-compatible stream URL string.

Variables (instance):
  _base_url         — Base URL string: "http://10.5.5.9:8080"
  _session          — requests.Session reused for all calls.
  _connected        — bool tracking last-known connection state.
  _streaming        — bool tracking whether stream/start was called.
  _keep_alive_timer — threading.Timer that fires periodically.

#todo Implement BLE control path for cameras that do not support WiFi streaming.
#todo Add GoPro settings configuration (resolution, frame rate, field of view).
#todo Handle multi-camera setups (two GoPros for stereo 3-D localisation).
#todo Parse telemetry (GPS, IMU) from GoPro metadata stream (GPMF).
"""

from __future__ import annotations

import threading
import time
from typing import Any

import requests

import config.settings as _s
from utils.logging_utils import get_logger

logger = get_logger(__name__)

# Timeout shorthand
_TIMEOUT = _s.GOPRO_TIMEOUT_S


class GoProInterface:
    """
    HTTP client wrapper for the GoPro Open GoPro API.

    Manages connection lifecycle, keep-alive pinging, and stream control.
    All network calls are synchronous (blocking) and guarded by try/except so
    callers receive a clean True/False / dict result rather than raw exceptions.

    Attributes:
        ip              (str):  IP address of the GoPro (default 10.5.5.9).
        api_port        (int):  HTTP API port (default 8080).
        _base_url       (str):  Full base URL e.g. "http://10.5.5.9:8080".
        _session   (requests.Session): Persistent HTTP session.
        _connected      (bool): Reflects last-known HTTP reachability.
        _streaming      (bool): True while preview stream is active.
        _ka_thread (threading.Thread | None): Keep-alive background thread.
        _ka_stop_event  (threading.Event): Signal to stop keep-alive loop.
    """

    def __init__(
        self,
        ip: str = _s.GOPRO_DEFAULT_IP,
        api_port: int = _s.GOPRO_API_PORT,
    ) -> None:
        """
        Initialise the GoPro interface.

        Args:
            ip:       GoPro IP address, default 10.5.5.9.
            api_port: HTTP API port, default 8080.
        """
        self.ip: str = ip
        self.api_port: int = api_port
        self._base_url: str = f"http://{ip}:{api_port}"
        self._session: requests.Session = requests.Session()
        self._session.headers.update({"Accept": "application/json"})
        self._connected: bool = False
        self._streaming: bool = False
        self._ka_thread: threading.Thread | None = None
        self._ka_stop_event: threading.Event = threading.Event()
        logger.debug("GoProInterface created for %s", self._base_url)

    # ──────────────────────────────────────────────────────────────────────────
    # Properties
    # ──────────────────────────────────────────────────────────────────────────

    @property
    def stream_url(self) -> str:
        """OpenCV-compatible URL string for the GoPro UDP preview stream."""
        return _s.GOPRO_PREVIEW_STREAM_URL

    def is_connected(self) -> bool:
        """Return True if the last HTTP call to GoPro succeeded."""
        return self._connected

    def is_streaming(self) -> bool:
        """Return True if stream/start was successfully called."""
        return self._streaming

    # ──────────────────────────────────────────────────────────────────────────
    # Connection management
    # ──────────────────────────────────────────────────────────────────────────

    def connect(self) -> bool:
        """
        Open an HTTP session and verify the GoPro is reachable.

        Sends a GET /gopro/camera/info request as a connectivity test.
        If successful, starts the background keep-alive thread.

        Returns:
            True if camera responded, False on network error or timeout.
        """
        logger.info("Connecting to GoPro at %s …", self._base_url)
        info = self.get_camera_info()
        if info is not None:
            self._connected = True
            self._start_keep_alive()
            logger.info("GoPro connected: %s", info.get("model_name", "unknown"))
            return True
        logger.error("GoPro connection failed — is the WiFi AP connected?")
        return False

    def disconnect(self) -> None:
        """
        Gracefully disconnect: stop the stream (if active) and keep-alive thread.
        """
        if self._streaming:
            self.stop_stream()
        self._stop_keep_alive()
        self._connected = False
        self._session.close()
        logger.info("GoPro disconnected.")

    # ──────────────────────────────────────────────────────────────────────────
    # Stream control
    # ──────────────────────────────────────────────────────────────────────────

    def start_stream(self) -> bool:
        """
        Instruct GoPro to begin sending the preview stream via UDP.

        The stream is sent to this machine on GOPRO_STREAM_UDP_PORT.
        After calling this, open the stream with cv2.VideoCapture(stream_url).

        Returns:
            True if GoPro acknowledged the request, False on error.
        """
        logger.info("Starting GoPro preview stream …")
        ok = self._post("/gopro/camera/stream/start")
        if ok:
            self._streaming = True
            logger.info(
                "GoPro streaming → %s",
                _s.GOPRO_PREVIEW_STREAM_URL,
            )
        else:
            logger.error("Failed to start GoPro stream.")
        return ok

    def stop_stream(self) -> bool:
        """
        Instruct GoPro to stop the preview stream.

        Returns:
            True if GoPro acknowledged, False on error.
        """
        logger.info("Stopping GoPro preview stream …")
        ok = self._post("/gopro/camera/stream/stop")
        self._streaming = False
        return ok

    # ──────────────────────────────────────────────────────────────────────────
    # Camera info / state
    # ──────────────────────────────────────────────────────────────────────────

    def get_camera_info(self) -> dict[str, Any] | None:
        """
        Retrieve static camera information from GET /gopro/camera/info.

        Returns:
            dict with fields like "model_name", "serial_number", "firmware_version",
            or None if the request failed.
        """
        resp = self._get("/gopro/camera/info")
        if resp is None:
            return None
        return resp.get("info", resp)

    def get_camera_state(self) -> dict[str, Any] | None:
        """
        Retrieve dynamic camera state from GET /gopro/camera/state.

        The state dict contains settings and status sub-dicts that map
        integer keys to values.  Consult the GoPro Open API docs for the
        full key-value table.

        Returns:
            dict with "status" and "settings" sub-dicts, or None on error.
        """
        return self._get("/gopro/camera/state")

    def set_preset(self, preset_id: int) -> bool:
        """
        Load a preset on the GoPro (e.g. Photo, Video, Time Lapse).

        Args:
            preset_id: Numeric preset ID (see GoPro Open API docs).

        Returns:
            True if accepted, False on error.
        """
        return self._post(f"/gopro/camera/presets/load?id={preset_id}")

    # ──────────────────────────────────────────────────────────────────────────
    # Keep-alive
    # ──────────────────────────────────────────────────────────────────────────

    def _start_keep_alive(self) -> None:
        """Start background thread that pings GoPro every few seconds."""
        if self._ka_thread and self._ka_thread.is_alive():
            return
        self._ka_stop_event.clear()
        self._ka_thread = threading.Thread(
            target=self._keep_alive_loop,
            name="gopro-keep-alive",
            daemon=True,
        )
        self._ka_thread.start()
        logger.debug("GoPro keep-alive thread started.")

    def _stop_keep_alive(self) -> None:
        """Signal the keep-alive thread to stop and wait for it."""
        self._ka_stop_event.set()
        if self._ka_thread:
            self._ka_thread.join(timeout=3.0)
        logger.debug("GoPro keep-alive thread stopped.")

    def _keep_alive_loop(self) -> None:
        """Background loop: POST /gopro/camera/keep_alive at configured interval."""
        interval = _s.GOPRO_KEEP_ALIVE_INTERVAL_S
        while not self._ka_stop_event.wait(timeout=interval):
            self._post("/gopro/camera/keep_alive")

    # ──────────────────────────────────────────────────────────────────────────
    # HTTP helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _get(self, path: str) -> dict[str, Any] | None:
        """
        Perform a GET request and return the JSON response body.

        Args:
            path: API path, e.g. "/gopro/camera/info".

        Returns:
            Parsed JSON dict, or None on any error.
        """
        url = self._base_url + path
        try:
            resp = self._session.get(url, timeout=_TIMEOUT)
            resp.raise_for_status()
            self._connected = True
            return resp.json()
        except requests.exceptions.ConnectionError:
            self._connected = False
            logger.warning("GoPro unreachable at %s", url)
        except requests.exceptions.Timeout:
            self._connected = False
            logger.warning("GoPro request timed out: GET %s", path)
        except requests.exceptions.HTTPError as exc:
            logger.warning("GoPro HTTP error: GET %s → %s", path, exc)
        except ValueError as exc:
            logger.warning("GoPro bad JSON on GET %s: %s", path, exc)
        return None

    def _post(self, path: str, json_body: dict | None = None) -> bool:
        """
        Perform a POST request.

        Args:
            path:      API path.
            json_body: Optional JSON body dict.

        Returns:
            True if the request returned 2xx, False otherwise.
        """
        url = self._base_url + path
        try:
            resp = self._session.post(url, json=json_body, timeout=_TIMEOUT)
            resp.raise_for_status()
            self._connected = True
            return True
        except requests.exceptions.ConnectionError:
            self._connected = False
            logger.warning("GoPro unreachable at %s", url)
        except requests.exceptions.Timeout:
            self._connected = False
            logger.warning("GoPro request timed out: POST %s", path)
        except requests.exceptions.HTTPError as exc:
            logger.warning("GoPro HTTP error: POST %s → %s", path, exc)
        return False
