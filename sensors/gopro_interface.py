"""GoProInterface — the Open GoPro HTTP control client for GoProSensor.

Implements the GoProControl protocol: start the camera streaming, keep it
alive, stop it, poll status. GoProSensor handles the video decode; this
handles camera control so the two stay separable and testable.

Two streaming modes (set via `mode=`):

  * "webcam"  (default) — GET /gopro/webcam/start, served as MPEG-TS over
    UDP 8554. **This is what works on the Hero 13** (verified hardware
    session 2026-05-17, HARDWARE_FINDINGS.md). The feed is H.264 1080p30.
  * "preview" — GET /gopro/camera/stream/start. This is really a WiFi-AP
    feature and did not deliver UDP over USB on the Hero 13; kept only as
    a fallback for other models.

Webcam mode needs no client keep-alive. The legacy `_GPHD_` datagram is
kept for preview mode only.
"""
from __future__ import annotations

import json
import logging
import socket
import time
import urllib.request
from typing import Any, Callable, Optional

from events.schemas import GoProStatusEvent

_log = logging.getLogger(__name__)

# Camera defaults in WiFi-AP mode (GoPro is the access point). USB-tethered
# operation uses 172.X.Y.51 instead — pass `ip=` per session (see
# HARDWARE_FINDINGS.md §2.1 / tools/gopro_stream_helper.ps1 Find-GoPro).
_DEFAULT_IP = "10.5.5.9"
_HTTP_PORT = 8080
_STREAM_PORT = 8554

# Documented `_GPHD_` UDP stream keep-alive: format "_GPHD_:%u:%u:%d:%1lf\n"
# with (0, 0, 2, 0) — command 2 is "keep alive".
_KEEPALIVE_PACKET = ("_GPHD_:%u:%u:%d:%1lf\n" % (0, 0, 2, 0)).encode("ascii")

# /gopro/camera/state `status` block field codes (HARDWARE_FINDINGS.md §2.4).
# The state JSON keys these as strings.
_STATUS_FIELDS = {
    "system_hot":            6,
    "system_busy":           8,
    "preview_stream_active": 32,
    "battery_percent":       70,
    "thermal_throttle":      86,
}


def _default_http_get(url: str, timeout_s: float) -> bool:
    """GET a URL; True on a 2xx response. Open GoPro returns 200 + `{}`."""
    try:
        with urllib.request.urlopen(url, timeout=timeout_s) as resp:
            return 200 <= resp.status < 300
    except Exception as exc:
        _log.error("gopro http GET failed: %s (%s)", url, exc)
        return False


def _default_http_get_json(url: str, timeout_s: float) -> Optional[Any]:
    """GET a URL and parse the JSON body; None on any failure."""
    try:
        with urllib.request.urlopen(url, timeout=timeout_s) as resp:
            if not (200 <= resp.status < 300):
                return None
            return json.loads(resp.read().decode("utf-8"))
    except Exception as exc:
        _log.error("gopro http GET (json) failed: %s (%s)", url, exc)
        return None


class GoProInterface:
    """Open GoPro HTTP control for the Hero 13 preview stream."""

    def __init__(self,
                 ip: str = _DEFAULT_IP,
                 http_port: int = _HTTP_PORT,
                 stream_port: int = _STREAM_PORT,
                 http_timeout_s: float = 5.0,
                 mode: str = "webcam",
                 webcam_fov: Optional[int] = None,
                 *,
                 http_get: Optional[Callable[[str, float], bool]] = None,
                 http_get_json: Optional[Callable[[str, float], Optional[Any]]] = None,
                 socket_factory: Optional[Callable[[], object]] = None):
        self._ip = ip
        self._http_port = http_port
        self._stream_port = stream_port
        self._timeout = http_timeout_s
        self._mode = mode
        # Webcam FOV code: 0=Wide (fisheye, camera default), 4=Linear
        # (de-warped), 3=Superview. None → don't send the param. Note some
        # firmware ignores it (OpenGoPro issue #459) — verify on the bench.
        self._webcam_fov = webcam_fov
        self._http_get = http_get or _default_http_get
        self._http_get_json = http_get_json or _default_http_get_json
        self._socket_factory = socket_factory or (
            lambda: socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
        self._ka_sock: Optional[object] = None

    @property
    def camera_ip(self) -> str:
        """The camera's HTTP/stream IP — FfmpegStreamCapture binds against it."""
        return self._ip

    # ── URL helper ───────────────────────────────────────────────────────

    def _url(self, path: str) -> str:
        return f"http://{self._ip}:{self._http_port}{path}"

    # ── GoProControl protocol ────────────────────────────────────────────

    def start_preview(self) -> bool:
        """Start the camera streaming MPEG-TS over UDP 8554.

        webcam mode → /gopro/webcam/start ; preview mode → stream/start.
        """
        if self._mode != "webcam":
            ok = self._http_get(self._url("/gopro/camera/stream/start"),
                                self._timeout)
        else:
            ok = False
            if self._webcam_fov is not None:
                # Try the fov query param; some firmware (e.g. Hero 13 H24)
                # rejects it — fall back to a plain start so the feed still
                # comes up (OpenGoPro issue #459).
                ok = self._http_get(
                    self._url(f"/gopro/webcam/start?fov={self._webcam_fov}"),
                    self._timeout)
                if not ok:
                    _log.warning("gopro: webcam fov=%s rejected by this "
                                 "firmware; starting without it",
                                 self._webcam_fov)
            if not ok:
                ok = self._http_get(self._url("/gopro/webcam/start"),
                                    self._timeout)
        if ok:
            _log.info("gopro: %s stream started (udp:%d)",
                      self._mode, self._stream_port)
        return ok

    def stop_preview(self) -> bool:
        """Stop streaming and release the keep-alive socket.

        webcam mode is stopped AND exited so the camera leaves webcam state.
        """
        if self._mode == "webcam":
            ok = self._http_get(self._url("/gopro/webcam/stop"), self._timeout)
            self._http_get(self._url("/gopro/webcam/exit"), self._timeout)
        else:
            ok = self._http_get(self._url("/gopro/camera/stream/stop"),
                                self._timeout)
        if self._ka_sock is not None:
            try:
                self._ka_sock.close()
            except OSError:
                pass
            self._ka_sock = None
        return ok

    def keep_alive(self) -> bool:
        """Keep the stream alive.

        Webcam mode needs no client keep-alive — it runs until stopped — so
        this is a no-op there. Preview mode sends the legacy `_GPHD_` packet.
        """
        if self._mode == "webcam":
            return True
        if self._ka_sock is None:
            self._ka_sock = self._socket_factory()
        try:
            self._ka_sock.sendto(_KEEPALIVE_PACKET,
                                  (self._ip, self._stream_port))
            return True
        except OSError as exc:
            _log.error("gopro: keep-alive sendto failed: %s", exc)
            return False

    def get_state(self) -> Optional[GoProStatusEvent]:
        """Poll GET /gopro/camera/state and parse the §2.4 status fields.

        Returns a populated GoProStatusEvent on success, or None if the camera
        is unreachable / the response is unparseable. The status poller in
        GoProSensor turns a None into a `reachable=False` event for the bus.
        """
        raw = self._http_get_json(self._url("/gopro/camera/state"),
                                  self._timeout)
        if not isinstance(raw, dict):
            return None
        status = raw.get("status")
        if not isinstance(status, dict):
            return None

        def _field(code: int) -> int:
            # state JSON keys status fields as strings; tolerate ints too.
            value = status.get(str(code), status.get(code, 0))
            try:
                return int(value)
            except (TypeError, ValueError):
                return 0

        return GoProStatusEvent(
            timestamp=time.monotonic(),
            reachable=True,
            battery_percent=_field(_STATUS_FIELDS["battery_percent"]),
            system_hot=bool(_field(_STATUS_FIELDS["system_hot"])),
            system_busy=bool(_field(_STATUS_FIELDS["system_busy"])),
            preview_stream_active=bool(
                _field(_STATUS_FIELDS["preview_stream_active"])),
            thermal_throttle=bool(_field(_STATUS_FIELDS["thermal_throttle"])),
        )
