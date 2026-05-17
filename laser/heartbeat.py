"""Independent daemon thread that pings the cube on a fixed cadence.

The cube's 4-second comms timeout means a paused main thread (debugger
breakpoint, Qt event loop hiccup, GC stutter) can lose the cube. This
thread survives those because the OS scheduler still runs it.

Reference: BOOTSTRAP_AMENDMENTS.md §9.10.
"""
from __future__ import annotations

import logging
import threading
from typing import Callable, Optional

from laser.transport import LaserCubeTransport
from laser.types import LaserInfo


_log = logging.getLogger(__name__)


StatusCallback = Callable[[LaserInfo], None]


class CubeHeartbeat:
    """Pings GET_FULL_INFO at a fixed interval. Never raises."""

    def __init__(self,
                 transport: LaserCubeTransport,
                 interval_s: float = 1.5,
                 on_status: Optional[StatusCallback] = None,
                 name: str = "CubeHeartbeat"):
        self._transport = transport
        self._interval_s = interval_s
        self._on_status = on_status
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name=name, daemon=True)
        self._tick_count = 0
        self._last_info: Optional[LaserInfo] = None
        self._lock = threading.Lock()

    def start(self) -> None:
        if not self._thread.is_alive():
            self._thread.start()

    def stop(self, timeout_s: float = 2.0) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=timeout_s)

    @property
    def tick_count(self) -> int:
        with self._lock:
            return self._tick_count

    @property
    def last_info(self) -> Optional[LaserInfo]:
        with self._lock:
            return self._last_info

    def _run(self) -> None:
        # Wait first so callers have a chance to settle before the first ping.
        while not self._stop.wait(self._interval_s):
            try:
                info = self._transport.get_full_info()
            except Exception:
                _log.exception("heartbeat get_full_info raised; continuing")
                continue
            if info is None:
                continue
            with self._lock:
                self._last_info = info
                self._tick_count += 1
            if self._on_status is not None:
                try:
                    self._on_status(info)
                except Exception:
                    _log.exception("heartbeat status callback raised; suppressing")
