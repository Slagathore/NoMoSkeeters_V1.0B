"""LaserCubeTransport ABC.

Live UDP and dry-run implementations both satisfy this interface so the
rest of the system (heartbeat, calibration controller, laser manager)
doesn't care which one is wired in. That lets the whole bus run offline.

Reference: BOOTSTRAP_AMENDMENTS.md §17 step 3.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

from laser.types import BufferEstimate, LaserInfo, LaserPoint


class LaserCubeTransport(ABC):
    """Interface every cube transport must satisfy.

    Implementations:
      - DryRunTransport — no UDP, returns canned successes (laser/transports/dry_run.py)
      - LaserCubeInterface — live UDP (laser/lasercube.py, Step 5)
    """

    # ── Lifecycle ────────────────────────────────────────────────────────

    @abstractmethod
    def connect(self) -> bool: ...

    @abstractmethod
    def disconnect(self) -> None: ...

    @abstractmethod
    def is_connected(self) -> bool: ...

    # ── Read-only commands ───────────────────────────────────────────────

    @abstractmethod
    def get_full_info(self) -> Optional[LaserInfo]: ...

    @abstractmethod
    def get_ringbuf_empty(self) -> Optional[int]: ...

    # ── State-modifying commands ─────────────────────────────────────────

    @abstractmethod
    def enable_output(self) -> bool: ...

    @abstractmethod
    def disable_output(self) -> bool: ...

    @abstractmethod
    def set_dac_rate(self, rate: int) -> bool: ...

    @abstractmethod
    def clear_ringbuffer(self) -> bool: ...

    # ── Sample data streaming ────────────────────────────────────────────

    @abstractmethod
    def send_chunk(self,
                   points: list[LaserPoint],
                   msg_num: int,
                   frame_num: int) -> bool:
        """Send ONE UDP packet of up to MAX_SAMPLES_PER_PACKET points.
        Returns True if the packet was emitted (or accepted by the dry-run)."""

    def send_frame(self,
                   points: list[LaserPoint],
                   frame_num: int = 0) -> bool:
        """Split a long point list across UDP packets and send. Default
        implementation drives `send_chunk` repeatedly; live transports may
        override for tighter sequence-number control."""
        from laser.protocol import MAX_SAMPLES_PER_PACKET
        if not points:
            return True
        msg_num = self._next_msg_num()
        ok = True
        i = 0
        while i < len(points):
            chunk = points[i:i + MAX_SAMPLES_PER_PACKET]
            if not self.send_chunk(chunk, msg_num, frame_num):
                ok = False
                break
            msg_num = (msg_num + 1) & 0xFF
            i += MAX_SAMPLES_PER_PACKET
        return ok

    @abstractmethod
    def _next_msg_num(self) -> int:
        """Return the next msg_num to use, advancing internal counter."""

    def next_msg_num(self) -> int:
        """Public sequence API for external streamers (e.g.
        targeting.patterns.stream_dragline) that drive send_chunk()
        directly. Allocating through here keeps their packets in the same
        msg_num sequence the transport's own send_frame() uses — never
        reach for the underscore method from outside the class."""
        return self._next_msg_num()

    # ── State surface ────────────────────────────────────────────────────

    @abstractmethod
    def buffer_estimate(self) -> BufferEstimate: ...
