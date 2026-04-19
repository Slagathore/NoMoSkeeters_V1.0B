"""
Iron Dome Anti-Mosquito System
================================
Detection Package
-----------------
Computer vision pipeline for finding and tracking mosquito-sized objects.

Modules:
  detector.py  — Frame-level detection using background subtraction + YOLO.
  tracker.py   — Multi-object Kalman-filter tracker with Hungarian assignment.

Classes (re-exported):
  Detection        — NamedTuple describing a single detection result.
  MosquitoDetector — QThread-based detection worker.
  TrackedTarget    — Dataclass for a live tracked target with velocity/prediction.
  MosquitoTracker  — Stateful multi-object tracker.

#todo Add a "detection confidence heatmap" that accumulates where mosquitos
      are most often detected to guide zone/placement recommendations.
"""

from .detector import Detection, MosquitoDetector
from .tracker import MosquitoTracker, TrackedTarget

__all__ = [
    "Detection",
    "MosquitoDetector",
    "TrackedTarget",
    "MosquitoTracker",
]
