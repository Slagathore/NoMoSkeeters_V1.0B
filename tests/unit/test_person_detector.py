"""Person detector backends: HOG smoke + FixedRegion stub semantics."""
from __future__ import annotations

import numpy as np

from safety.person_detector import (FixedRegionPersonDetector, HOGPersonDetector,
                                     PersonDetection)


def test_hog_detector_runs_on_a_blank_frame_without_error() -> None:
    # A blank frame should produce zero detections cleanly (the HOG detector
    # raises on weird inputs sometimes — the wrapper must swallow that).
    det = HOGPersonDetector()
    img = np.zeros((128, 64, 3), dtype=np.uint8)
    assert det.detect(img) == []


def test_hog_detector_handles_empty_input() -> None:
    det = HOGPersonDetector()
    assert det.detect(np.zeros((0, 0, 3), dtype=np.uint8)) == []


def test_fixed_region_returns_configured_detections() -> None:
    d = PersonDetection(10, 20, 30, 40, 0.9)
    det = FixedRegionPersonDetector([d])
    out = det.detect(np.zeros((100, 100, 3), dtype=np.uint8))
    assert out == [d]


def test_fixed_region_set_replaces_detections() -> None:
    det = FixedRegionPersonDetector()
    assert det.detect(np.zeros((10, 10, 3), dtype=np.uint8)) == []
    d = PersonDetection(1, 2, 3, 4, 0.5)
    det.set([d])
    assert det.detect(np.zeros((10, 10, 3), dtype=np.uint8)) == [d]


def test_person_detection_geometry() -> None:
    d = PersonDetection(10, 20, 30, 40, 0.5)
    assert d.center_px == (25, 40)
    assert d.area_px2 == 1200
    cx, cy = d.center_norm(100, 100)
    assert 0.0 <= cx <= 1.0 and 0.0 <= cy <= 1.0
