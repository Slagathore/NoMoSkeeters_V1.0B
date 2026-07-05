"""Step 7: Detector — MOG2 pipeline on synthetic frames, classifier toggle."""
from __future__ import annotations

import numpy as np

from detection.classifier import DetectionClassifier
from detection.detector import Detector
from events.schemas import DetectionEvent


def _background(seed: int = 0) -> np.ndarray:
    """A static dim frame with a little sensor noise."""
    rng = np.random.default_rng(seed)
    return (rng.integers(25, 35, size=(240, 320, 3))).astype(np.uint8)


def _frame_with_blob(side: int = 16, cx: int = 160, cy: int = 120) -> np.ndarray:
    frame = _background(seed=99)
    half = side // 2
    frame[cy - half:cy + half, cx - half:cx + half] = 255
    return frame


def _warm_up(detector: Detector, n: int = 15) -> None:
    """Feed background frames so MOG2 builds its model."""
    for i in range(n):
        detector.detect_bgsub(_background(seed=i))


# ── Pipeline ─────────────────────────────────────────────────────────────

def test_detector_finds_a_moving_blob():
    det = Detector()
    _warm_up(det)
    detections = det.detect_bgsub(_frame_with_blob(), sensor_id="gopro")
    assert len(detections) >= 1
    d = detections[0]
    assert isinstance(d, DetectionEvent)
    # The blob was centered ~(160,120) → norm ~(0.5, 0.5).
    assert abs(d.x_norm - 0.5) < 0.1
    assert abs(d.y_norm - 0.5) < 0.1
    assert d.area_pixels > 0


def test_detector_quiet_on_static_background():
    det = Detector()
    _warm_up(det)
    assert det.detect_bgsub(_background(seed=123)) == []


def test_detection_event_fields_populated():
    det = Detector()
    _warm_up(det)
    d = det.detect_bgsub(_frame_with_blob(), sensor_id="gopro",
                         timestamp=42.0)[0]
    assert d.sensor_id == "gopro"
    assert d.timestamp == 42.0
    assert len(d.bbox) == 4
    assert d.detection_id == 0


# ── Classifier toggle (§6.1 label/conf fix) ──────────────────────────────

def test_no_classifier_labels_candidate():
    det = Detector(classifier=None)
    _warm_up(det)
    d = det.detect_bgsub(_frame_with_blob())[0]
    # The §6.1 fix: label/conf are valid even with no classifier.
    assert d.classifier_label == "candidate"
    assert d.classifier_confidence == 0.5


def test_with_classifier_runs_classification():
    det = Detector(classifier=DetectionClassifier(model_path=""))
    _warm_up(det)
    dets = det.detect_bgsub(_frame_with_blob())
    # A compact 16×16 blob clears the heuristic → labeled mosquito, kept.
    assert len(dets) >= 1
    assert dets[0].classifier_label == "mosquito"


def test_world_fn_populates_world_fields():
    det = Detector()
    _warm_up(det)
    d = det.detect_bgsub(_frame_with_blob(), sensor_id="kinect_v2",
                         world_fn=lambda xn, yn: (0.1, -0.2, 2.5))[0]
    assert d.z_world_m == 2.5
    assert d.x_world_m == 0.1
    assert d.y_world_m == -0.2


def test_world_fn_none_result_stays_2d():
    det = Detector()
    _warm_up(det)
    d = det.detect_bgsub(_frame_with_blob(), sensor_id="kinect_v2",
                         world_fn=lambda xn, yn: None)[0]
    assert d.z_world_m is None and d.x_world_m is None
