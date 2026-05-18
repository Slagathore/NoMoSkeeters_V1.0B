"""Step 11: LaserDotDetector — bright-blob laser-dot finder."""
from __future__ import annotations

import cv2
import numpy as np

from targeting.dot_detector import LaserDotDetector


def _frame_with_dot(*dots, size=240) -> np.ndarray:
    img = np.full((size, size, 3), 20, dtype=np.uint8)   # dim room
    for (cx, cy) in dots:
        cv2.circle(img, (cx, cy), 5, (255, 255, 255), -1)
    return img


def test_detects_single_dot():
    det = LaserDotDetector()
    obs = det.detect(_frame_with_dot((120, 96)))
    assert obs is not None
    assert abs(obs.x_px - 120) <= 2 and abs(obs.y_px - 96) <= 2
    assert obs.x_norm == obs.x_px / 239
    assert obs.confidence == 1.0
    assert obs.peak_brightness >= 200


def test_none_on_blank_frame():
    det = LaserDotDetector()
    blank = np.full((240, 240, 3), 20, dtype=np.uint8)
    assert det.detect(blank) is None
    assert det.detect(None) is None


def test_confidence_drops_with_multiple_blobs():
    det = LaserDotDetector()
    obs = det.detect(_frame_with_dot((60, 60), (180, 180)))
    assert obs is not None
    assert obs.confidence == 0.5          # two bright blobs → ambiguous


def test_picks_largest_blob():
    det = LaserDotDetector()
    img = np.full((240, 240, 3), 20, dtype=np.uint8)
    cv2.circle(img, (60, 60), 2, (255, 255, 255), -1)     # small
    cv2.circle(img, (180, 180), 9, (255, 255, 255), -1)   # large
    obs = det.detect(img)
    assert abs(obs.x_px - 180) <= 3 and abs(obs.y_px - 180) <= 3


# ── detect_diff — temporal differencing ──────────────────────────────────

def test_detect_diff_finds_the_dot_against_a_bright_scene():
    det = LaserDotDetector()
    # A scene full of bright objects (windows/screens) that plain detect()
    # would lock onto — present in BOTH frames, so the diff cancels them.
    bg = np.full((240, 240, 3), 20, dtype=np.uint8)
    cv2.rectangle(bg, (0, 40), (30, 200), (255, 255, 255), -1)   # bright wall
    cv2.circle(bg, (200, 50), 25, (240, 240, 240), -1)           # bright lamp
    lit = bg.copy()
    cv2.circle(lit, (130, 110), 5, (210, 160, 90), -1)           # the laser dot

    obs = det.detect_diff(lit, bg)
    assert obs is not None
    assert abs(obs.x_px - 130) <= 3 and abs(obs.y_px - 110) <= 3
    assert obs.confidence == 1.0          # only the dot survives the diff


def test_detect_diff_none_when_nothing_changed():
    det = LaserDotDetector()
    bg = _frame_with_dot((100, 100))
    assert det.detect_diff(bg, bg) is None          # identical → empty diff
