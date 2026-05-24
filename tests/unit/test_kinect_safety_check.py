"""KinectSafetyCheck: depth-blob extraction + human-sized classification."""
from __future__ import annotations

import numpy as np

from safety.kinect_safety_check import KinectSafetyCheck


def _make_depth(shape=(424, 512), background=0.0,
                blob=None) -> np.ndarray:
    """Synthetic depth frame in metres. `blob` is (x, y, w, h, depth_m)."""
    d = np.full(shape, background, dtype=np.float32)
    if blob is not None:
        x, y, w, h, depth = blob
        d[y:y + h, x:x + w] = depth
    return d


def test_empty_frame_has_no_blobs() -> None:
    chk = KinectSafetyCheck()
    assert chk.analyze(_make_depth()) == []
    assert chk.is_person_present(_make_depth()) is False


def test_human_sized_blob_at_2m_flags_human() -> None:
    # Person-shaped silhouette: ~1.7 m tall, narrower than tall.
    # At depth 2m with fy=365.456, 1.7 m → ~310 px tall, ~80 px wide → person.
    depth_m = 2.0
    h_px = int(1.7 * 365.456 / depth_m)
    w_px = int(0.5 * 365.456 / depth_m)
    img = _make_depth(blob=(100, 50, w_px, h_px, depth_m))
    chk = KinectSafetyCheck()
    blobs = chk.analyze(img)
    assert len(blobs) == 1
    assert blobs[0].is_human_sized
    assert chk.is_person_present(img)


def test_short_squat_blob_is_not_human_sized() -> None:
    # 0.4 m tall, 1 m wide — a couch, not a person.
    depth_m = 2.0
    h_px = int(0.4 * 365.456 / depth_m)
    w_px = int(1.0 * 365.456 / depth_m)
    img = _make_depth(blob=(50, 200, w_px, h_px, depth_m))
    chk = KinectSafetyCheck()
    blobs = chk.analyze(img)
    assert len(blobs) == 1
    assert not blobs[0].is_human_sized
    assert not chk.is_person_present(img)


def test_depth_outside_working_range_is_ignored() -> None:
    chk = KinectSafetyCheck(min_depth_m=0.5, max_depth_m=4.5)
    # Blob at 6m → outside range → invisible to the check.
    img = _make_depth(blob=(0, 0, 200, 400, 6.0))
    assert chk.analyze(img) == []


def test_zero_depth_pixels_are_treated_as_no_data() -> None:
    # The Kinect emits 0 for "no return". They must not be counted as blobs.
    chk = KinectSafetyCheck()
    img = _make_depth(background=0.0)
    assert chk.analyze(img) == []


def test_person_blobs_filters_out_non_human() -> None:
    chk = KinectSafetyCheck()
    depth_m = 2.0
    person = _make_depth(blob=(50, 50,
                               int(0.4 * 365.456 / depth_m),
                               int(1.7 * 365.456 / depth_m), depth_m))
    out = chk.person_blobs(person)
    assert len(out) == 1
