"""Step 7: DetectionClassifier — features, heuristic, fail-mode, toggles."""
from __future__ import annotations

import numpy as np
import pytest

from config import settings
from detection.classifier import DetectionClassifier


FRAME = np.zeros((240, 320, 3), dtype=np.uint8)


def _square(side: int, x0: int = 50, y0: int = 50) -> np.ndarray:
    """A solid axis-aligned square contour."""
    return np.array([[[x0, y0]], [[x0 + side, y0]],
                     [[x0 + side, y0 + side]], [[x0, y0 + side]]],
                    dtype=np.int32)


def _rect(w: int, h: int) -> np.ndarray:
    return np.array([[[10, 10]], [[10 + w, 10]],
                     [[10 + w, 10 + h]], [[10, 10 + h]]], dtype=np.int32)


# ── Feature extraction ───────────────────────────────────────────────────

def test_extract_features_has_full_vector():
    feats = DetectionClassifier._extract_features(FRAME, _square(20))
    for name in ("aspect_ratio", "solidity", "extent", "equiv_diameter",
                 "perimeter_squared", "area_frame_ratio",
                 "hu_1", "hu_2", "hu_3", "hu_4"):
        assert name in feats
    assert feats["aspect_ratio"] == pytest.approx(1.0)
    assert feats["solidity"] == pytest.approx(1.0, abs=0.05)


def test_extract_features_rejects_degenerate_contour():
    degenerate = np.array([[[5, 5]], [[5, 5]], [[5, 5]]], dtype=np.int32)
    with pytest.raises(ValueError):
        DetectionClassifier._extract_features(FRAME, degenerate)


# ── Heuristic classification ─────────────────────────────────────────────

def test_heuristic_accepts_compact_blob():
    clf = DetectionClassifier(model_path="")
    label, conf = clf.classify(FRAME, _square(20))
    assert label == "mosquito"
    assert conf > 0.5


def test_heuristic_rejects_too_small():
    clf = DetectionClassifier(model_path="")
    label, _ = clf.classify(FRAME, _square(3))      # area 9 < min_area_px 20
    assert label == "false_positive"


def test_heuristic_rejects_too_elongated():
    clf = DetectionClassifier(model_path="")
    label, _ = clf.classify(FRAME, _rect(120, 4))   # aspect 30 > max_aspect 5
    assert label == "false_positive"


# ── Fail-mode ────────────────────────────────────────────────────────────

@pytest.mark.parametrize("mode,expected", [
    ("open", "mosquito"),
    ("closed", "false_positive"),
    ("neutral", "candidate"),
])
def test_fail_mode_on_degenerate_contour(monkeypatch, mode, expected):
    monkeypatch.setattr(settings, "CLASSIFIER_FAIL_MODE", mode)
    clf = DetectionClassifier(model_path="")
    degenerate = np.array([[[5, 5]], [[5, 5]]], dtype=np.int32)
    label, conf = clf.classify(FRAME, degenerate)
    assert label == expected
    assert conf == 0.5


# ── Feature toggles (§6.2 — Hu opt-in) ───────────────────────────────────

def test_hu_moments_off_by_default():
    assert settings.CLASSIFIER_USE_HU_MOMENTS is False
    clf = DetectionClassifier(model_path="")
    assert "hu_1" not in clf._enabled_features()


def test_enabled_features_follows_toggles(monkeypatch):
    monkeypatch.setattr(settings, "CLASSIFIER_USE_HU_MOMENTS", True)
    clf = DetectionClassifier(model_path="")
    enabled = clf._enabled_features()
    assert all(f"hu_{i}" in enabled for i in (1, 2, 3, 4))
    feats = clf._extract_features(FRAME, _square(20))
    assert clf.feature_vector(feats).shape[0] == len(enabled)


# ── ML mode ──────────────────────────────────────────────────────────────

class _FakeModel:
    """Duck-typed sklearn estimator."""
    def predict(self, X):
        return ["mosquito"] * len(X)

    def predict_proba(self, X):
        return np.array([[0.12, 0.88]] * len(X))


def test_ml_mode_routes_through_model():
    clf = DetectionClassifier(model_path="")
    clf._model = _FakeModel()
    assert clf.mode == "ml"
    label, conf = clf.classify(FRAME, _square(20))
    assert label == "mosquito"
    assert conf == pytest.approx(0.88)
