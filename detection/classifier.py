"""DetectionClassifier — shape-feature classification of contour candidates.

Two modes: a heuristic (hand-tuned thresholds, always available) and an
optional ML model (RandomForest pickle). classify() never raises in the hot
path — on any failure it returns a configurable fail-mode result.

Hu moments are demoted to opt-in per the §6.2 amendment: on the 2–5 px blobs
typical at mosquito range they are dominated by quantization noise. Each
feature is independently toggleable so the ML vector matches its training.

Reference: BOOTSTRAP.md §6.2, BOOTSTRAP_AMENDMENTS.md §6.2.
"""
from __future__ import annotations

import logging
import math
import pickle
from typing import Optional

import cv2
import numpy as np

from config import settings

_log = logging.getLogger(__name__)

# Fixed feature order. The ML vector is this order, filtered by the toggles;
# a model MUST be trained with the same toggle set it is scored with.
FEATURE_ORDER = (
    "aspect_ratio", "solidity", "extent", "equiv_diameter",
    "perimeter_squared", "area_frame_ratio",
    "hu_1", "hu_2", "hu_3", "hu_4",
)


def _fail_result() -> tuple[str, float]:
    """Map CLASSIFIER_FAIL_MODE to a (label, confidence) pair."""
    mode = settings.CLASSIFIER_FAIL_MODE
    if mode == "open":
        return ("mosquito", 0.5)        # let it through; operator/track decides
    if mode == "closed":
        return ("false_positive", 0.5)  # drop it
    return ("candidate", 0.5)           # "neutral" — neither accept nor reject


class DetectionClassifier:
    """Classifies a contour as 'mosquito' / 'false_positive' / 'candidate'."""

    def __init__(self,
                 model_path: Optional[str] = None,
                 heuristic_thresholds: Optional[dict] = None):
        self._model = self._try_load_model(
            model_path if model_path is not None
            else settings.CLASSIFIER_MODEL_PATH)
        self._thresholds = dict(heuristic_thresholds
                                or settings.CLASSIFIER_HEURISTIC_DEFAULTS)

    @property
    def mode(self) -> str:
        return "ml" if self._model is not None else "heuristic"

    # ── Public API ───────────────────────────────────────────────────────

    def classify(self, frame: np.ndarray,
                 contour: np.ndarray) -> tuple[str, float]:
        """Return (label, confidence). Never raises in the hot path."""
        try:
            features = self._extract_features(frame, contour)
            if self._model is not None:
                return self._classify_ml(features)
            return self._classify_heuristic(features)
        except Exception:
            _log.debug("classify() failed; applying fail-mode %s",
                       settings.CLASSIFIER_FAIL_MODE, exc_info=True)
            return _fail_result()

    # ── Feature extraction ───────────────────────────────────────────────

    @staticmethod
    def _extract_features(frame: np.ndarray,
                          contour: np.ndarray) -> dict[str, float]:
        """The 10-feature shape vector (§6.2). Raises on a degenerate
        contour — classify() catches that and applies the fail-mode."""
        area = float(cv2.contourArea(contour))
        if area <= 0.0:
            raise ValueError("degenerate contour (zero area)")
        x, y, w, h = cv2.boundingRect(contour)
        perimeter = float(cv2.arcLength(contour, True))
        hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
        frame_h, frame_w = frame.shape[:2]

        hu = cv2.HuMoments(cv2.moments(contour)).flatten()
        # Log-scale Hu moments; they span many orders of magnitude.
        hu_log = [-math.copysign(1.0, v) * math.log10(abs(v))
                  if v != 0.0 else 0.0 for v in hu[:4]]

        return {
            "aspect_ratio": w / h if h else 0.0,
            "solidity": area / hull_area if hull_area else 0.0,
            "extent": area / (w * h) if (w and h) else 0.0,
            "equiv_diameter": math.sqrt(4.0 * area / math.pi),
            "perimeter_squared": perimeter * perimeter,
            "area_frame_ratio": area / (frame_w * frame_h)
            if (frame_w and frame_h) else 0.0,
            "hu_1": hu_log[0], "hu_2": hu_log[1],
            "hu_3": hu_log[2], "hu_4": hu_log[3],
            "_area_px": area,   # carried for the heuristic, not a vector feature
        }

    @staticmethod
    def _enabled_features() -> tuple[str, ...]:
        """The feature names active under the current config toggles."""
        flags = {
            "aspect_ratio": settings.CLASSIFIER_USE_ASPECT,
            "solidity": settings.CLASSIFIER_USE_SOLIDITY,
            "extent": settings.CLASSIFIER_USE_EXTENT,
            "equiv_diameter": settings.CLASSIFIER_USE_EQUIV_DIAM,
            "perimeter_squared": settings.CLASSIFIER_USE_PERIM_SQ,
            "area_frame_ratio": settings.CLASSIFIER_USE_AREA_RATIO,
            "hu_1": settings.CLASSIFIER_USE_HU_MOMENTS,
            "hu_2": settings.CLASSIFIER_USE_HU_MOMENTS,
            "hu_3": settings.CLASSIFIER_USE_HU_MOMENTS,
            "hu_4": settings.CLASSIFIER_USE_HU_MOMENTS,
        }
        return tuple(name for name in FEATURE_ORDER if flags[name])

    def feature_vector(self, features: dict[str, float]) -> np.ndarray:
        """Build the ML input vector from a feature dict, honoring toggles."""
        return np.array([features[name] for name in self._enabled_features()],
                        dtype=float)

    # ── Classification modes ─────────────────────────────────────────────

    def _classify_heuristic(self, features: dict[str, float]) -> tuple[str, float]:
        """Hand-tuned threshold gate. Each check that passes adds confidence;
        any hard fail returns 'false_positive'."""
        t = self._thresholds
        area = features["_area_px"]
        aspect = features["aspect_ratio"]
        solidity = features["solidity"]

        if not (t["min_area_px"] <= area <= t["max_area_px"]):
            return ("false_positive", 0.8)
        if not (t["min_aspect"] <= aspect <= t["max_aspect"]):
            return ("false_positive", 0.7)
        if solidity < t["min_solidity"]:
            return ("false_positive", 0.6)
        # Passed every gate — confidence scales with how compact the blob is.
        conf = min(0.95, 0.6 + 0.35 * solidity)
        return ("mosquito", conf)

    def _classify_ml(self, features: dict[str, float]) -> tuple[str, float]:
        """Score with the loaded model. Returns its label + class probability."""
        vec = self.feature_vector(features).reshape(1, -1)
        label = str(self._model.predict(vec)[0])
        conf = 0.5
        if hasattr(self._model, "predict_proba"):
            conf = float(np.max(self._model.predict_proba(vec)[0]))
        return (label, conf)

    # ── Model loading ────────────────────────────────────────────────────

    @staticmethod
    def _try_load_model(model_path: Optional[str]):
        """Load the RandomForest pickle if present; None → heuristic mode."""
        if not model_path:
            return None
        from pathlib import Path
        path = Path(model_path)
        if not path.is_absolute():
            path = settings.BASE_DIR / path
        if not path.exists():
            _log.info("no classifier model at %s — using heuristic mode", path)
            return None
        try:
            with open(path, "rb") as fh:
                model = pickle.load(fh)
            _log.info("loaded classifier model: %s", path)
            return model
        except Exception:
            _log.exception("classifier model load failed — heuristic fallback")
            return None
