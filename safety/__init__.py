"""Safety subsystem — fire/no-fire gate for the laser output.

The architecture: `LaserManager` streams sample data unconditionally; the
cube's `enable_output()` is the only thing that lets a photon out. Per
§9.11 / Step 12 the safety layer owns that gate. `SafetyModerator` is that
owner — it consumes frames from sensors carrying the SAFETY role and
produces a single boolean `is_safe_to_fire(...)` plus an event stream.

Reference: BOOTSTRAP.md §5 (sensor roles), BOOTSTRAP.md §12 (safety),
laser/laser_manager.py docstring.
"""

from safety.person_detector import (HOGPersonDetector, PersonDetection,
                                     PersonDetector)
from safety.voice_warning import VoiceWarning
from safety.kinect_safety_check import KinectSafetyCheck
from safety.safety_moderator import SafetyModerator, SafetyState, SafetyVerdict

__all__ = [
    "HOGPersonDetector",
    "KinectSafetyCheck",
    "PersonDetection",
    "PersonDetector",
    "SafetyModerator",
    "SafetyState",
    "SafetyVerdict",
    "VoiceWarning",
]
