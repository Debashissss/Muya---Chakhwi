"""
priority.py — Priority scoring engine for the DRISHYA human detection module.

Scoring factors:
  - RGB detection confidence  (0..1)
  - Thermal confirmation      (0..1, 0 if not available)
  - Victim posture cue        (standing / lying / crouching)
  - Hazard proximity flag     (fire / flood / debris detected in same frame)
  - Track age                 (new vs. confirmed track)

Output bands:
  CRITICAL ≥ 0.80  →  rescue first
  HIGH     ≥ 0.60  →  next wave
  MEDIUM   ≥ 0.40  →  queue
  LOW      <  0.40 →  monitor
"""
from dataclasses import dataclass
from typing import Optional


@dataclass
class PriorityResult:
    score: float           # 0.0 – 1.0
    band: str              # CRITICAL / HIGH / MEDIUM / LOW
    colour: tuple          # BGR for OpenCV annotation
    label: str             # display label
    factors: dict          # breakdown for logging


# Band → BGR colour (OpenCV)
BAND_COLOURS = {
    "CRITICAL": (0,   0,   220),   # Red
    "HIGH":     (0,   128, 255),   # Orange
    "MEDIUM":   (0,   220, 255),   # Yellow
    "LOW":      (0,   200, 80),    # Green
}

# Posture cue weights
POSTURE_WEIGHT = {
    "lying":    0.20,   # likely incapacitated — boost score
    "crouching":0.05,
    "standing": 0.00,
    "unknown":  0.00,
}


def compute_priority(
    rgb_conf: float,
    thermal_conf: float = 0.0,
    posture: str = "unknown",
    hazard_near: bool = False,
    track_age: int = 0,
    bbox_area_ratio: float = 0.0,
) -> PriorityResult:
    """
    Compute a composite priority score and band for a single detection.

    Args:
        rgb_conf        Detection confidence from YOLO (0–1).
        thermal_conf    Thermal overlap confidence (0–1, 0 = no thermal).
        posture         Posture cue: 'lying', 'crouching', 'standing', 'unknown'.
        hazard_near     True if a hazard (fire/water/debris) was detected nearby.
        track_age       Number of frames this track has been confirmed.
        bbox_area_ratio Fraction of frame area occupied (larger = closer / more certain).
    Returns:
        PriorityResult
    """
    # Base: RGB confidence (50 % weight)
    score = rgb_conf * 0.50

    # Thermal confirmation (25 % weight)
    score += thermal_conf * 0.25

    # Posture cue (up to 20 %)
    score += POSTURE_WEIGHT.get(posture, 0.0)

    # Hazard proximity bonus (up to 10 % — victim may be in immediate danger)
    if hazard_near:
        score += 0.10

    # Track age discount (new track = less certain, cap at 5 % bonus)
    age_bonus = min(track_age / 20.0, 0.05)
    score += age_bonus

    # BBox area slight boost (closer victim)
    score += min(bbox_area_ratio * 0.10, 0.05)

    # Clamp
    score = min(max(score, 0.0), 1.0)

    # Band
    if score >= 0.80:
        band = "CRITICAL"
    elif score >= 0.60:
        band = "HIGH"
    elif score >= 0.40:
        band = "MEDIUM"
    else:
        band = "LOW"

    colour = BAND_COLOURS[band]
    label  = f"{band} [{score:.2f}]"
    factors = {
        "rgb_conf":       round(rgb_conf, 3),
        "thermal_conf":   round(thermal_conf, 3),
        "posture":        posture,
        "hazard_near":    hazard_near,
        "track_age":      track_age,
        "bbox_area_ratio":round(bbox_area_ratio, 4),
        "final_score":    round(score, 4),
    }

    return PriorityResult(score=score, band=band, colour=colour,
                          label=label, factors=factors)
