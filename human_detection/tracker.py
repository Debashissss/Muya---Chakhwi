"""
tracker.py — Lightweight IoU-based multi-object tracker.

No external library needed. Works frame-to-frame to:
  • assign stable track IDs to repeated detections
  • count confirmation frames (track_age)
  • suppress duplicate alerts for the same victim

Replace with DeepSORT / BotSORT for production if needed.
"""
from dataclasses import dataclass, field
from typing import List, Optional
import numpy as np


@dataclass
class Track:
    track_id:   int
    bbox:       np.ndarray     # [x1, y1, x2, y2]
    confidence: float
    age:        int = 0        # frames confirmed
    missed:     int = 0        # consecutive missed frames
    priority:   Optional[object] = None


def iou(a: np.ndarray, b: np.ndarray) -> float:
    """Compute IoU between two boxes [x1,y1,x2,y2]."""
    ix1 = max(a[0], b[0]); iy1 = max(a[1], b[1])
    ix2 = min(a[2], b[2]); iy2 = min(a[3], b[3])
    inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
    if inter == 0:
        return 0.0
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[2] - b[0]) * (b[3] - b[1])
    return inter / (area_a + area_b - inter)


class IoUTracker:
    """
    Greedy IoU tracker.
      - iou_thresh: minimum IoU to match a detection to an existing track.
      - max_missed: frames without a match before a track is removed.
    """

    def __init__(self, iou_thresh: float = 0.35, max_missed: int = 10):
        self.iou_thresh  = iou_thresh
        self.max_missed  = max_missed
        self._tracks: List[Track] = []
        self._next_id  = 1

    def update(self, detections: List[dict]) -> List[dict]:
        """
        Args:
            detections: list of dicts with keys
                        'bbox' ([x1,y1,x2,y2]), 'confidence'
        Returns:
            Same list enriched with 'track_id' and 'track_age'.
        """
        if not detections:
            for t in self._tracks:
                t.missed += 1
            self._prune()
            return []

        det_boxes = [np.array(d["bbox"], dtype=float) for d in detections]
        matched_track_ids = set()
        result = []

        for i, det in enumerate(detections):
            best_iou   = self.iou_thresh
            best_track = None
            for t in self._tracks:
                if t.track_id in matched_track_ids:
                    continue
                sc = iou(det_boxes[i], t.bbox)
                if sc > best_iou:
                    best_iou   = sc
                    best_track = t

            if best_track is not None:
                best_track.bbox       = det_boxes[i]
                best_track.confidence = det["confidence"]
                best_track.age       += 1
                best_track.missed     = 0
                matched_track_ids.add(best_track.track_id)
                det = {**det, "track_id": best_track.track_id,
                       "track_age": best_track.age}
            else:
                new_track = Track(
                    track_id=self._next_id,
                    bbox=det_boxes[i],
                    confidence=det["confidence"],
                )
                self._tracks.append(new_track)
                matched_track_ids.add(self._next_id)
                det = {**det, "track_id": self._next_id, "track_age": 0}
                self._next_id += 1

            result.append(det)

        # Increment missed for unmatched tracks
        for t in self._tracks:
            if t.track_id not in matched_track_ids:
                t.missed += 1

        self._prune()
        return result

    def _prune(self) -> None:
        self._tracks = [t for t in self._tracks if t.missed <= self.max_missed]
