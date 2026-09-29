"""
thermal_fusion.py — RGB + Thermal frame fusion utilities.

Fusion strategy:
  1. Resize thermal to match RGB resolution.
  2. Apply a pseudo-colour map (COLORMAP_INFERNO) to the thermal grey frame.
  3. Blend: fused = α * rgb + (1-α) * thermal_colour
  4. For detection, extract the thermal ROI aligned with a YOLO bbox and
     compute a thermal confidence score (mean temperature percentile).
"""
import numpy as np
import cv2
import logging
from typing import Optional, Tuple, Union

logger = logging.getLogger(__name__)


class ThermalFusion:
    """
    Handles reading a thermal camera and fusing it with RGB frames.
    """

    def __init__(self, thermal_src: Optional[Union[int, str]] = None,
                 alpha: float = 0.55):
        """
        Args:
            thermal_src: OpenCV capture index or RTSP/file path for thermal cam.
            alpha:       Weight of RGB in the blended frame (0..1).
        """
        self.alpha = alpha
        self._cap: Optional[cv2.VideoCapture] = None
        self._last_thermal: Optional[np.ndarray] = None

        if thermal_src is not None:
            self._cap = cv2.VideoCapture(thermal_src)
            if not self._cap.isOpened():
                logger.error("Cannot open thermal source: %s", thermal_src)
                self._cap = None
            else:
                logger.info("Thermal camera opened: %s", thermal_src)

    def read_thermal(self) -> Optional[np.ndarray]:
        """Read one frame from the thermal camera (grey or BGR)."""
        if self._cap is None:
            return None
        ret, frame = self._cap.read()
        if not ret:
            return None
        if len(frame.shape) == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self._last_thermal = frame
        return frame

    def fuse(self, rgb: np.ndarray,
             thermal_grey: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Blend RGB with thermal pseudo-colour. Returns an RGB frame.
        """
        if thermal_grey is None:
            thermal_grey = self._last_thermal
        if thermal_grey is None:
            return rgb

        h, w = rgb.shape[:2]
        t = cv2.resize(thermal_grey, (w, h), interpolation=cv2.INTER_LINEAR)
        # Normalise to 0-255 (handles 14-bit thermal data)
        t_norm = cv2.normalize(t, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        t_colour = cv2.applyColorMap(t_norm, cv2.COLORMAP_INFERNO)
        fused = cv2.addWeighted(rgb, self.alpha, t_colour, 1 - self.alpha, 0)
        return fused

    def thermal_confidence(self, thermal_grey: np.ndarray,
                           bbox: Tuple[int, int, int, int],
                           frame_shape: Tuple[int, int]) -> float:
        """
        Return a confidence score [0..1] based on how 'hot' the
        bounding-box ROI is compared to the whole frame.

        Args:
            thermal_grey: Full thermal frame (HxW uint8 or uint16).
            bbox:         (x1, y1, x2, y2) in pixels (RGB frame coords).
            frame_shape:  (H, W) of the RGB frame.
        Returns:
            Thermal confidence in [0, 1].
        """
        if thermal_grey is None:
            return 0.0
        h_rgb, w_rgb = frame_shape
        h_t,   w_t   = thermal_grey.shape[:2]

        # Scale bbox to thermal resolution
        sx = w_t / w_rgb
        sy = h_t / h_rgb
        x1 = int(bbox[0] * sx); y1 = int(bbox[1] * sy)
        x2 = int(bbox[2] * sx); y2 = int(bbox[3] * sy)
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w_t, x2), min(h_t, y2)

        if x2 <= x1 or y2 <= y1:
            return 0.0

        roi   = thermal_grey[y1:y2, x1:x2].astype(float)
        full  = thermal_grey.astype(float)

        roi_mean  = roi.mean()
        full_mean = full.mean()
        full_std  = full.std() + 1e-6

        # z-score of ROI relative to frame
        z = (roi_mean - full_mean) / full_std
        # Sigmoid to [0, 1], calibrated so z=2 → conf≈0.88
        conf = 1.0 / (1.0 + np.exp(-0.8 * z))
        return float(np.clip(conf, 0.0, 1.0))

    def release(self) -> None:
        if self._cap:
            self._cap.release()
