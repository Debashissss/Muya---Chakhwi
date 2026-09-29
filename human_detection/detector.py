"""
detector.py — YOLO11s / ONNX human detection engine for DRISHYA.

For offline / edge use (Raspberry Pi 5):
  • Place yolo11s.pt  OR  yolo11s.onnx  in the models/ folder.
  • The detector will NOT download weights from the internet.
  • Prefer the ONNX path on Pi 5 (faster, no torch dependency):
      python main.py --model yolo11s.onnx
"""
import logging
from pathlib import Path
from typing import List, Optional

import numpy as np
import cv2

logger = logging.getLogger(__name__)

try:
    from ultralytics import YOLO
    HAS_ULTRALYTICS = True
except ImportError:
    HAS_ULTRALYTICS = False
    logger.warning("ultralytics not installed — ONNX-only mode available.")

try:
    import onnxruntime as ort
    HAS_ORT = True
except ImportError:
    HAS_ORT = False

# Default model filename — must exist in models/ for offline use
DEFAULT_MODEL = "yolo11s.pt"

# COCO class index for 'person'
PERSON_CLASS_ID = 0


class HumanDetector:
    """
    Wraps YOLO11s for real-time human detection.

    Usage:
        detector = HumanDetector()
        detections = detector.detect(frame)
    """

    def __init__(
        self,
        model_path: str = DEFAULT_MODEL,
        conf_thresh: float = 0.35,
        iou_thresh: float = 0.45,
        device: str = "cpu",
        imgsz: int = 416,
    ):
        """
        Args:
            model_path:  Filename inside models/ — MUST exist for offline use.
                         Accepts .pt  (ultralytics) or .onnx (onnxruntime).
            conf_thresh: Minimum confidence to accept a detection.
            iou_thresh:  NMS IoU threshold.
            device:      'cpu' (default on Pi 5). 'cuda'/'mps' where available.
            imgsz:       Inference image size (must be multiple of 32). 416 is
                         a good trade-off between speed and accuracy on Pi 5.
        """
        self.conf_thresh = conf_thresh
        self.iou_thresh  = iou_thresh
        self.imgsz       = imgsz
        self.device      = device
        self.model       = None          # set only when using .pt backend
        self._ort_session = None
        self._ort_input_name = None

        # ── Resolve model path (local models/ folder only — no downloads) ──
        models_dir = Path(__file__).parent / "models"
        local = models_dir / model_path
        if not local.exists():
            raise FileNotFoundError(
                f"Model file not found: {local}\n"
                f"For offline use, copy the weights into the models/ folder.\n"
                f"  .pt  → yolo11s.pt   (requires ultralytics + torch)\n"
                f"  .onnx → yolo11s.onnx (recommended for Raspberry Pi 5)"
            )
        model_src = str(local)

        # ── Choose inference backend ───────────────────────────────────────
        if model_src.endswith(".onnx"):
            if not HAS_ORT:
                raise RuntimeError(
                    "onnxruntime not installed.\n"
                    "On Pi 5: pip install onnxruntime"
                )
            providers = ["CPUExecutionProvider"]
            logger.info("Loading ONNX model from '%s' via onnxruntime", model_src)
            self._ort_session = ort.InferenceSession(model_src, providers=providers)
            self._ort_input_name = self._ort_session.get_inputs()[0].name
            logger.info("ONNX model loaded. Input: %s", self._ort_input_name)
        else:
            if not HAS_ULTRALYTICS:
                raise RuntimeError(
                    "ultralytics not installed.\n"
                    "Install with: pip install ultralytics\n"
                    "Or use the ONNX model for a lighter dependency footprint."
                )
            logger.info("Loading YOLO model from '%s' on device='%s'", model_src, device)
            self.model = YOLO(model_src)
            self.model.to(device)
            logger.info("YOLO model loaded successfully.")

    # ------------------------------------------------------------------ inference
    def detect(self, frame: np.ndarray) -> List[dict]:
        """
        Run inference on a single BGR frame.

        Returns:
            List of detection dicts:
            {
              'bbox':       [x1, y1, x2, y2],   # int pixels
              'confidence': float,
              'class_id':   int,
              'class_name': str,
              'center':     (cx, cy),
              'area_ratio': float,               # fraction of frame area
            }
        """
        if frame is None or frame.size == 0:
            return []

        if self._ort_session is not None:
            return self._detect_onnx(frame)
        return self._detect_ultralytics(frame)

    def _detect_ultralytics(self, frame: np.ndarray) -> List[dict]:
        """Inference via ultralytics YOLO (.pt model)."""
        h, w = frame.shape[:2]
        results = self.model.predict(
            source=frame,
            conf=self.conf_thresh,
            iou=self.iou_thresh,
            imgsz=self.imgsz,
            classes=[PERSON_CLASS_ID],
            verbose=False,
            device=self.device,
        )
        detections = []
        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                cls_id = int(box.cls[0])
                if cls_id != PERSON_CLASS_ID:
                    continue
                conf = float(box.conf[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                detections.append(self._make_det(x1, y1, x2, y2, conf, cls_id, w, h))
        return detections

    def _detect_onnx(self, frame: np.ndarray) -> List[dict]:
        """
        Inference via ONNX Runtime.
        Expects a YOLOv8/YOLO11 ONNX export with output shape
        [1, 84, num_anchors] (COCO 80-class).
        Export with: yolo export model=yolo11s.pt format=onnx imgsz=416
        """
        h_orig, w_orig = frame.shape[:2]
        sz = self.imgsz

        # Pre-process: resize → RGB → float32 → NCHW
        blob = cv2.resize(frame, (sz, sz))
        blob = cv2.cvtColor(blob, cv2.COLOR_BGR2RGB)
        blob = blob.astype(np.float32) / 255.0
        blob = np.transpose(blob, (2, 0, 1))[np.newaxis]  # 1,3,H,W

        raw = self._ort_session.run(None, {self._ort_input_name: blob})[0]  # [1,84,N]
        raw = raw[0]  # [84, N]
        raw = raw.T   # [N, 84]  — cx,cy,w,h, cls0..cls79

        detections = []
        scale_x = w_orig / sz
        scale_y = h_orig / sz

        for row in raw:
            scores = row[4:]                      # class scores (80 classes)
            cls_id = int(np.argmax(scores))
            if cls_id != PERSON_CLASS_ID:
                continue
            conf = float(scores[cls_id])
            if conf < self.conf_thresh:
                continue
            # YOLO11 ONNX outputs cx, cy, bw, bh in imgsz-pixel space
            cx, cy, bw, bh = row[:4]
            x1 = int((cx - bw / 2) * scale_x)
            y1 = int((cy - bh / 2) * scale_y)
            x2 = int((cx + bw / 2) * scale_x)
            y2 = int((cy + bh / 2) * scale_y)
            detections.append(self._make_det(x1, y1, x2, y2, conf, cls_id, w_orig, h_orig))

        # Simple NMS to remove duplicates
        detections = self._nms(detections)
        return detections

    @staticmethod
    def _make_det(x1: int, y1: int, x2: int, y2: int,
                  conf: float, cls_id: int, w: int, h: int) -> dict:
        """Build a detection dict from raw box coordinates."""
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        area_ratio = ((x2 - x1) * (y2 - y1)) / max(w * h, 1)
        return {
            "bbox":       [x1, y1, x2, y2],
            "confidence": round(conf, 4),
            "class_id":   cls_id,
            "class_name": "person",
            "center":     (cx, cy),
            "area_ratio": round(area_ratio, 5),
        }

    def _nms(self, detections: List[dict]) -> List[dict]:
        """CPU non-maximum suppression for the ONNX path."""
        if not detections:
            return []
        boxes = np.array([d["bbox"] for d in detections], dtype=np.float32)
        scores = np.array([d["confidence"] for d in detections], dtype=np.float32)
        # Convert to (x, y, w, h) for cv2.dnn.NMSBoxes
        xywh = np.stack([
            boxes[:, 0],
            boxes[:, 1],
            boxes[:, 2] - boxes[:, 0],
            boxes[:, 3] - boxes[:, 1],
        ], axis=1).tolist()
        indices = cv2.dnn.NMSBoxes(
            xywh, scores.tolist(), self.conf_thresh, self.iou_thresh
        )
        if len(indices) == 0:
            return []
        # cv2.dnn.NMSBoxes returns shape (N,1) in OpenCV ≥4.7, flat in older
        indices = np.array(indices).flatten().astype(int).tolist()
        return [detections[i] for i in indices]

    def detect_batch(self, frames: List[np.ndarray]) -> List[List[dict]]:
        """Run inference on a batch of frames. Returns list of detection lists."""
        return [self.detect(f) for f in frames]

    @staticmethod
    def annotate(frame: np.ndarray, detections: List[dict],
                 priority_results: Optional[list] = None) -> np.ndarray:
        """
        Draw bounding boxes and labels on a copy of the frame.

        Args:
            frame:            BGR image.
            detections:       Output of detect().
            priority_results: Optional list of PriorityResult objects (same order).
        Returns:
            Annotated BGR image.
        """
        out = frame.copy()
        for i, d in enumerate(detections):
            x1, y1, x2, y2 = d["bbox"]
            colour = (0, 255, 0)  # default green
            label  = f"Person {d['confidence']:.2f}"

            if priority_results and i < len(priority_results):
                pr = priority_results[i]
                colour = pr.colour
                tid    = d.get("track_id", "?")
                label  = f"ID{tid} {pr.label}"

            cv2.rectangle(out, (x1, y1), (x2, y2), colour, 2)

            # Background pill for text
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
            cv2.rectangle(out, (x1, y1 - th - 6), (x1 + tw + 4, y1), colour, -1)
            cv2.putText(out, label, (x1 + 2, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2,
                        cv2.LINE_AA)

        # Frame summary
        cnt = len(detections)
        summary = f"Detected: {cnt} person(s)"
        cv2.putText(out, summary, (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                    0.9, (0, 220, 255), 2, cv2.LINE_AA)
        return out
