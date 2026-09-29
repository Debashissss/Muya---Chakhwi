#!/usr/bin/env python3
"""
main.py — DRISHYA V1 Human Detection Module entry point.

Runs YOLO11s-based human detection on a camera, video, image, or folder.
Supports optional RGB+Thermal fusion and MAVLink GPS geo-tagging.

Usage examples:
  python main.py --source 0                         # live webcam
  python main.py --source video.mp4                 # video file
  python main.py --source images/                   # image folder
  python main.py --source 0 --thermal 1 --fuse      # dual camera fusion
  python main.py --source 0 --mavlink COM3          # with GPS tagging
  python main.py --source 0 --device cpu --imgsz 416  # edge-optimised
"""

import argparse
import json
import logging
import os
import sys
import time
import uuid
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

# ── Local modules ──────────────────────────────────────────────────────────
from detector          import HumanDetector
from tracker           import IoUTracker
from priority          import compute_priority
from gps_fuser         import GPSFuser
from thermal_fusion    import ThermalFusion
from db                import init_db, log_session, log_detection

# ── Paths ──────────────────────────────────────────────────────────────────
ROOT       = Path(__file__).parent
DET_DIR    = ROOT / "detections"
LOG_DIR    = ROOT / "logs"
DET_DIR.mkdir(exist_ok=True)
LOG_DIR.mkdir(exist_ok=True)

# ── Logging ────────────────────────────────────────────────────────────────
def setup_logging(session_id: str) -> None:
    log_file = LOG_DIR / f"session_{session_id}.log"
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        handlers=[
            logging.FileHandler(str(log_file)),
            logging.StreamHandler(sys.stdout),
        ],
    )

logger = logging.getLogger("DRISHYA.main")


# ═══════════════════════════════════════════════════════════════════════════
def build_source_list(source: str):
    """Expand the --source argument into a list of (frame_or_path, is_video) tuples."""
    p = Path(source)
    if source.isdigit():
        return [("camera", int(source))]
    if p.is_dir():
        exts = {".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".webp"}
        return [("image", f) for f in sorted(p.iterdir()) if f.suffix.lower() in exts]
    if p.is_file():
        video_exts = {".mp4", ".avi", ".mov", ".mkv", ".m4v"}
        if p.suffix.lower() in video_exts:
            return [("video", str(p))]
        return [("image", p)]
    # RTSP / HTTP stream
    return [("camera", source)]


# ═══════════════════════════════════════════════════════════════════════════
def save_detection_snapshot(frame: np.ndarray, det: dict,
                             priority_result, session_id: str,
                             frame_id: int) -> str:
    """Save annotated crop + JSON sidecar. Returns snapshot path."""
    ts = datetime.utcnow().strftime("%Y%m%dT%H%M%S%f")
    tid = det.get("track_id", 0)
    fname = f"{ts}_t{tid:03d}_f{frame_id:06d}"
    jpg_path  = str(DET_DIR / f"{fname}.jpg")
    json_path = str(DET_DIR / f"{fname}.json")

    # Save cropped detection with a small margin
    x1, y1, x2, y2 = det["bbox"]
    h, w = frame.shape[:2]
    pad = 20
    cx1, cy1 = max(0, x1 - pad), max(0, y1 - pad)
    cx2, cy2 = min(w, x2 + pad), min(h, y2 + pad)
    crop = frame[cy1:cy2, cx1:cx2]
    cv2.imwrite(jpg_path, crop)

    meta = {
        "session_id":  session_id,
        "frame_id":    frame_id,
        "track_id":    tid,
        "timestamp":   datetime.utcnow().isoformat(),
        "confidence":  det["confidence"],
        "bbox":        det["bbox"],
        "center":      list(det["center"]),
        "priority": {
            "band":  priority_result.band,
            "score": priority_result.score,
            "label": priority_result.label,
        } if priority_result else {},
        "gps": {
            "lat": det.get("lat", 0.0),
            "lon": det.get("lon", 0.0),
            "alt": det.get("alt", 0.0),
        },
    }
    with open(json_path, "w") as f:
        json.dump(meta, f, indent=2)

    return jpg_path


# ═══════════════════════════════════════════════════════════════════════════
def run(args: argparse.Namespace) -> None:
    session_id = datetime.utcnow().strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:6]
    setup_logging(session_id)
    logger.info("═" * 60)
    logger.info("DRISHYA V1 Human Detection — Session %s", session_id)
    logger.info("Source  : %s", args.source)
    logger.info("Model   : %s | Device: %s | ImgSz: %d", args.model, args.device, args.imgsz)
    logger.info("Conf: %.2f | IoU: %.2f | Fuse: %s", args.conf, args.iou, args.fuse)
    logger.info("═" * 60)

    # ── Initialise components ──────────────────────────────────────────────
    detector = HumanDetector(
        model_path=args.model,
        conf_thresh=args.conf,
        iou_thresh=args.iou,
        device=args.device,
        imgsz=args.imgsz,
    )
    tracker  = IoUTracker(iou_thresh=0.35, max_missed=15)
    gps      = GPSFuser(port=args.mavlink, baud=args.baud)
    thermal  = ThermalFusion(thermal_src=args.thermal, alpha=0.55)
    db_conn  = init_db()
    log_session(db_conn, session_id, str(args.source), args.model, args.device)

    source_list = build_source_list(str(args.source))
    total_detections = 0
    frame_id = 0

    for src_type, src_val in source_list:
        cap = None
        if src_type in ("camera", "video"):
            cap = cv2.VideoCapture(src_val)
            if not cap.isOpened():
                logger.error("Cannot open source: %s", src_val)
                continue

        logger.info("Processing source: type=%s val=%s", src_type, src_val)

        while True:
            # ── Read frame ────────────────────────────────────────────────
            if src_type in ("camera", "video"):
                ret, frame = cap.read()
                if not ret:
                    break
            else:  # image
                frame = cv2.imread(str(src_val))
                if frame is None:
                    logger.warning("Cannot read image: %s", src_val)
                    break

            frame_id += 1
            t0 = time.perf_counter()

            # ── Optional thermal fusion ────────────────────────────────────
            thermal_grey = thermal.read_thermal() if args.thermal else None
            display_frame = thermal.fuse(frame, thermal_grey) if args.fuse and thermal_grey is not None else frame

            # ── YOLO11s Detection ─────────────────────────────────────────
            detections = detector.detect(frame)   # always detect on RGB

            # ── Tracker update ────────────────────────────────────────────
            detections = tracker.update(detections)

            # ── GPS tag + Priority score ───────────────────────────────────
            priority_results = []
            for det in detections:
                cx, cy = det["center"]
                lat, lon = gps.pixel_to_gps(cx, cy)
                _, _, alt = gps.get_drone_gps()
                det["lat"] = lat
                det["lon"] = lon
                det["alt"] = alt

                # Thermal confidence for this bbox
                t_conf = 0.0
                if thermal_grey is not None:
                    t_conf = thermal.thermal_confidence(
                        thermal_grey, det["bbox"], frame.shape[:2]
                    )
                det["thermal_conf"] = t_conf

                pr = compute_priority(
                    rgb_conf=det["confidence"],
                    thermal_conf=t_conf,
                    track_age=det.get("track_age", 0),
                    bbox_area_ratio=det.get("area_ratio", 0.0),
                )
                priority_results.append(pr)
                det["priority"] = pr.band

                # ── DB + snapshot ─────────────────────────────────────────
                snap = ""
                if args.save and det.get("track_age", 0) % 10 == 0:
                    snap = save_detection_snapshot(display_frame, det, pr,
                                                   session_id, frame_id)

                log_detection(
                    db_conn,
                    session_id=session_id,
                    timestamp=datetime.utcnow().isoformat(),
                    frame_id=frame_id,
                    track_id=det.get("track_id", 0),
                    confidence=det["confidence"],
                    priority=pr.band,
                    bbox_x1=det["bbox"][0], bbox_y1=det["bbox"][1],
                    bbox_x2=det["bbox"][2], bbox_y2=det["bbox"][3],
                    center_x=cx, center_y=cy,
                    lat=lat, lon=lon, altitude_m=alt,
                    thermal_conf=t_conf,
                    source_type=src_type,
                    snapshot_path=snap,
                    extra_json=pr.factors,
                )
                total_detections += 1

            # ── Annotate & display ────────────────────────────────────────
            annotated = HumanDetector.annotate(display_frame, detections, priority_results)
            fps = 1.0 / max(time.perf_counter() - t0, 1e-6)
            cv2.putText(annotated, f"FPS: {fps:.1f}", (10, 56),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (200, 200, 200), 2)

            if args.show:
                cv2.imshow("DRISHYA — Human Detection [YOLO11s]", annotated)
                key = cv2.waitKey(1 if src_type in ("camera","video") else 0) & 0xFF
                if key == ord("q") or key == 27:
                    logger.info("User quit.")
                    break

            if src_type == "image":
                break  # one iteration per image

        if cap:
            cap.release()

    cv2.destroyAllWindows()
    thermal.release()
    gps.stop()

    logger.info("─" * 60)
    logger.info("Session complete. Total detections logged: %d", total_detections)
    logger.info("Snapshots saved in: %s", DET_DIR)
    logger.info("Database: %s", ROOT / "human_detections.db")


# ═══════════════════════════════════════════════════════════════════════════
def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="DRISHYA V1 — YOLO11s Human Detection Module"
    )
    p.add_argument("--source",  default="0",
                   help="Camera index, video path, image path, or folder")
    p.add_argument("--thermal", default=None,
                   help="Thermal camera index or RTSP URL (optional)")
    p.add_argument("--fuse",    action="store_true",
                   help="Enable RGB + Thermal fusion display")
    p.add_argument("--model",   default="yolo11s.pt",
                   help="Model filename inside models/ (default: yolo11s.pt). "
                        "Use yolo11s.onnx for faster offline inference on Pi 5.")
    p.add_argument("--conf",    type=float, default=0.35,
                   help="Detection confidence threshold (default: 0.35)")
    p.add_argument("--iou",     type=float, default=0.45,
                   help="NMS IoU threshold (default: 0.45)")
    p.add_argument("--imgsz",   type=int,   default=416,
                   help="Inference image size (default: 416 — optimised for Pi 5)")
    p.add_argument("--device",  default="cpu",
                   help="Device: cpu (default) / cuda / mps")
    p.add_argument("--mavlink", default=None,
                   help="MAVLink serial port for GPS tagging (e.g. /dev/ttyAMA0)")
    p.add_argument("--baud",    type=int,   default=57600,
                   help="MAVLink baud rate (default: 57600)")
    p.add_argument("--save",    action="store_true", default=True,
                   help="Save annotated detection snapshots (default: True)")
    p.add_argument("--no-save", dest="save", action="store_false")
    p.add_argument("--show",    action="store_true", default=False,
                   help="Show live preview window (add --show if display attached)")
    p.add_argument("--no-show", dest="show", action="store_false")
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()
    run(args)
