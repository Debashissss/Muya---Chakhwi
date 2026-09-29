# DRISHYA V1 — Human Detection Module
### YOLOv11s-powered Search-and-Rescue Human Detector

---

## Overview

Real-time human detection pipeline designed for disaster SAR drone missions.
Uses **YOLO11s** (small, edge-optimised) to detect people from:

- RGB camera frames (live stream or video file)
- Thermal / IR frames (FLIR / Lepton / OAK-D Thermal)
- Fused RGB + Thermal (dual-camera rig)
- Static images for batch post-processing

Each detection produces:
- Bounding box + confidence score
- Priority band (CRITICAL / HIGH / MEDIUM / LOW)
- GPS geo-tag (fused from MAVLink telemetry)
- SQLite log entry + annotated JPEG snapshot

---

## Directory Structure
```
human_detection/
├── main.py              ← Entry point (run this)
├── detector.py          ← YOLO11s detection engine
├── tracker.py           ← Simple IoU tracker (dedup across frames)
├── priority.py          ← Priority scoring engine
├── gps_fuser.py         ← Pixel→GPS coordinate mapper
├── db.py                ← SQLite detection database
├── thermal_fusion.py    ← RGB + Thermal frame fusion
├── requirements.txt
├── models/              ← YOLO11s weights (auto-downloaded)
├── detections/          ← Annotated snapshots + JSON metadata
└── logs/                ← Session logs
```

---

## Installation

```bash
pip install -r requirements.txt
```

> **GPU (CUDA):** For Jetson / RPi5 / x86 with NVIDIA GPU, replace torch with the
> CUDA-enabled wheel. YOLO11s runs fine on CPU too (~4–8 FPS on RPi5).

---

## Running

### Live camera (RGB only)
```bash
python main.py --source 0
```

### Video file
```bash
python main.py --source path/to/video.mp4
```

### Static image or folder
```bash
python main.py --source path/to/image.jpg
python main.py --source path/to/images/
```

### Dual-camera RGB + Thermal
```bash
python main.py --source 0 --thermal 1 --fuse
```

### With MAVLink GPS tagging
```bash
python main.py --source 0 --mavlink COM3 --baud 57600
```

### Raspberry Pi 5 / edge device (optimised)
```bash
python main.py --source 0 --device cpu --imgsz 416 --conf 0.40
```

---

## Key Parameters

| Flag | Default | Description |
|------|---------|-------------|
| `--source` | `0` | Camera index, video path, image path, or folder |
| `--thermal` | `None` | Thermal camera index or RTSP URL |
| `--fuse` | `False` | Enable RGB+Thermal fusion |
| `--model` | `yolo11s.pt` | YOLO model weights file |
| `--conf` | `0.35` | Detection confidence threshold |
| `--iou` | `0.45` | NMS IoU threshold |
| `--imgsz` | `640` | Inference image size |
| `--device` | `auto` | `cpu`, `cuda`, `mps` |
| `--mavlink` | `None` | MAVLink port for GPS (e.g. `COM3`) |
| `--baud` | `57600` | MAVLink baud rate |
| `--save` | `True` | Save annotated snapshots |
| `--show` | `True` | Display live preview window |

---

## Priority Bands

| Band | Colour | Confidence | Conditions |
|------|--------|------------|------------|
| 🔴 CRITICAL | Red | ≥ 0.80 | + thermal heat signature |
| 🟠 HIGH | Orange | ≥ 0.65 | Single camera, high conf |
| 🟡 MEDIUM | Yellow | ≥ 0.50 | Partial / occluded |
| 🟢 LOW | Green | < 0.50 | Uncertain / background |

---

## Hardware Requirements

| Component | Spec |
|-----------|------|
| Edge compute | Raspberry Pi 5 (8 GB) / Jetson Orin Nano |
| RGB camera | Pi Camera Module 3 / USB OV5640 |
| Thermal camera | FLIR Lepton 3.5 / OAK-D Pro Thermal |
| Flight controller | Pixhawk 6X v2 (MAVLink) |

---

## Output

- `detections/<timestamp>_<id>.jpg` — annotated frame snapshot
- `detections/<timestamp>_<id>.json` — detection metadata + GPS
- `logs/session_<date>.log` — session log
- `human_detections.db` — SQLite database (all sessions)
