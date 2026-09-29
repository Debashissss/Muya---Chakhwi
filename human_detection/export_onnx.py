#!/usr/bin/env python3
"""
export_onnx.py — Export YOLO11s to ONNX for offline Raspberry Pi 5 use.

Run this ONCE on a PC/Mac (with GPU recommended, ultralytics installed):
    python export_onnx.py

Then copy  models/yolo11s.onnx  to your Pi 5.
"""

from pathlib import Path
from ultralytics import YOLO

# ── Config ────────────────────────────────────────────────────────────────────
MODEL_NAME = "yolo11s.pt"   # downloaded automatically if not present here
OUTPUT_DIR = Path(__file__).parent / "models"
IMGSZ      = 416            # inference size used on Pi 5 (faster, still accurate)
# ─────────────────────────────────────────────────────────────────────────────

OUTPUT_DIR.mkdir(exist_ok=True)

print(f"[export] Loading {MODEL_NAME} ...")
model = YOLO(MODEL_NAME)

print(f"[export] Exporting to ONNX (imgsz={IMGSZ}) ...")
export_path = model.export(
    format="onnx",
    imgsz=IMGSZ,
    simplify=True,   # apply ONNX simplifier for smaller, faster model
    dynamic=False,   # fixed batch size = 1 (required for static ORT inference)
    opset=17,
)

# Move to models/ if exported to project root
src = Path(export_path)
dst = OUTPUT_DIR / src.name
if src.resolve() != dst.resolve():
    src.rename(dst)

print(f"\n[export] Done!  Model saved to: {dst}")
print("[export] Copy this file to your Pi 5:  models/yolo11s.onnx")
print("[export] Then run: python main.py --model yolo11s.onnx --source 0")
