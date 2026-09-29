#!/usr/bin/env bash
# setup_pi5.sh — One-shot offline setup for DRISHYA on Raspberry Pi 5
#
# Run once after copying the project folder to the Pi:
#   chmod +x setup_pi5.sh && ./setup_pi5.sh
#
# Pre-requisites:
#   • Raspberry Pi OS Bookworm 64-bit (Python 3.11 ships by default)
#   • models/yolo11s.onnx  must exist (export on PC first via export_onnx.py)

set -e

echo "═══════════════════════════════════════════════"
echo " DRISHYA V1 — Raspberry Pi 5 Setup"
echo "═══════════════════════════════════════════════"

# ── System packages (installed once, works offline with apt cache) ──────────
echo "[1/4] Installing system dependencies ..."
sudo apt-get update -qq
sudo apt-get install -y --no-install-recommends \
    python3-pip python3-venv \
    libatlas-base-dev libopenblas-dev \
    libcamera-apps \
    v4l-utils \
    libglib2.0-0 \
    libjpeg-dev libpng-dev

# ── Python virtual environment ─────────────────────────────────────────────
echo "[2/4] Creating Python virtual environment ..."
python3 -m venv .venv --system-site-packages
source .venv/bin/activate

# ── Python packages ────────────────────────────────────────────────────────
echo "[3/4] Installing Python packages ..."
pip install --upgrade pip --quiet
pip install -r requirements.txt --quiet

# ── Verify model weights ───────────────────────────────────────────────────
echo "[4/4] Checking model weights ..."
if [ -f "models/yolo11s.onnx" ]; then
    echo "      ✓  models/yolo11s.onnx found — ready for offline inference!"
elif [ -f "models/yolo11s.pt" ]; then
    echo "      ✓  models/yolo11s.pt found (requires ultralytics+torch)."
    echo "      ⚠  For better Pi 5 performance, export to ONNX on your PC:"
    echo "         python export_onnx.py   (run on PC, not Pi)"
else
    echo ""
    echo "  ╔═══════════════════════════════════════════════════════════════╗"
    echo "  ║  ERROR: No model weights found in models/                    ║"
    echo "  ║  1. On your PC (with internet):                              ║"
    echo "  ║       python export_onnx.py                                  ║"
    echo "  ║  2. Copy models/yolo11s.onnx to this Pi.                     ║"
    echo "  ╚═══════════════════════════════════════════════════════════════╝"
    exit 1
fi

echo ""
echo "═══════════════════════════════════════════════"
echo " Setup complete!  Run the detector:"
echo ""
echo "   source .venv/bin/activate"
echo "   python main.py --model yolo11s.onnx --source 0"
echo "═══════════════════════════════════════════════"
