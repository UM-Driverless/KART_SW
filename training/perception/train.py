#!/usr/bin/env python3
"""Train YOLOv11 for cone detection.

Thin wrapper around ultralytics that loads dataset.yaml and fine-tunes
a YOLOv11 model. Saves best weights to models/perception/yolo/.

Usage:
    python training/perception/train.py [OPTIONS]

Options:
    --model PATH       Base model to fine-tune (default: yolo11n.pt)
    --epochs INT       Number of training epochs (default: 100)
    --batch INT        Batch size (default: 16)
    --imgsz INT        Image size (default: 640)
    --device STR       Device: 'cuda', 'mps', 'cpu', or device id (default: auto)
    --project PATH     Project directory for runs (default: training/perception/runs)
    --name STR         Run name (default: auto)
    --weights PATH     Path to trained weights for validation-only mode
    --val-only         Run validation only (requires --weights)
    --resume           Resume training from last checkpoint
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent.parent
DATASET_YAML = SCRIPT_DIR / "dataset.yaml"
MODELS_DIR = REPO_ROOT / "models" / "perception" / "yolo"


def main():
    parser = argparse.ArgumentParser(description="Train YOLOv11 cone detector")
    parser.add_argument("--model", type=str, default="yolo11n.pt")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", type=str, default=None)
    parser.add_argument("--project", type=Path, default=SCRIPT_DIR / "runs")
    parser.add_argument("--name", type=str, default=None)
    parser.add_argument("--weights", type=Path, default=None)
    parser.add_argument("--val-only", action="store_true")
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()

    from ultralytics import YOLO

    if args.val_only:
        if not args.weights:
            parser.error("--val-only requires --weights")
        print(f"Validating {args.weights} on {DATASET_YAML}")
        model = YOLO(str(args.weights))
        metrics = model.val(data=str(DATASET_YAML), imgsz=args.imgsz, batch=args.batch)
        print(f"\nmAP50:    {metrics.box.map50:.4f}")
        print(f"mAP50-95: {metrics.box.map:.4f}")
        return

    # Training
    model = YOLO(args.model)
    train_kwargs = dict(
        data=str(DATASET_YAML),
        epochs=args.epochs,
        batch=args.batch,
        imgsz=args.imgsz,
        project=str(args.project),
        exist_ok=True,
    )
    if args.device:
        train_kwargs["device"] = args.device
    if args.name:
        train_kwargs["name"] = args.name
    if args.resume:
        train_kwargs["resume"] = True

    print(f"Training {args.model} for {args.epochs} epochs")
    print(f"Dataset: {DATASET_YAML}")
    print(f"Batch: {args.batch}, Image size: {args.imgsz}")

    results = model.train(**train_kwargs)

    # Copy best weights to models directory
    best_pt = Path(results.save_dir) / "weights" / "best.pt"
    if best_pt.exists():
        dest = MODELS_DIR / "cone_detector_latest.pt"
        shutil.copy2(best_pt, dest)
        print(f"\nBest weights copied to: {dest}")
    else:
        print("\nWARNING: best.pt not found in training output")

    print("Training complete!")


if __name__ == "__main__":
    main()
