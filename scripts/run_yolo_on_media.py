#!/usr/bin/env python3
import argparse
import pathlib
import sys


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run YOLOv5 on a local image/video and save annotated output."
    )
    parser.add_argument(
        "--source",
        default="test_data/driverless_test_media/cones_test.png",
        help="Path to an image, video, or a directory of media files.",
    )
    parser.add_argument(
        "--weights",
        default="models/perception/yolo/ruben_yolov11n_2026_03.pt",
        help="Path to YOLO weights (.pt).",
    )
    parser.add_argument(
        "--output",
        default="outputs/yolo",
        help="Directory where annotated results are saved.",
    )
    parser.add_argument(
        "--conf",
        type=float,
        default=0.25,
        help="Confidence threshold for detections.",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=640,
        help="Inference image size (square).",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    source = pathlib.Path(args.source)
    weights = pathlib.Path(args.weights)
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)

    if not weights.exists():
        print(f"Missing weights at {weights}", file=sys.stderr)
        return 2

    try:
        from ultralytics import YOLO
        model = YOLO(str(weights))
        results = model(
            str(source), imgsz=args.imgsz, conf=args.conf, iou=0.45,
            save=True, project=str(output.parent), name=output.name,
        )
        print(f"Saved annotated results to {output}")
    except ImportError:
        print("Missing ultralytics. Install with: pip install ultralytics", file=sys.stderr)
        return 2

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
