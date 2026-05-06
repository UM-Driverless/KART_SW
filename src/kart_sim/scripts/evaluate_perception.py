#!/usr/bin/env python3
"""Evaluate YOLO detection on Gazebo-rendered frames.

Loads captured RGB frames, runs YOLOv5 inference, computes ground-truth
bounding boxes from known cone world positions + camera pose, and reports
precision, recall, and detection rate per class.

Usage:
    python3 evaluate_perception.py \
        --frames /tmp/gazebo_frames \
        --weights ~/kart-brain/models/perception/yolo/best_adri.pt \
        --world-sdf ~/kart-brain/install/kart_sim/share/kart_sim/worlds/fs_track.sdf

The camera is assumed to be at the kart's start pose (20, 0, yaw=π/2) for
the first frame. If an odom log is available, pass --odom-csv for per-frame
poses. Otherwise all frames use the same static pose (suitable for a
stationary kart).
"""
import argparse
import csv
import math
import os
import sys
import warnings
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

# Suppress YOLOv5 warnings
warnings.filterwarnings("ignore", message=".*autocast.*", category=FutureWarning)
os.environ.setdefault("MPLBACKEND", "Agg")


# ── Cone parsing (same logic as perfect_perception_node) ──────────────────

def parse_cones_from_sdf(sdf_path: str) -> List[Dict]:
    """@brief Parse cone positions from the world SDF file.

    @param sdf_path Path to the SDF world file.
    @return List of dicts with keys: name, x, y, z, class_id.
    """
    import re

    cones = []
    with open(sdf_path) as f:
        content = f.read()

    # Inline models
    for m in re.finditer(
        r'<model name="((?:blue|yellow|orange)_\w+)".*?<pose>([\d\s.eE+\-]+)</pose>',
        content, re.DOTALL
    ):
        name, pose = m.group(1), m.group(2).strip().split()
        cones.append(_make_cone(name, pose))

    # Include tags
    for m in re.finditer(
        r'<include>\s*<uri>model://cone_\w+</uri>\s*'
        r'<name>((?:blue|yellow|orange)_\w+)</name>\s*'
        r'<pose>([\d\s.eE+\-]+)</pose>\s*</include>',
        content, re.DOTALL
    ):
        name, pose = m.group(1), m.group(2).strip().split()
        cones.append(_make_cone(name, pose))

    return cones


def _make_cone(name: str, pose: List[str]) -> Dict:
    """@brief Create a cone dict from an SDF model name and pose strings.

    @param name SDF model name (e.g. "blue_rs_0").
    @param pose List of pose coordinate strings [x, y, z, ...].
    @return Dict with keys: name, x, y, z, class_id.
    """
    x, y, z = float(pose[0]), float(pose[1]), float(pose[2])
    if name.startswith("blue"):
        cls = "blue_cone"
    elif name.startswith("yellow"):
        cls = "yellow_cone"
    elif name.startswith("orange"):
        cls = "orange_cone"
    else:
        cls = "unknown"
    return {"name": name, "x": x, "y": y, "z": z, "class_id": cls}


# ── Camera model ──────────────────────────────────────────────────────────

# Camera intrinsics derived from the sensor's HFOV (1.396 rad = 80°) and resolution.
# Note: Gazebo Fortress CameraInfo reports incorrect intrinsics (CX=160, CY=120 for
# a 640×360 image). The HFOV-derived values match the actual rendered images.
IMG_W, IMG_H = 640, 360
HFOV_RAD = 1.396
FX = IMG_W / (2.0 * math.tan(HFOV_RAD / 2.0))
FY = FX
CX = IMG_W / 2.0
CY = IMG_H / 2.0

# Camera offset from kart base_link
CAM_X_OFFSET = 0.55
# The kart model has internal z-offset 0.22, but the <include> pose in fs_track.sdf
# overrides it to z=0. After physics settling, effective camera z ≈ 0.30m.
# Calibrated empirically by matching GT projections to YOLO detections.
CAM_Z_OFFSET = 0.30

# Cone physical sizes (for bounding box estimation)
CONE_RADIUS = 0.114  # m (base)
CONE_HEIGHT_SMALL = 0.325
CONE_HEIGHT_BIG = 0.505


def project_cone_to_bbox(
    cone: Dict,
    kart_x: float, kart_y: float, kart_yaw: float,
) -> Optional[Tuple[float, float, float, float, str]]:
    """@brief Project a 3D cone into a 2D bounding box in the image.

    @param cone Dict with x, y, z, name, class_id keys.
    @param kart_x Kart X position in world frame (meters).
    @param kart_y Kart Y position in world frame (meters).
    @param kart_yaw Kart heading in world frame (radians).
    @return Tuple (x1, y1, x2, y2, class_id) or None if cone is not visible.
    """
    cos_yaw = math.cos(kart_yaw)
    sin_yaw = math.sin(kart_yaw)

    # Camera world position
    cam_wx = kart_x + CAM_X_OFFSET * cos_yaw
    cam_wy = kart_y + CAM_X_OFFSET * sin_yaw
    cam_wz = CAM_Z_OFFSET

    # Cone base center (ground level)
    cone_x, cone_y = cone["x"], cone["y"]
    height = CONE_HEIGHT_BIG if "big" in cone["name"] or "sf" in cone["name"] else CONE_HEIGHT_SMALL
    cone_bottom_z = 0.0
    cone_top_z = height

    # Transform to camera frame (X=forward, Y=left, Z=up)
    dx = cone_x - cam_wx
    dy = cone_y - cam_wy
    cam_fwd = dx * cos_yaw + dy * sin_yaw  # X in cam frame
    cam_left = -dx * sin_yaw + dy * cos_yaw  # Y in cam frame

    if cam_fwd < 0.5:  # behind or too close
        return None

    # Project 4 corners of the cone bounding cylinder into the image
    # Camera optical frame: Z=forward, X=right, Y=down
    # Conversion: optical_z = cam_fwd, optical_x = -cam_left, optical_y = -(cam_z - cam_wz)

    points_2d = []
    for cz in [cone_bottom_z, cone_top_z]:
        for offset in [-CONE_RADIUS, CONE_RADIUS]:
            opt_z = cam_fwd  # depth
            opt_x = -cam_left + offset  # horizontal offset (approximation)
            opt_y = -(cz - cam_wz)  # vertical

            if opt_z <= 0:
                continue

            u = FX * opt_x / opt_z + CX
            v = FY * opt_y / opt_z + CY
            points_2d.append((u, v))

    if len(points_2d) < 2:
        return None

    us = [p[0] for p in points_2d]
    vs = [p[1] for p in points_2d]
    x1, x2 = min(us), max(us)
    y1, y2 = min(vs), max(vs)

    # Clip to image bounds
    x1 = max(0, x1)
    y1 = max(0, y1)
    x2 = min(IMG_W, x2)
    y2 = min(IMG_H, y2)

    # Filter out boxes too small or fully outside
    if x2 - x1 < 3 or y2 - y1 < 3:
        return None

    return (x1, y1, x2, y2, cone["class_id"])


# ── IoU computation ───────────────────────────────────────────────────────

def iou(box_a: Tuple, box_b: Tuple) -> float:
    """@brief Compute Intersection over Union between two bounding boxes.

    @param box_a First box as (x1, y1, x2, y2).
    @param box_b Second box as (x1, y1, x2, y2).
    @return IoU value in [0.0, 1.0].
    """
    xa = max(box_a[0], box_b[0])
    ya = max(box_a[1], box_b[1])
    xb = min(box_a[2], box_b[2])
    yb = min(box_a[3], box_b[3])
    inter = max(0, xb - xa) * max(0, yb - ya)
    area_a = (box_a[2] - box_a[0]) * (box_a[3] - box_a[1])
    area_b = (box_b[2] - box_b[0]) * (box_b[3] - box_b[1])
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


# ── Main evaluation ──────────────────────────────────────────────────────

def load_yolo_model(weights_path: str, conf: float = 0.25, iou_thresh: float = 0.45):
    """@brief Load a YOLOv5 model from custom weights.

    @param weights_path Path to the .pt weights file.
    @param conf Confidence threshold for detections.
    @param iou_thresh NMS IoU threshold.
    @return Loaded YOLOv5 model on CPU.
    """
    import torch
    model = torch.hub.load("ultralytics/yolov5", "custom", path=weights_path)
    model.conf = conf
    model.iou = iou_thresh
    model.to("cpu")
    return model


def run_evaluation(
    frames_dir: Path,
    weights_path: str,
    world_sdf: str,
    kart_x: float = 20.0,
    kart_y: float = 0.0,
    kart_yaw: float = math.pi / 2,
    odom_csv: Optional[str] = None,
    iou_threshold: float = 0.5,
    conf_threshold: float = 0.25,
):
    """@brief Run YOLO evaluation on captured Gazebo frames against ground-truth cone positions.

    @param frames_dir Directory containing captured RGB frames.
    @param weights_path Path to YOLOv5 weights (.pt).
    @param world_sdf Path to the SDF world file for ground-truth cone positions.
    @param kart_x Default kart X position (used if no odom CSV).
    @param kart_y Default kart Y position.
    @param kart_yaw Default kart heading (radians).
    @param odom_csv Optional CSV with per-frame x, y, yaw columns.
    @param iou_threshold IoU threshold for matching predictions to ground truth.
    @param conf_threshold Confidence threshold for YOLO detections.
    @return True if detection rate exceeds 80%.
    """
    # Parse cones from world
    cones = parse_cones_from_sdf(world_sdf)
    print(f"Loaded {len(cones)} cones from {world_sdf}")

    # Load per-frame odometry if available
    odom_poses = None
    if odom_csv and Path(odom_csv).exists():
        odom_poses = []
        with open(odom_csv) as f:
            reader = csv.DictReader(f)
            for row in reader:
                odom_poses.append((
                    float(row["x"]), float(row["y"]), float(row["yaw"])
                ))
        print(f"Loaded {len(odom_poses)} odom poses from {odom_csv}")

    # Load YOLO model
    print(f"Loading YOLO model from {weights_path}...")
    model = load_yolo_model(weights_path, conf=conf_threshold)
    class_names = model.names
    print(f"YOLO classes: {class_names}")

    # Find RGB frames
    rgb_dir = frames_dir / "rgb"
    if not rgb_dir.exists():
        rgb_dir = frames_dir  # flat directory
    frames = sorted(rgb_dir.glob("*.png"))
    if not frames:
        print(f"No PNG frames found in {rgb_dir}")
        sys.exit(1)
    print(f"Found {len(frames)} frames")

    # Metrics accumulators
    total_gt = 0
    total_tp = 0
    total_fp = 0
    total_fn = 0
    class_stats: Dict[str, Dict[str, int]] = {}
    all_confidences = []

    for i, frame_path in enumerate(frames):
        # Determine kart pose for this frame
        if odom_poses and i < len(odom_poses):
            kx, ky, kyaw = odom_poses[i]
        else:
            kx, ky, kyaw = kart_x, kart_y, kart_yaw

        # Compute ground-truth bounding boxes
        gt_boxes = []
        for cone in cones:
            result = project_cone_to_bbox(cone, kx, ky, kyaw)
            if result is not None:
                gt_boxes.append(result)

        # Run YOLO inference
        frame_bgr = cv2.imread(str(frame_path))
        if frame_bgr is None:
            print(f"  Warning: could not read {frame_path}")
            continue
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        results = model(frame_rgb, size=640)

        # Parse YOLO detections
        pred_boxes = []
        for det in results.xyxy[0].tolist():
            x1, y1, x2, y2, conf, cls_id = det
            cls_name = class_names[int(cls_id)]
            pred_boxes.append((x1, y1, x2, y2, cls_name, conf))
            all_confidences.append(conf)

        # Match predictions to ground truth (greedy, IoU threshold)
        gt_matched = [False] * len(gt_boxes)
        pred_matched = [False] * len(pred_boxes)

        # Sort predictions by confidence (highest first)
        pred_sorted = sorted(range(len(pred_boxes)), key=lambda k: pred_boxes[k][5], reverse=True)

        tp = 0
        for pi in pred_sorted:
            px1, py1, px2, py2, pcls, pconf = pred_boxes[pi]
            best_iou = 0.0
            best_gi = -1
            for gi, (gx1, gy1, gx2, gy2, gcls) in enumerate(gt_boxes):
                if gt_matched[gi]:
                    continue
                # Class must match (or be compatible: orange_cone matches both)
                if pcls != gcls:
                    continue
                cur_iou = iou((px1, py1, px2, py2), (gx1, gy1, gx2, gy2))
                if cur_iou > best_iou:
                    best_iou = cur_iou
                    best_gi = gi
            if best_iou >= iou_threshold and best_gi >= 0:
                gt_matched[best_gi] = True
                pred_matched[pi] = True
                tp += 1

        fp = sum(1 for m in pred_matched if not m)
        fn = sum(1 for m in gt_matched if not m)

        total_gt += len(gt_boxes)
        total_tp += tp
        total_fp += fp
        total_fn += fn

        # Per-class stats
        for gx1, gy1, gx2, gy2, gcls in gt_boxes:
            if gcls not in class_stats:
                class_stats[gcls] = {"gt": 0, "tp": 0, "fp": 0}
            class_stats[gcls]["gt"] += 1
        for gi, matched in enumerate(gt_matched):
            gcls = gt_boxes[gi][4]
            if matched:
                class_stats[gcls]["tp"] += 1
        for pi, matched in enumerate(pred_matched):
            pcls = pred_boxes[pi][4]
            if not matched:
                if pcls not in class_stats:
                    class_stats[pcls] = {"gt": 0, "tp": 0, "fp": 0}
                class_stats[pcls]["fp"] += 1

        print(
            f"  Frame {i:3d}: GT={len(gt_boxes):2d} Pred={len(pred_boxes):2d} "
            f"TP={tp} FP={fp} FN={fn}"
        )

    # ── Summary ──
    print("\n" + "=" * 60)
    print("EVALUATION SUMMARY")
    print("=" * 60)

    precision = total_tp / (total_tp + total_fp) if (total_tp + total_fp) > 0 else 0.0
    recall = total_tp / (total_tp + total_fn) if (total_tp + total_fn) > 0 else 0.0
    detection_rate = total_tp / total_gt if total_gt > 0 else 0.0
    avg_conf = sum(all_confidences) / len(all_confidences) if all_confidences else 0.0

    print(f"  Total ground truth:  {total_gt}")
    print(f"  True positives:      {total_tp}")
    print(f"  False positives:     {total_fp}")
    print(f"  False negatives:     {total_fn}")
    print(f"  Precision:           {precision:.1%}")
    print(f"  Recall:              {recall:.1%}")
    print(f"  Detection rate:      {detection_rate:.1%}")
    print(f"  Average confidence:  {avg_conf:.3f}")
    print(f"  IoU threshold:       {iou_threshold}")

    print(f"\nPer-class breakdown:")
    for cls, stats in sorted(class_stats.items()):
        cls_det_rate = stats["tp"] / stats["gt"] if stats["gt"] > 0 else 0.0
        print(f"  {cls:20s}: GT={stats['gt']:3d}  TP={stats['tp']:3d}  FP={stats['fp']:3d}  DetRate={cls_det_rate:.1%}")

    # Pass/fail
    print(f"\n{'PASS' if detection_rate > 0.8 else 'FAIL'}: Detection rate {detection_rate:.1%} {'>' if detection_rate > 0.8 else '<='} 80% threshold")

    return detection_rate > 0.8


def main():
    """@brief CLI entrypoint for YOLO perception evaluation."""
    parser = argparse.ArgumentParser(description="Evaluate YOLO on Gazebo frames")
    parser.add_argument("--frames", required=True, help="Directory with captured frames")
    parser.add_argument("--weights", required=True, help="Path to YOLOv5 weights (.pt)")
    parser.add_argument("--world-sdf", required=True, help="Path to fs_track.sdf")
    parser.add_argument("--kart-x", type=float, default=20.0)
    parser.add_argument("--kart-y", type=float, default=0.0)
    parser.add_argument("--kart-yaw", type=float, default=math.pi / 2)
    parser.add_argument("--odom-csv", default=None, help="CSV with per-frame x,y,yaw")
    parser.add_argument("--iou-threshold", type=float, default=0.5)
    parser.add_argument("--conf-threshold", type=float, default=0.25)
    args = parser.parse_args()

    passed = run_evaluation(
        frames_dir=Path(args.frames),
        weights_path=args.weights,
        world_sdf=args.world_sdf,
        kart_x=args.kart_x,
        kart_y=args.kart_y,
        kart_yaw=args.kart_yaw,
        odom_csv=args.odom_csv,
        iou_threshold=args.iou_threshold,
        conf_threshold=args.conf_threshold,
    )
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
