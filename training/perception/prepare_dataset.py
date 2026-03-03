#!/usr/bin/env python3
"""Download, convert, merge, and split cone detection datasets.

Reads sources.yaml for enabled datasets, downloads/converts them to YOLO format,
and merges into data/images/{train,val} + data/labels/{train,val}.

Usage:
    python training/perception/prepare_dataset.py [OPTIONS]

Options:
    --sources-yaml PATH   Path to sources.yaml (default: auto-detect next to this script)
    --output-dir PATH     Output directory (default: data/ next to this script)
    --val-split FLOAT     Validation split ratio (default: 0.15)
    --seed INT            Random seed for reproducible splits (default: 42)
    --dry-run             Print what would be done without downloading/copying
    --roboflow-key KEY    Roboflow API key (or set ROBOFLOW_API_KEY env var)
"""

from __future__ import annotations

import argparse
import json
import os
import random
import shutil
import sys
from collections import Counter
from pathlib import Path

import yaml

# Our canonical class list
CLASSES = ["blue_cone", "yellow_cone", "orange_cone", "large_orange_cone"]
CLASS_TO_ID = {name: i for i, name in enumerate(CLASSES)}

SCRIPT_DIR = Path(__file__).resolve().parent


def load_sources(path: Path) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)["sources"]


# ---------------------------------------------------------------------------
# Roboflow download
# ---------------------------------------------------------------------------

def download_roboflow(source: dict, dest: Path, api_key: str) -> Path:
    """Download a dataset from Roboflow in YOLOv11 format."""
    try:
        from roboflow import Roboflow
    except ImportError:
        print("ERROR: pip install roboflow", file=sys.stderr)
        sys.exit(1)

    rf_cfg = source["roboflow"]
    rf = Roboflow(api_key=api_key)
    project = rf.workspace(rf_cfg["workspace"]).project(rf_cfg["project"])
    dataset = project.version(rf_cfg["version"]).download(
        "yolov11", location=str(dest), overwrite=True
    )
    return Path(dataset.location)


# ---------------------------------------------------------------------------
# FSOCO Supervisely → YOLO conversion
# ---------------------------------------------------------------------------

def convert_supervisely_to_yolo(
    src_dir: Path, dest_dir: Path, class_map: dict[str, str | None]
) -> list[Path]:
    """Convert FSOCO Supervisely annotations to YOLO txt format.

    Supervisely structure per dataset:
        <dataset>/img/<image_files>
        <dataset>/ann/<image_name>.json

    Each JSON has objects with classTitle and a bitmap/polygon geometry.
    We convert bounding boxes to YOLO normalized format.
    """
    converted: list[Path] = []
    img_dest = dest_dir / "images"
    lbl_dest = dest_dir / "labels"
    img_dest.mkdir(parents=True, exist_ok=True)
    lbl_dest.mkdir(parents=True, exist_ok=True)

    # FSOCO structure: fsoco-dataset/ has subdirectories per team, each with img/ and ann/
    ann_dirs = sorted(src_dir.rglob("ann"))
    if not ann_dirs:
        print(f"  WARNING: No 'ann' directories found in {src_dir}")
        return converted

    for ann_dir in ann_dirs:
        img_dir = ann_dir.parent / "img"
        if not img_dir.exists():
            continue

        for ann_file in sorted(ann_dir.glob("*.json")):
            with open(ann_file) as f:
                ann = json.load(f)

            img_w = ann.get("size", {}).get("width", 0)
            img_h = ann.get("size", {}).get("height", 0)
            if img_w == 0 or img_h == 0:
                continue

            lines = []
            for obj in ann.get("objects", []):
                src_class = obj.get("classTitle", "")
                target_class = class_map.get(src_class)
                if target_class is None:
                    continue  # skip unmapped / explicitly null
                if target_class not in CLASS_TO_ID:
                    continue

                # Get bounding box from points (polygon or rectangle)
                exterior = obj.get("points", {}).get("exterior", [])
                if len(exterior) < 2:
                    continue
                xs = [p[0] for p in exterior]
                ys = [p[1] for p in exterior]
                x_min, x_max = min(xs), max(xs)
                y_min, y_max = min(ys), max(ys)

                # Normalize to [0, 1]
                x_center = ((x_min + x_max) / 2) / img_w
                y_center = ((y_min + y_max) / 2) / img_h
                w = (x_max - x_min) / img_w
                h = (y_max - y_min) / img_h

                class_id = CLASS_TO_ID[target_class]
                lines.append(f"{class_id} {x_center:.6f} {y_center:.6f} {w:.6f} {h:.6f}")

            if not lines:
                continue

            # Find matching image file
            stem = ann_file.stem
            img_file = None
            for ext in (".jpg", ".jpeg", ".png", ".bmp"):
                candidate = img_dir / (stem + ext)
                if candidate.exists():
                    img_file = candidate
                    break
            if img_file is None:
                continue

            # Copy image and write label
            out_img = img_dest / img_file.name
            out_lbl = lbl_dest / (img_file.stem + ".txt")

            # Handle duplicate filenames by adding suffix
            counter = 1
            while out_img.exists():
                out_img = img_dest / f"{img_file.stem}_{counter}{img_file.suffix}"
                out_lbl = lbl_dest / f"{img_file.stem}_{counter}.txt"
                counter += 1

            shutil.copy2(img_file, out_img)
            out_lbl.write_text("\n".join(lines) + "\n")
            converted.append(out_img)

    return converted


# ---------------------------------------------------------------------------
# YOLO format ingestion (Roboflow downloads, MIT Driverless, own data)
# ---------------------------------------------------------------------------

def ingest_yolo_dataset(
    src_dir: Path, dest_dir: Path, class_map: dict[str, str | None],
    src_class_names: list[str] | None = None,
) -> list[Path]:
    """Copy a YOLO-format dataset into dest_dir, remapping classes.

    Expects src_dir to contain:
        images/ (or train/images, valid/images, etc.)
        labels/ (matching structure)

    src_class_names: ordered list of class names in the source dataset (index → name).
        If None, tries to read from data.yaml in src_dir.
    """
    ingested: list[Path] = []
    img_dest = dest_dir / "images"
    lbl_dest = dest_dir / "labels"
    img_dest.mkdir(parents=True, exist_ok=True)
    lbl_dest.mkdir(parents=True, exist_ok=True)

    # Try to discover class names from data.yaml if not provided
    if src_class_names is None:
        for yaml_name in ("data.yaml", "dataset.yaml", "_darknet.labels"):
            yaml_path = src_dir / yaml_name
            if yaml_path.exists():
                with open(yaml_path) as f:
                    cfg = yaml.safe_load(f)
                names = cfg.get("names", {})
                if isinstance(names, dict):
                    src_class_names = [names[k] for k in sorted(names.keys())]
                elif isinstance(names, list):
                    src_class_names = names
                break

    if src_class_names is None:
        print(f"  WARNING: Could not determine class names for {src_dir}")
        return ingested

    # Build source_id → target_id mapping
    id_remap: dict[int, int | None] = {}
    for src_id, src_name in enumerate(src_class_names):
        target_name = class_map.get(src_name)
        if target_name is None:
            id_remap[src_id] = None  # skip
        elif target_name in CLASS_TO_ID:
            id_remap[src_id] = CLASS_TO_ID[target_name]
        else:
            id_remap[src_id] = None

    # Find all image files across common YOLO directory layouts
    img_dirs = []
    for pattern in ["images", "train/images", "valid/images", "test/images"]:
        d = src_dir / pattern
        if d.is_dir():
            img_dirs.append(d)
    # Also check flat layout
    if not img_dirs:
        img_dirs = [src_dir]

    for img_dir in img_dirs:
        # Corresponding label dir
        lbl_dir = Path(str(img_dir).replace("/images", "/labels"))
        if not lbl_dir.is_dir():
            continue

        for img_file in sorted(img_dir.iterdir()):
            if img_file.suffix.lower() not in (".jpg", ".jpeg", ".png", ".bmp"):
                continue

            lbl_file = lbl_dir / (img_file.stem + ".txt")
            if not lbl_file.exists():
                continue

            # Remap label classes
            new_lines = []
            for line in lbl_file.read_text().strip().splitlines():
                parts = line.strip().split()
                if len(parts) < 5:
                    continue
                src_id = int(parts[0])
                target_id = id_remap.get(src_id)
                if target_id is None:
                    continue
                new_lines.append(f"{target_id} {' '.join(parts[1:])}")

            if not new_lines:
                continue

            # Copy with dedup
            out_img = img_dest / img_file.name
            out_lbl = lbl_dest / (img_file.stem + ".txt")
            counter = 1
            while out_img.exists():
                out_img = img_dest / f"{img_file.stem}_{counter}{img_file.suffix}"
                out_lbl = lbl_dest / f"{img_file.stem}_{counter}.txt"
                counter += 1

            shutil.copy2(img_file, out_img)
            out_lbl.write_text("\n".join(new_lines) + "\n")
            ingested.append(out_img)

    return ingested


# ---------------------------------------------------------------------------
# Train/val split
# ---------------------------------------------------------------------------

def split_train_val(
    merged_dir: Path, output_dir: Path, val_ratio: float, seed: int
) -> tuple[int, int]:
    """Split merged images+labels into train/val sets."""
    img_src = merged_dir / "images"
    lbl_src = merged_dir / "labels"

    images = sorted(img_src.glob("*"))
    images = [p for p in images if p.suffix.lower() in (".jpg", ".jpeg", ".png", ".bmp")]

    random.seed(seed)
    random.shuffle(images)

    val_count = int(len(images) * val_ratio)
    val_images = images[:val_count]
    train_images = images[val_count:]

    for split_name, split_imgs in [("train", train_images), ("val", val_images)]:
        img_dest = output_dir / "images" / split_name
        lbl_dest = output_dir / "labels" / split_name
        img_dest.mkdir(parents=True, exist_ok=True)
        lbl_dest.mkdir(parents=True, exist_ok=True)

        for img_path in split_imgs:
            lbl_path = lbl_src / (img_path.stem + ".txt")
            shutil.move(str(img_path), img_dest / img_path.name)
            if lbl_path.exists():
                shutil.move(str(lbl_path), lbl_dest / lbl_path.name)

    return len(train_images), len(val_images)


# ---------------------------------------------------------------------------
# Statistics
# ---------------------------------------------------------------------------

def print_stats(data_dir: Path) -> None:
    """Print dataset statistics."""
    print("\n=== Dataset Statistics ===")
    for split in ("train", "val"):
        lbl_dir = data_dir / "labels" / split
        if not lbl_dir.exists():
            continue
        counts = Counter()
        total_files = 0
        for lbl_file in lbl_dir.glob("*.txt"):
            total_files += 1
            for line in lbl_file.read_text().strip().splitlines():
                parts = line.strip().split()
                if parts:
                    counts[int(parts[0])] += 1

        print(f"\n{split}: {total_files} images")
        for cls_id, cls_name in enumerate(CLASSES):
            print(f"  {cls_name}: {counts.get(cls_id, 0)} annotations")
        print(f"  total annotations: {sum(counts.values())}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Prepare cone detection dataset")
    parser.add_argument("--sources-yaml", type=Path, default=SCRIPT_DIR / "sources.yaml")
    parser.add_argument("--output-dir", type=Path, default=SCRIPT_DIR / "data")
    parser.add_argument("--val-split", type=float, default=0.15)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--roboflow-key", type=str, default=None)
    args = parser.parse_args()

    sources = load_sources(args.sources_yaml)
    # Load .env file if present (next to this script)
    env_file = SCRIPT_DIR / ".env"
    if env_file.exists():
        for line in env_file.read_text().splitlines():
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                k, v = line.split("=", 1)
                os.environ.setdefault(k.strip(), v.strip())

    api_key = args.roboflow_key or os.environ.get("ROBOFLOW_API_KEY")

    # Temp staging directory for merged data before split
    staging_dir = args.output_dir / "_staging"
    if staging_dir.exists():
        shutil.rmtree(staging_dir)
    staging_dir.mkdir(parents=True)

    total_images = 0

    for key, source in sources.items():
        if not source.get("enabled", False):
            print(f"\nSkipping {source['name']} (disabled)")
            continue

        print(f"\n{'='*60}")
        print(f"Processing: {source['name']}")
        print(f"{'='*60}")

        fmt = source.get("format", "yolo")
        class_map = source.get("class_map", {})

        if args.dry_run:
            print(f"  [DRY RUN] Would process {source['name']} ({fmt} format)")
            continue

        if fmt == "roboflow":
            if not api_key:
                print("  ERROR: Roboflow API key required. Set ROBOFLOW_API_KEY or --roboflow-key")
                continue
            dl_dir = args.output_dir / "_downloads" / key
            dl_dir.mkdir(parents=True, exist_ok=True)
            try:
                src_path = download_roboflow(source, dl_dir, api_key)
                files = ingest_yolo_dataset(src_path, staging_dir, class_map)
                print(f"  Ingested {len(files)} images")
                total_images += len(files)
            except Exception as e:
                print(f"  ERROR downloading {source['name']}: {e}")

        elif fmt == "supervisely":
            local_path = source.get("local_path")
            if not local_path:
                print("  ERROR: local_path not set. Clone FSOCO first, then update sources.yaml")
                continue
            local_path = Path(local_path).expanduser()
            if not local_path.is_absolute():
                local_path = SCRIPT_DIR / local_path
            if not local_path.exists():
                print(f"  ERROR: {local_path} does not exist")
                continue
            files = convert_supervisely_to_yolo(local_path, staging_dir, class_map)
            print(f"  Converted {len(files)} images")
            total_images += len(files)

        elif fmt == "yolo":
            local_path = source.get("local_path")
            if not local_path:
                print("  ERROR: local_path not set for YOLO source")
                continue
            local_path = Path(local_path).expanduser()
            if not local_path.is_absolute():
                local_path = SCRIPT_DIR / local_path
            if not local_path.exists():
                print(f"  ERROR: {local_path} does not exist")
                continue
            files = ingest_yolo_dataset(local_path, staging_dir, class_map)
            print(f"  Ingested {len(files)} images")
            total_images += len(files)

    if args.dry_run:
        print("\n[DRY RUN] No files were modified.")
        return

    if total_images == 0:
        print("\nNo images collected. Check source configs and paths.")
        # Clean up
        shutil.rmtree(staging_dir, ignore_errors=True)
        return

    print(f"\nTotal images collected: {total_images}")
    print(f"Splitting {100 * (1 - args.val_split):.0f}/{100 * args.val_split:.0f} train/val (seed={args.seed})...")

    # Clear existing train/val data
    for split in ("train", "val"):
        for subdir in ("images", "labels"):
            d = args.output_dir / subdir / split
            if d.exists():
                shutil.rmtree(d)
            d.mkdir(parents=True)

    n_train, n_val = split_train_val(staging_dir, args.output_dir, args.val_split, args.seed)
    print(f"  train: {n_train} images")
    print(f"  val:   {n_val} images")

    # Clean up staging
    shutil.rmtree(staging_dir, ignore_errors=True)

    print_stats(args.output_dir)
    print("\nDone! Dataset ready at:", args.output_dir)


if __name__ == "__main__":
    main()
