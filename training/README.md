# Training

Training pipelines for kart_brain models.

## Perception — YOLOv11 Cone Detection

Detects formula-student cones from camera images using YOLOv11.

### Classes

| ID | Name               | Description                        |
|----|--------------------|------------------------------------|
| 0  | `blue_cone`        | Standard blue cone (track left)    |
| 1  | `yellow_cone`      | Standard yellow cone (track right) |
| 2  | `orange_cone`      | Small orange cone (start/finish)   |
| 3  | `large_orange_cone`| Large orange cone (start/finish)   |

### Dataset sources

| Source | Images | Format | License | Link |
|--------|--------|--------|---------|------|
| FSOCO v2 | ~11.5k | Supervisely | MIT | https://github.com/fsoco/fsoco-dataset |
| FSAE Cone (Roboflow) | ~9.9k | YOLO | CC BY 4.0 | https://universe.roboflow.com/roboflow-jvuqo/fsae-cone/dataset/2 |
| TBReAI FS Cones (Roboflow) | ~2.1k | YOLO | CC BY 4.0 | https://universe.roboflow.com/tbreai/fs-cones |
| ARECE 3 (Roboflow) | ~1.5k | YOLO | CC BY 4.0 | https://universe.roboflow.com/arece/arece-3 |
| UQ Racing (Roboflow) | ~795 | YOLO | CC BY 4.0 | https://universe.roboflow.com/uqracing/uqr-cone-detection |
| MIT Driverless | varies | YOLO | Apache-2.0 | https://github.com/cv-core/MIT-Driverless-CV-TrainingInfra |
| Own data | — | YOLO | internal | `training/perception/data/own/` |

### Quick start

```bash
# 1. Install dependencies
pip install ultralytics roboflow pyyaml

# 2. Prepare dataset (download, convert, merge, split)
python training/perception/prepare_dataset.py

# 3. Train
python training/perception/train.py --epochs 100 --batch 16

# 4. Evaluate (automatic at end of training, or manually)
python training/perception/train.py --weights runs/detect/train/weights/best.pt --val-only
```

### Class mapping across sources

Different datasets use different class names. `prepare_dataset.py` normalizes them:

| Source class name | → Our class |
|-------------------|-------------|
| `blue_cone`, `blue-cone`, `Blue Cone` | 0 (`blue_cone`) |
| `yellow_cone`, `yellow-cone`, `Yellow Cone` | 1 (`yellow_cone`) |
| `orange_cone`, `orange-cone`, `Orange Cone`, `small_orange_cone` | 2 (`orange_cone`) |
| `large_orange_cone`, `big_orange_cone`, `Large Orange Cone` | 3 (`large_orange_cone`) |
| `unknown_cone`, `unknown`, `other` | **skipped** |

### File structure

```
training/
├── README.md                ← this file
├── perception/
│   ├── dataset.yaml         ← ultralytics dataset config
│   ├── sources.yaml         ← registry of all dataset sources + URLs
│   ├── prepare_dataset.py   ← download, convert, merge, split
│   ├── train.py             ← training wrapper
│   └── data/                ← gitignored — populated by prepare_dataset.py
│       ├── images/{train,val}/
│       └── labels/{train,val}/
```
