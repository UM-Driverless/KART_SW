# TODO

## HIGH PRIORITY (Firmware & Core)

- [ ] **PID tuning** -- tune kp/ki/kd now that steering gears are fixed and outputLimit works

## MEDIUM PRIORITY (Optimization & Infrastructure)

- [ ] **Reduce ZED depth GPU load to speed up YOLO** -- ZED's `NEURAL_LIGHT` depth mode runs a neural net on GPU every frame, competing with YOLO (~45 Hz instead of 72 Hz in isolation). Try: (1) switch to stereo-based depth mode (`ULTRA`/`QUALITY`) which uses CPU instead, freeing GPU for YOLO; (2) reduce depth computation frequency (we don't need depth at 100 Hz); (3) try INT8 quantization for the YOLO TRT engine (~1.5x speedup).
- [ ] **Benchmark YOLOv10n vs v11n on Orin with TensorRT** -- export both, compare ms/frame. v10 removes NMS which may help.
- [ ] Create reproducible Orin setup script/guide

## LOW PRIORITY (Long-Term)

- [ ] Trajectory planning from cone positions
- [ ] Make a map of the university https://x.com/junyi42/status/2031024111716331759
