# RTAB-Map runtime performance in offline dataset mode

## Background

`rtabmap-rgbd_dataset` runs every frame through the full SLAM pipeline because
`--Rtabmap/DetectionRate 0` is required for dense pseudo-GT extraction.  This
is intentionally different from real-time ROS operation where place recognition
runs at ≤1 Hz.  The cost breakdown per frame is:

| Step | What it does | Typical time |
|---|---|---|
| **camera** | Load and decode RGB + depth image from disk | 5–6 ms |
| **odom** | Frame-to-map visual odometry (F2M, ORB features) | 25–35 ms |
| **slam** | VBoW retrieval + local map update + (rare) loop closure | 20–35 ms |

At 20 fps input the frame budget is 50 ms.  Mean total is 58–66 ms depending on
scene, so RTAB-Map **runs at roughly 15–17 fps** rather than real-time.

ORB-SLAM3 for comparison: ~89 fps (11 ms/frame) — 5–6× faster.

---

## Why the slam step has a hard floor

The VBoW (Visual Bag of Words) retrieval queries a pre-trained vocabulary tree.
Its cost is O(log N_vocab), where N_vocab is the vocabulary size — **not** the
number of keyframes in the map.  Measured on freiburg1_desk (229 KFs):

```
slam time slope vs KF count: –0.004 ms/KF  (essentially zero)
slam baseline intercept:       35 ms
```

The slam step costs ~35 ms whether the map has 10 or 230 keyframes.  It cannot
be reduced by limiting keyframe count or map retention time.

---

## The self-regulating keyframe insertion loop

A key finding from empirical testing: **reducing `Kp/MaxFeatures` does not
proportionally reduce total compute**, because of a compensatory feedback loop:

```
fewer features per frame
       ↓
sparser local map model (fewer points per KF)
       ↓
current frame finds fewer matches in the local map
       ↓
RTAB-Map inserts a new KF to "refresh" map coverage
       ↓
more KF insertions → more slam backend invocations
       ↑────────────────────────────── cancels the odom saving
```

### Measured on freiburg1_desk (290 frames, 20 fps, ros2_raw bag)

| Preset | Features | KF rate | Odom | Slam | Total mean | Median | On-budget |
|---|---|---|---|---|---|---|---|
| default | 500 | 45 % | 31 ms | 21 ms | 58 ms | 52 ms | 44 % |
| fast | 250 | 67 % | 26 ms | 25 ms | 57 ms | 48 ms | 51 % |

Odom improved –16 % as expected.  Slam degraded +18 % because KF rate rose
from 45 % to 67 %.  Net wall-clock fps change: **+2 %** (16.80 → 17.11 fps).

`Mem/STMSize 10` (short-term memory cap) made things worse: it triggered
frequent STM→LTM eviction writes that doubled the slam spike count
(20 → 41 frames exceeding 50 ms), with no improvement in mean fps.

---

## What does work

### 1. Reduce input FPS (most reliable)

The pipeline already implements this via `--target-fps`.  At 10 fps input:

| Input fps | Frame budget | Mean total | Margin |
|---|---|---|---|
| 30 fps | 33 ms | 58–66 ms | –25 to –33 ms (impossible) |
| 20 fps | 50 ms | 58–66 ms | –8 to –16 ms (marginal) |
| 10 fps | 100 ms | 58–66 ms | **+34–42 ms comfortable** |

Only 1.7 % of frames exceed 100 ms at 10 fps input.

### 2. Use the `fast` preset for a modest median improvement

`--rtabmap-preset fast` (Kp/MaxFeatures = 250) gives:
- Median frame time: 52 → 48 ms (–8 %)
- On-budget frames: 44 % → 51 % (+7 pp)
- Wall fps: 16.8 → 17.1 fps (+2 %)
- Slam spike count increases (20 → 41 spikes > 50 ms)

Use it when you want slightly better worst-case latency distribution, not as a
path to real-time.

### 3. Accept offline non-real-time as fine for pseudo-GT

RTAB-Map is one of three candidates in an agreement system.  It runs offline on
persisted images — there is no requirement to process faster than real-time.  A
17 fps processor on a 20 fps dataset simply takes 15 % longer wall time.

---

## Runtime FPS in the summary

The `diagnostics/summary.md` **Runtime Performance** section reports these
numbers automatically from each run's log:

```
## Runtime Performance (tracking throughput)

- `rtabmap_rgbd`: **14.9 fps** | features=500 | per-step: camera=5ms, odom=26ms, slam=34ms | total=20.0s
- `orbslam3_rgbd`: **88.6 fps** | mean=11.3ms/frame | median=11.2ms/frame
- `colmap_sfm`: stages: feat=3s, match=86s, map=170s
```

COLMAP is a batch SfM method; its fps is not meaningful for real-time SLAM.
The timing numbers land in `agreement.json` under the `timing` key and in
`run_manifest.json` under each method's `metrics`.

---

## Recommendations

| Situation | Action |
|---|---|
| Need RTAB-Map to track in real-time | Use `--target-fps 10` |
| High-fps camera (30 fps RealSense) | Profile sets `rtabmap_preset: fast` automatically |
| Accuracy-critical scene (with GT) | Keep `default` preset, use `--target-fps 20` |
| Want faster overall pipeline | ORB-SLAM3 (89 fps) + COLMAP are the fast methods; RTAB-Map is the corroborating vote |
