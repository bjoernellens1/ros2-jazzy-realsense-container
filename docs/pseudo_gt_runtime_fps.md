# Pseudo-GT pipeline: runtime FPS tracking

The pipeline records tracking throughput for each SLAM method and includes it
in every run's `diagnostics/summary.md` and `agreement.json`.

## What is measured

### ORB-SLAM3
Parsed from the binary's final summary line:
```
median tracking time: 0.0111723
mean tracking time: 0.0112887
```
`runtime_fps = 1 / mean_tracking_time`

### RTAB-Map
Parsed from the `rtabmap-rgbd_dataset` iteration log and the `rtabmap-report`
summary line:
```
Iteration 298/298: camera=6ms, odom(quality=333/958, kfs=229)=24ms, slam=19ms
Total time=20.009015s
  slam: avg=34 ms (max=167 ms) loops=17, odom: avg=26ms (max=83ms), camera: avg=5ms
```
`runtime_fps = n_frames / total_time`

Also captures: `kp_max_features`, `slam_avg_ms`, `odom_avg_ms`, `camera_avg_ms`.

### COLMAP
COLMAP is a batch SfM method, not an online SLAM tracker.  Runtime is reported
per stage:
```
feature_extraction_sec, sequential_matching_sec, mapper_sec
```
Parsed from `Elapsed time: X.XXX [minutes]` lines.  Sequential matcher emits
two elapsed-time lines (sift + geometric verification); the parser assigns the
first to extraction, sums the middle lines for matching, and takes the last for
mapper.

## Example output (freiburg1_desk, 20 fps, 298 frames)

```markdown
## Runtime Performance (tracking throughput)

- `colmap_sfm`: stages: feat=3s, match=86s, map=170s
- `orbslam3_rgbd`: **88.6 fps** | mean=11.3ms/frame | median=11.2ms/frame
- `rtabmap_rgbd`: **14.9 fps** | features=500 | per-step: camera=5ms, odom=26ms, slam=34ms | total=20.0s
```

## Interpreting the numbers

### ORB-SLAM3 at 88 fps
Mean 11 ms/frame.  Well above real-time for any camera up to ~60 fps.  The
tracking time reported excludes image loading (which the binary handles
internally via `associations.txt`).

### RTAB-Map at 15 fps
At 20 fps input the 50 ms frame budget is regularly exceeded (mean ~65 ms).
See `docs/rtabmap_runtime_performance.md` for a full analysis.  Short version:
the slam backend has a ~35 ms VBoW floor that cannot be reduced by tuning
feature counts, because RTAB-Map's keyframe insertion rate self-compensates.

### COLMAP timing
COLMAP is run once per dataset (offline batch).  Sequential matching dominates
for long sequences.  `fast` preset cuts matching via vocabulary tree rather than
exhaustive search.  Mapper time scales with scene complexity and reconstruction
size, not purely frame count.

## Where the data lives

| Location | Content |
|---|---|
| `diagnostics/summary.md` | Human-readable Runtime Performance section |
| `diagnostics/agreement.json` → `timing` | Per-method dict with all keys |
| `run_manifest.json` → `results[].metrics` | Same keys inside each method's metrics |
| `candidates/<method>/run.log` | Raw timing log to re-parse |

## Adding new methods

Implement a `parse_<method>_log_metrics(log: Path) -> dict` function in
`scripts/pseudo_gt_pipeline.py` that returns at least `runtime_fps`.  Merge
its output into the `CandidateResult.metrics` dict.  The `evaluate_agreement`
function automatically picks up any key in `_timing_keys` and propagates it to
`agreement["timing"]` and the summary.
