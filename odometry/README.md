# Modular odometry options for D435i

## Recommended first module: RTAB-Map RGB-D Odometry

Why it is the first thing to wire in:

- Available as ROS 2 Jazzy binary package.
- Works with standard RGB + aligned depth + camera info topics.
- Gives you `/odom` and TF quickly.
- Has a mature path from odometry-only to full SLAM / loop closure.
- Good enough to validate capture quality and topic timing before building a custom GS tracker.

Run:

```bash
./scripts/camera_ros.sh
./scripts/rtabmap_rgbd_odom.sh
```

For live odometry evaluation, use the conservative default camera profile:

```text
DEPTH_PROFILE=640x480x15
COLOR_PROFILE=640x480x15
ALIGN_DEPTH=true
INITIAL_RESET=true
```

This keeps raw and aligned depth publishing reliably on the tested D435i. Higher 30 Hz or 720p color profiles may publish color while depth stalls, which leaves RViz without live depth and RTAB-Map without an `odom` TF.

## Second layer: RTAB-Map full SLAM

After odometry works, add full `rtabmap_launch` mapping. Keep this separate from recording so failed SLAM does not ruin raw data capture.

Typical topic mapping:

```text
rgb_topic:         /camera/camera/color/image_raw
depth_topic:       /camera/camera/aligned_depth_to_color/image_raw
camera_info_topic: /camera/camera/color/camera_info
frame_id:          camera_link
odom_frame_id:     odom
map_frame_id:      map
```

## Research-grade alternatives to evaluate later

### ORB-SLAM3 RGB-D / RGB-D-Inertial

Good candidate for accuracy comparison and trajectory baselines, but less plug-and-play in ROS 2 Jazzy containers than RTAB-Map. It is better as a separate module once recording is stable.

### ICP / GICP odometry

Good fit for your GS-ICP-SLAM direction because it consumes depth geometry directly. For D435i handheld room data, pure ICP can struggle on planar corridors and walls unless helped by features, IMU, robust filtering, or loop closures.

### Visual-inertial odometry

The D435i has raw IMU but does not output integrated pose. IMU fusion requires proper timestamping, calibration, and a VIO backend. Keep IMU in every bag even if you do not consume it initially.

## Recommended progression

1. Record raw RGB-D-IMU bags.
2. Verify topic timing, intrinsics, aligned depth, and dropped frames.
3. Run RTAB-Map RGB-D odometry offline and online.
4. Run the containerized pseudo-GT platform:

   ```bash
   ./scripts/best_pseudo_gt_from_bag.sh --profile realsense_d435i_ros2 /path/to/bag
   ```

   For raw TUM RGB-D sequence directories, use `--profile tum_rgbd --input-format tum_rgbd --methods colmap_sfm,orbslam3_rgbd`.

5. Inspect `diagnostics/agreement.json` and only feed `best_pseudo_gt_tum.csv` into Gaussian Splatting when the agreement gate succeeds.

The pseudo-GT platform keeps heavy intermediate frames and matcher databases in container RAM by default. Add `--persist-intermediates` only when debugging extraction or a failed method.
