# ROS 2 Jazzy RealSense D435i on Fedora

A containerized RealSense starter repo for Fedora hosts. It gives you one Compose-based workflow for:

- Intel RealSense D435i via `realsense2_camera`
- librealsense2 / RealSense Viewer
- RViz2 GUI support
- easy RGB-D-IMU rosbag recording
- optional RTAB-Map RGB-D odometry as the first modular SLAM/odometry layer
- offline pseudo-GT estimation from RealSense and Orbbec RGB-D bags with RTAB-Map, COLMAP, and ORB-SLAM3 agreement checks

The intended host is Fedora KDE/GNOME with Podman or Docker. The container image is Ubuntu 24.04 + ROS 2 Jazzy, because Jazzy and the official ROS packages are native there.

The image installs the Jazzy RealSense wrapper, librealsense2 runtime, and RealSense tooling when those packages are available for Jazzy.

## Layout

```text
.
├── compose.yaml                    # one compose file for Docker Compose and Podman Compose
├── Containerfile                   # Ubuntu 24.04 + ROS 2 Jazzy + RealSense + RViz + RTAB-Map
├── .env.example                    # host/runtime knobs
├── scripts/
│   ├── compose_cmd.sh              # picks docker compose or podman compose
│   ├── build.sh
│   ├── shell.sh
│   ├── allow_gui.sh
│   ├── viewer.sh                   # RealSense Viewer
│   ├── camera_ros.sh               # D435i ROS driver
│   ├── rviz.sh
│   ├── list_topics.sh
│   ├── set_preset.sh
│   ├── record_rgbd_imu.sh          # launches camera + records useful topics
│   ├── record_existing_camera.sh   # records topics from an already running camera
│   ├── play_bag.sh
│   └── rtabmap_rgbd_odom.sh        # optional odometry module
├── config/
│   ├── d435i.env
│   └── presets.md
├── rviz/
│   └── realsense_rgbd.rviz
├── odometry/
│   └── README.md
└── toolbx/
    └── README.md
```

## Fedora host prerequisites

For Podman:

```bash
sudo dnf install -y podman podman-compose xorg-x11-xhost usbutils
```

For Docker:

```bash
sudo dnf install -y docker docker-compose-plugin xorg-x11-xhost usbutils
sudo systemctl enable --now docker
```

Check that the camera appears on the host:

```bash
lsusb | grep -i -E 'realsense|8086'
```

For rootless Podman, USB device access can be stricter than Docker. The most reliable first test is privileged compose with `/dev` mounted, as done here. After it works, you can tighten permissions.

## Quick start

```bash
git clone <this-repo>
cd ros2-jazzy-realsense-fedora
cp .env.example .env

./scripts/allow_gui.sh
./scripts/build.sh
```

Build the optional CUDA pseudo-GT image for NVIDIA cluster nodes:

```bash
./scripts/build.sh --cuda
```

Start RealSense Viewer:

```bash
./scripts/viewer.sh
```

Start the ROS camera driver:

```bash
./scripts/camera_ros.sh
```

In another terminal, inspect topics:

```bash
./scripts/list_topics.sh
```

Open RViz2:

```bash
./scripts/rviz.sh
```

Open the RTAB-Map odometry RViz profile:

```bash
./scripts/rviz_rtabmap.sh
```

Record RGB-D-IMU data:

```bash
./scripts/record_rgbd_imu.sh my_room_loop_01
```

The bag will be written under `./bags/my_room_loop_01`.

## Recommended D435i recording baseline for rooms / floors

The defaults in `config/d435i.env` are chosen for handheld room capture:

```text
Depth: 640x480 @ 15 Hz
Color: 640x480 @ 15 Hz
Aligned depth: enabled
Hardware reset: enabled on camera startup
IMU: enabled
Preset: Medium Density by default, changeable after launch
Point cloud: disabled by default for recording
```

Why point cloud disabled? It explodes bag size and is reproducible later from aligned depth + intrinsics.

Why 15 Hz VGA by default? It is the conservative live RGB-D odometry profile for this container workflow. Higher color/depth profiles can work, but on this D435i setup they caused missing or stalled depth frames and RTAB-Map lost the `odom` frame.

## Change RealSense visual preset

With the camera driver running:

```bash
./scripts/set_preset.sh "Medium Density"
./scripts/set_preset.sh "High Accuracy"
./scripts/set_preset.sh "High Density"
```

If the wrapper exposes the preset under a slightly different parameter name, the script prints all matching preset parameters so you can adjust quickly.

## Optional odometry module: RTAB-Map RGB-D odometry

Start the camera first:

```bash
./scripts/camera_ros.sh
```

Then in another terminal:

```bash
./scripts/rtabmap_rgbd_odom.sh
```

This starts RTAB-Map RGB-D odometry against the RealSense RGB + aligned depth topics and publishes odometry. Treat this as the simplest robust module to add first, not as your final research tracker.

## Offline best pseudo-GT estimation

The pseudo-GT workflow is container-only. The host script only resolves input/output paths and then runs Compose. By default, heavy intermediates are created under container shared memory, so extracted frames, COLMAP databases, temporary sparse models, ORB-SLAM3 scratch files, and RTAB-Map scratch logs do not fill the repository.

RealSense SDK ROS1 bag:

```bash
./scripts/best_pseudo_gt_from_bag.sh \
  --profile realsense_d435i_ros1 \
  --max-frames 500 \
  --colmap-preset fast \
  --force \
  /path/to/recording.bag
```

RealSense or Orbbec ROS2 MCAP/rosbag2:

```bash
./scripts/best_pseudo_gt_from_bag.sh \
  --profile orbbec_femto_bolt_ros2 \
  --methods rtabmap_rgbd,rtabmap_rgbd_imu,colmap_sfm,orbslam3_rgbd \
  --workspace-mode ram \
  --force \
  /path/to/rosbag2_dir_or_file.mcap
```

Raw TUM RGB-D sequence directory:

```bash
./scripts/best_pseudo_gt_from_bag.sh \
  --profile tum_rgbd \
  --input-format tum_rgbd \
  --methods colmap_sfm,orbslam3_rgbd \
  --max-frames 300 \
  --force \
  /path/to/rgbd_dataset_freiburg1_xyz
```

CUDA path for NVIDIA machines:

```bash
./scripts/best_pseudo_gt_from_bag.sh --cuda \
  --profile realsense_d435i_ros2 \
  --colmap-preset robust \
  --force \
  /path/to/rosbag2_dir
```

The output directory defaults to `<input>_pseudo_gt` and contains only the compact result bundle:

```text
best_pseudo_gt_tum.csv              # only when agreement succeeds
candidates/<method>/trajectory_tum.csv
candidates/<method>/run.log
diagnostics/agreement.json
diagnostics/pairwise_agreement.csv
diagnostics/summary.md
diagnostics/trajectory_xy.png
diagnostics/coverage.png
run_manifest.json
```

Reliability is agreement-gated. At least two healthy methods must agree with the moderate default gate before `best_pseudo_gt_tum.csv` is written. If no pair agrees, the run exits nonzero, writes diagnostics, and does not call the result reliable. Use `--allow-unreliable-best` only when you explicitly want a low-confidence fallback file named `candidate_best_unreliable_tum.csv`.

Use `--persist-intermediates` to copy the normalized dataset and scratch workspace to disk for debugging. Use `PSEUDO_GT_SHM_SIZE=64gb` with Compose if the default shared-memory workspace is too small.

## Notes for Gaussian Splatting capture

For GS reconstruction, the most valuable outputs are:

- RGB image
- aligned depth image
- RGB camera info
- raw depth camera info, if you want to redo alignment
- IMU, for later VIO/fusion experiments
- `/tf_static`, for camera frame geometry
- eventual external odometry / trajectory

For high-quality GS data, move slowly, avoid pure wall-only motion, close loops, keep exposure stable, and record short calibration/test loops with `Medium Density`, `High Accuracy`, and `Default` before doing long floor recordings.

## Known caveats

- The D435i does **not** provide built-in 6-DoF pose like a T265. It gives RGB, stereo depth, and IMU.
- RealSense inside containers is easiest with privileged device passthrough. This repo prioritizes reliable lab use first.
- On Wayland, RViz and RealSense Viewer usually still work through XWayland via `DISPLAY` + `/tmp/.X11-unix`.
- If RealSense Viewer cannot open the camera, close all ROS camera nodes first. Only one process should own the device unless the SDK backend supports your exact shared access mode.
