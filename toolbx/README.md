# Toolbx notes

The main path in this repo is Compose. That is the most reliable way to pass USB devices and GUI access to a RealSense container on Fedora.

You can still create an interactive Toolbx-like container from the same image after building it:

```bash
./scripts/build.sh
toolbox create --image localhost/ros2-jazzy-realsense:latest ros2-jazzy-realsense
toolbox enter ros2-jazzy-realsense
```

Inside the toolbox:

```bash
source /opt/ros/jazzy/setup.bash
realsense-viewer
ros2 launch realsense2_camera rs_launch.py
```

Caveat: Fedora Toolbx is optimized for Fedora-based images. Ubuntu images may work for interactive development, but USB and GUI passthrough is generally less predictable than the Compose workflow in this repo. If Toolbx refuses the image, use the Compose scripts or consider Distrobox for the Ubuntu/Jazzy container.
