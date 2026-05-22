# Fedora host notes

## USB

Check the D435i:

```bash
lsusb | grep -i -E '8086|realsense'
```

For the D435i, prefer a real USB 3 port and a known-good USB 3 cable. If the camera falls back to USB 2, high-resolution color + depth modes may fail or drop frames.

## GUI

Run:

```bash
./scripts/allow_gui.sh
```

This grants local root containers access to your X server. On Wayland sessions this usually goes through XWayland.

## Podman vs Docker

Use Podman first on Fedora:

```bash
CONTAINER_ENGINE=podman ./scripts/build.sh
```

Use Docker if USB permissioning with rootless Podman gets annoying:

```bash
CONTAINER_ENGINE=docker ./scripts/build.sh
```

## If RealSense Viewer cannot see the device

Try:

```bash
fuser -v /dev/bus/usb/*/* 2>/dev/null | grep -i realsense
```

Then close any other RealSense Viewer, ROS driver, or process using the camera.
