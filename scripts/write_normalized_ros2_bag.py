#!/usr/bin/env python3
"""Write a ROS2 mcap bag from a normalized pseudo-GT dataset directory.

The normalized dataset is expected to contain:
  frames.csv          - index, timestamp, rgb_file, depth_file, ...
  images/             - RGB PNGs (uint8 BGR, written by OpenCV)
  depth/              - depth PNGs (uint16, depth_factor applied)
  camera_info.json    - fx, fy, cx, cy, width, height, depth_factor, d[]
  imu.csv             - (optional) t ax ay az gx gy gz  (TUM-style)

The produced bag contains synchronized Image + CameraInfo messages at the
original dataset timestamps, suitable for ros2 bag play -> RTAB-Map.
Depth is always published as 16UC1 with 1 unit = 1 mm (ROS REP 117),
rescaling from the source depth_factor if needed.
"""

import argparse
import csv
import json
import sys
from pathlib import Path

import cv2
import numpy as np
from builtin_interfaces.msg import Time
from rclpy.serialization import serialize_message
from sensor_msgs.msg import CameraInfo, Imu, Image


def make_stamp(ts: float) -> Time:
    stamp = Time()
    stamp.sec = int(ts)
    stamp.nanosec = int((ts - stamp.sec) * 1e9)
    return stamp


def make_image(ts: float, frame_id: str, encoding: str, data: np.ndarray) -> Image:
    msg = Image()
    msg.header.stamp = make_stamp(ts)
    msg.header.frame_id = frame_id
    msg.height = data.shape[0]
    msg.width = data.shape[1]
    msg.encoding = encoding
    msg.is_bigendian = False
    msg.step = data.strides[0]
    msg.data = data.tobytes()
    return msg


def make_camera_info(ts: float, frame_id: str, height: int, width: int,
                     fx: float, fy: float, cx: float, cy: float,
                     distortion: list) -> CameraInfo:
    msg = CameraInfo()
    msg.header.stamp = make_stamp(ts)
    msg.header.frame_id = frame_id
    msg.height = height
    msg.width = width
    msg.distortion_model = "plumb_bob"
    d = list(distortion) + [0.0] * max(0, 5 - len(distortion))
    msg.d = d[:5]
    msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return msg


def make_imu(ts: float, frame_id: str,
             ax: float, ay: float, az: float,
             gx: float, gy: float, gz: float) -> Imu:
    msg = Imu()
    msg.header.stamp = make_stamp(ts)
    msg.header.frame_id = frame_id
    msg.linear_acceleration.x = ax
    msg.linear_acceleration.y = ay
    msg.linear_acceleration.z = az
    msg.angular_velocity.x = gx
    msg.angular_velocity.y = gy
    msg.angular_velocity.z = gz
    msg.orientation_covariance[0] = -1.0  # orientation unknown
    return msg


def write_bag(
    dataset: Path,
    output: Path,
    rgb_topic: str,
    depth_topic: str,
    info_topic: str,
    frame_id: str,
    imu_topic: str | None = None,
) -> None:
    import rosbag2_py

    frames_csv = dataset / "frames.csv"
    camera_info_json = dataset / "camera_info.json"
    imu_csv = dataset / "imu.csv"
    if not frames_csv.exists():
        sys.exit(f"frames.csv not found in {dataset}")
    if not camera_info_json.exists():
        sys.exit(f"camera_info.json not found in {dataset}")

    with camera_info_json.open() as fh:
        ci = json.load(fh)
    fx = float(ci["fx"])
    fy = float(ci["fy"])
    cx = float(ci["cx"])
    cy = float(ci["cy"])
    height = int(ci["height"])
    width = int(ci["width"])
    distortion = [float(v) for v in ci.get("d", [0.0, 0.0, 0.0, 0.0, 0.0])]
    # depth_factor: units per metre in the stored PNGs. ROS REP 117 uses 1000 (mm).
    depth_factor = float(ci.get("depth_factor", 1000.0))

    with frames_csv.open(newline="") as fh:
        reader = csv.DictReader(fh)
        frames = list(reader)

    # Load IMU data if present and a topic was requested
    imu_rows: list[tuple[float, float, float, float, float, float, float]] = []
    if imu_topic and imu_csv.exists():
        with imu_csv.open(newline="") as fh:
            for line in fh:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split()
                if len(parts) >= 7:
                    imu_rows.append(tuple(float(p) for p in parts[:7]))
        print(f"[bag] loaded {len(imu_rows)} IMU samples from {imu_csv}")

    output.parent.mkdir(parents=True, exist_ok=True)
    storage_options = rosbag2_py.StorageOptions(uri=str(output), storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions("", "")
    writer = rosbag2_py.SequentialWriter()
    writer.open(storage_options, converter_options)

    topics = [
        (rgb_topic, "sensor_msgs/msg/Image"),
        (depth_topic, "sensor_msgs/msg/Image"),
        (info_topic, "sensor_msgs/msg/CameraInfo"),
    ]
    if imu_topic and imu_rows:
        topics.append((imu_topic, "sensor_msgs/msg/Imu"))
    for idx, (topic, msg_type) in enumerate(topics):
        meta = rosbag2_py.TopicMetadata(
            id=idx, name=topic, type=msg_type, serialization_format="cdr"
        )
        writer.create_topic(meta)

    # Interleave IMU and image messages by timestamp
    imu_idx = 0
    written = 0
    for row in frames:
        ts = float(row["timestamp"])
        ts_ns = int(ts * 1e9)

        # Flush any IMU samples that precede this image frame
        while imu_topic and imu_rows and imu_idx < len(imu_rows):
            imu_ts = imu_rows[imu_idx][0]
            if imu_ts > ts:
                break
            _, ax, ay, az, gx, gy, gz = imu_rows[imu_idx]
            imu_msg = make_imu(imu_ts, frame_id, ax, ay, az, gx, gy, gz)
            writer.write(imu_topic, serialize_message(imu_msg), int(imu_ts * 1e9))
            imu_idx += 1

        rgb_path = dataset / row["rgb_file"]
        depth_path = dataset / row["depth_file"]

        bgr = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
        if bgr is None:
            print(f"[bag] skipping {rgb_path} (unreadable)", file=sys.stderr)
            continue
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

        depth = cv2.imread(str(depth_path), cv2.IMREAD_ANYDEPTH)
        if depth is None:
            print(f"[bag] skipping {depth_path} (unreadable)", file=sys.stderr)
            continue

        # Rescale to ROS REP 117 standard: 16UC1 in mm (1 unit = 1 mm)
        if depth_factor != 1000.0:
            depth = np.clip(
                depth.astype(np.float32) * (1000.0 / depth_factor), 0, 65535
            ).astype(np.uint16)

        rgb_msg = make_image(ts, frame_id, "rgb8", rgb)
        depth_msg = make_image(ts, frame_id, "16UC1", depth)
        info_msg = make_camera_info(ts, frame_id, height, width, fx, fy, cx, cy, distortion)

        writer.write(rgb_topic, serialize_message(rgb_msg), ts_ns)
        writer.write(depth_topic, serialize_message(depth_msg), ts_ns)
        writer.write(info_topic, serialize_message(info_msg), ts_ns)
        written += 1

    # Flush remaining IMU samples after the last image
    while imu_topic and imu_rows and imu_idx < len(imu_rows):
        imu_ts = imu_rows[imu_idx][0]
        _, ax, ay, az, gx, gy, gz = imu_rows[imu_idx]
        imu_msg = make_imu(imu_ts, frame_id, ax, ay, az, gx, gy, gz)
        writer.write(imu_topic, serialize_message(imu_msg), int(imu_ts * 1e9))
        imu_idx += 1

    del writer
    print(f"[bag] wrote {written} frames → {output}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("output", type=Path, help="Output mcap bag directory")
    parser.add_argument("--rgb-topic", default="/camera/rgb/image_color")
    parser.add_argument("--depth-topic", default="/camera/depth/image")
    parser.add_argument("--info-topic", default="/camera/rgb/camera_info")
    parser.add_argument("--frame-id", default="camera_rgb_optical_frame")
    parser.add_argument("--imu-topic", default=None, help="Publish imu.csv on this topic (optional)")
    args = parser.parse_args()
    write_bag(args.dataset, args.output, args.rgb_topic, args.depth_topic, args.info_topic,
              args.frame_id, imu_topic=args.imu_topic)
    return 0


if __name__ == "__main__":
    sys.exit(main())
