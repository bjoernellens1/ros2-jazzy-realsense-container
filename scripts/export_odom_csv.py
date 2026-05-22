#!/usr/bin/env python3
import argparse
import csv
import glob
import os
import queue
import shutil
import threading
import time
from pathlib import Path


import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import rosbag2_py
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Image, CameraInfo, Imu

POSE_COV_COLUMNS = [f"pose_cov_{i}" for i in range(36)]
TWIST_COV_COLUMNS = [f"twist_cov_{i}" for i in range(36)]


class OdomCsvExporter:
    def __init__(self, topic: str, tum_csv: Path, full_csv: Path, out_bag: Path) -> None:
        self.node = rclpy.create_node("odom_csv_exporter")
        self.odom_topic = topic
        self.tum_file = tum_csv.open("w", newline="")
        self.full_file = full_csv.open("w", newline="")
        self.tum_writer = csv.writer(self.tum_file)
        self.full_writer = csv.writer(self.full_file)
        self.count = 0
        self.last_msg_time = None
        self.camera_pid = None
        self.bag_fd = None
        self.total_bag_size = 0


        self.tum_writer.writerow(["timestamp", "tx", "ty", "tz", "qx", "qy", "qz", "qw"])
        self.full_writer.writerow(
            [
                "timestamp",
                "frame_id",
                "child_frame_id",
                "tx",
                "ty",
                "tz",
                "qx",
                "qy",
                "qz",
                "qw",
                "linear_x",
                "linear_y",
                "linear_z",
                "angular_x",
                "angular_y",
                "angular_z",
                *POSE_COV_COLUMNS,
                *TWIST_COV_COLUMNS,
            ]
        )

        # Set up rosbag2 MCAP writer
        self.out_bag = out_bag
        self.temp_bag = self.out_bag.parent / (self.out_bag.name + "_uncompressed")
        if self.temp_bag.exists():
            shutil.rmtree(self.temp_bag)

        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(
            rosbag2_py.StorageOptions(
                uri=str(self.temp_bag),
                storage_id="mcap",
                storage_preset_profile=""
            ),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr"
            )
        )

        # Initialize background writer queue & thread
        self.write_queue = queue.Queue()
        self.total_messages_written = 0
        self.write_thread = threading.Thread(target=self._write_worker, daemon=True)
        self.write_thread.start()

        # Create topic metadata
        self.writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=0,
                name=self.odom_topic,
                type="nav_msgs/msg/Odometry",
                serialization_format="cdr"
            )
        )
        self.writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=1,
                name="/tf",
                type="tf2_msgs/msg/TFMessage",
                serialization_format="cdr"
            )
        )
        self.writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=2,
                name="/tf_static",
                type="tf2_msgs/msg/TFMessage",
                serialization_format="cdr"
            )
        )
        self.writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=3,
                name="/camera/camera/color/image_raw",
                type="sensor_msgs/msg/Image",
                serialization_format="cdr"
            )
        )
        self.writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=4,
                name="/camera/camera/aligned_depth_to_color/image_raw",
                type="sensor_msgs/msg/Image",
                serialization_format="cdr"
            )
        )
        self.writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=5,
                name="/camera/camera/color/camera_info",
                type="sensor_msgs/msg/CameraInfo",
                serialization_format="cdr"
            )
        )
        self.writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=6,
                name="/camera/camera/depth/camera_info",
                type="sensor_msgs/msg/CameraInfo",
                serialization_format="cdr"
            )
        )
        self.writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=7,
                name="/camera/camera/imu",
                type="sensor_msgs/msg/Imu",
                serialization_format="cdr"
            )
        )
        self.writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=8,
                name="/camera/camera/accel/sample",
                type="sensor_msgs/msg/Imu",
                serialization_format="cdr"
            )
        )
        self.writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=9,
                name="/camera/camera/gyro/sample",
                type="sensor_msgs/msg/Imu",
                serialization_format="cdr"
            )
        )
        self.writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=10,
                name="/camera/camera/depth/image_rect_raw",
                type="sensor_msgs/msg/Image",
                serialization_format="cdr"
            )
        )
        self.writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=11,
                name="/camera/camera/aligned_depth_to_color/camera_info",
                type="sensor_msgs/msg/CameraInfo",
                serialization_format="cdr"
            )
        )

        self.record_ext_odom = (self.odom_topic != "/odom")
        if self.record_ext_odom:
            self.writer.create_topic(
                rosbag2_py.TopicMetadata(
                    id=12,
                    name="/odom",
                    type="nav_msgs/msg/Odometry",
                    serialization_format="cdr"
                )
            )

        # Create subscriptions
        self.subscription = self.node.create_subscription(
            Odometry, self.odom_topic, self._on_odom, 100
        )
        self.tf_sub = self.node.create_subscription(
            TFMessage, "/tf", self._on_tf, 100
        )
        tf_static_qos = QoSProfile(
            depth=100,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        self.tf_static_sub = self.node.create_subscription(
            TFMessage, "/tf_static", self._on_tf_static, tf_static_qos
        )
        self.rgb_sub = self.node.create_subscription(
            Image, "/camera/camera/color/image_raw", self._on_rgb, 10
        )
        self.depth_sub = self.node.create_subscription(
            Image, "/camera/camera/aligned_depth_to_color/image_raw", self._on_depth, 10
        )
        self.info_sub = self.node.create_subscription(
            CameraInfo, "/camera/camera/color/camera_info", self._on_info, 10
        )
        self.depth_info_sub = self.node.create_subscription(
            CameraInfo, "/camera/camera/depth/camera_info", self._on_depth_info, 10
        )
        self.raw_depth_sub = self.node.create_subscription(
            Image, "/camera/camera/depth/image_rect_raw", self._on_raw_depth, 10
        )
        self.aligned_depth_info_sub = self.node.create_subscription(
            CameraInfo, "/camera/camera/aligned_depth_to_color/camera_info", self._on_aligned_depth_info, 10
        )
        if self.record_ext_odom:
            self.ext_odom_sub = self.node.create_subscription(
                Odometry, "/odom", self._on_ext_odom, 100
            )

        imu_qos = QoSProfile(
            depth=100,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        self.imu_sub = self.node.create_subscription(
            Imu, "/camera/camera/imu", self._on_imu, imu_qos
        )
        self.accel_sub = self.node.create_subscription(
            Imu, "/camera/camera/accel/sample", self._on_accel, imu_qos
        )
        self.gyro_sub = self.node.create_subscription(
            Imu, "/camera/camera/gyro/sample", self._on_gyro, imu_qos
        )

        self.start_time = time.time()
        # Create a timer to check for playback finished and print progress
        self.timer = self.node.create_timer(1.0, self._on_timer)

    def _write_to_queue(self, topic: str, msg, stamp_ns: int) -> None:
        self.write_queue.put((topic, serialize_message(msg), stamp_ns))

    def _write_worker(self) -> None:
        while True:
            item = self.write_queue.get()
            if item is None:
                self.write_queue.task_done()
                break
            topic, serialized_msg, stamp_ns = item
            try:
                self.writer.write(topic, serialized_msg, stamp_ns)
                self.total_messages_written += 1
            except Exception as e:
                self.node.get_logger().error(f"Error writing message to bag: {e}")
            self.write_queue.task_done()

    def _compress_bag_offline(self) -> None:
        self.node.get_logger().info(f"Offline post-processing: Compressing bag to {self.out_bag}...")
        
        if not self.temp_bag.exists():
            self.node.get_logger().error(f"Temporary uncompressed bag not found at {self.temp_bag}")
            return

        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(self.temp_bag), storage_id="mcap"),
            rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
        )

        if self.out_bag.exists():
            shutil.rmtree(self.out_bag)

        writer = rosbag2_py.SequentialWriter()
        writer.open(
            rosbag2_py.StorageOptions(
                uri=str(self.out_bag),
                storage_id="mcap",
                storage_preset_profile="zstd_small"
            ),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr"
            )
        )

        # Register all topics
        for topic_meta in reader.get_all_topics_and_types():
            writer.create_topic(topic_meta)

        total_msgs = max(1, self.total_messages_written)
        compressed_count = 0
        self.node.get_logger().info(f"Starting compression of {total_msgs} messages...")
        
        last_progress_time = time.time()
        while reader.has_next():
            topic, data, t = reader.read_next()
            writer.write(topic, data, t)
            compressed_count += 1
            
            current_time = time.time()
            if compressed_count % 500 == 0 or compressed_count == total_msgs or (current_time - last_progress_time) > 2.0:
                last_progress_time = current_time
                fraction = compressed_count / total_msgs
                percentage = fraction * 100.0
                bar_width = 20
                filled_width = int(fraction * bar_width)
                bar = "█" * filled_width + "░" * (bar_width - filled_width)
                self.node.get_logger().info(
                    f"Compression Progress: [{bar}] {percentage:.1f}% ({compressed_count}/{total_msgs})"
                )

        del writer
        del reader

        self.node.get_logger().info("Compression complete. Cleaning up temporary uncompressed bag...")
        shutil.rmtree(self.temp_bag)
        self.node.get_logger().info("Cleanup finished. Offline pipeline completed successfully.")

    def _kill_heavy_nodes(self) -> None:
        self.node.get_logger().info("Terminating realsense2_camera and rgbd_odometry to free up resources for compression...")
        import signal
        killed_count = 0
        try:
            for pid_dir in glob.glob("/proc/[0-9]*"):
                pid_str = os.path.basename(pid_dir)
                if not pid_str.isdigit():
                    continue
                pid = int(pid_str)
                try:
                    with open(os.path.join(pid_dir, "cmdline"), "r") as f:
                        cmdline = f.read()
                    if "realsense2_camera" in cmdline or "rgbd_odometry" in cmdline:
                        self.node.get_logger().info(f"Killing process {pid}...")
                        os.kill(pid, signal.SIGTERM)
                        killed_count += 1
                except Exception:
                    pass
        except Exception as e:
            self.node.get_logger().error(f"Error terminating heavy processes: {e}")
        self.node.get_logger().info(f"Terminated {killed_count} processes.")

    def close(self) -> None:
        self.node.get_logger().info("Closing database and shutting down worker thread...")
        self.write_queue.put(None)
        self.write_thread.join()

        self.tum_file.flush()
        self.full_file.flush()
        self.tum_file.close()
        self.full_file.close()
        
        del self.writer  # Close temporary bag writer
        
        # Kill heavy nodes before compression to release CPU/GPU
        self._kill_heavy_nodes()
        
        self._compress_bag_offline()
        self.node.destroy_node()

    def _on_timer(self) -> None:
        self._check_timeout()
        self._print_progress()

    def _check_timeout(self) -> None:
        if self.last_msg_time is not None:
            elapsed = (self.node.get_clock().now() - self.last_msg_time).nanoseconds / 1_000_000_000.0
            if elapsed > 15.0:
                self.node.get_logger().info(f"No odometry messages received for {elapsed:.1f}s. Assuming playback finished. Exiting...")
                rclpy.shutdown()

    def _get_bag_playback_progress(self):
        if self.camera_pid is not None and self.bag_fd is not None:
            try:
                fdinfo_path = f"/proc/{self.camera_pid}/fdinfo/{self.bag_fd}"
                pos = 0
                with open(fdinfo_path, "r") as info_f:
                    for line in info_f:
                        if line.startswith("pos:"):
                            pos = int(line.split()[1])
                            break
                return pos, self.total_bag_size, None
            except Exception:
                self.camera_pid = None
                self.bag_fd = None

        try:
            for pid_dir in glob.glob("/proc/[0-9]*"):
                pid = os.path.basename(pid_dir)
                try:
                    with open(os.path.join(pid_dir, "cmdline"), "r") as f:
                        cmdline = f.read()
                    if "realsense2_camera" in cmdline:
                        fd_dir = os.path.join(pid_dir, "fd")
                        if os.path.exists(fd_dir):
                            for fd in os.listdir(fd_dir):
                                fd_path = os.path.join(fd_dir, fd)
                                try:
                                    target = os.readlink(fd_path)
                                    if target.lower().endswith(".bag"):
                                        self.camera_pid = pid
                                        self.bag_fd = fd
                                        self.total_bag_size = os.path.getsize(target)
                                        fdinfo_path = os.path.join(pid_dir, "fdinfo", fd)
                                        pos = 0
                                        with open(fdinfo_path, "r") as info_f:
                                            for line in info_f:
                                                if line.startswith("pos:"):
                                                    pos = int(line.split()[1])
                                                    break
                                        return pos, self.total_bag_size, target
                                except Exception:
                                    pass
                except Exception:
                    pass
        except Exception:
            pass
        return None


    def _print_progress(self) -> None:
        progress_info = self._get_bag_playback_progress()
        if progress_info is None:
            return

        pos, total_size, bag_path = progress_info
        if total_size == 0:
            return

        fraction = pos / total_size
        percentage = fraction * 100.0

        elapsed = time.time() - self.start_time
        if fraction > 0.01:
            total_est = elapsed / fraction
            remaining = total_est - elapsed
            remaining_str = f"{int(remaining)}s"
        else:
            remaining_str = "Calculating..."

        bar_width = 20
        filled_width = int(fraction * bar_width)
        bar = "█" * filled_width + "░" * (bar_width - filled_width)

        pos_gb = pos / (1024 * 1024 * 1024)
        total_gb = total_size / (1024 * 1024 * 1024)

        self.node.get_logger().info(
            f"Progress: [{bar}] {percentage:.1f}% ({pos_gb:.2f} GB / {total_gb:.2f} GB) | Est. Remaining: {remaining_str}"
        )


    def _on_rgb(self, msg: Image) -> None:
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self._write_to_queue("/camera/camera/color/image_raw", msg, stamp_ns)

    def _on_depth(self, msg: Image) -> None:
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self._write_to_queue("/camera/camera/aligned_depth_to_color/image_raw", msg, stamp_ns)

    def _on_info(self, msg: CameraInfo) -> None:
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self._write_to_queue("/camera/camera/color/camera_info", msg, stamp_ns)

    def _on_depth_info(self, msg: CameraInfo) -> None:
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self._write_to_queue("/camera/camera/depth/camera_info", msg, stamp_ns)

    def _on_imu(self, msg: Imu) -> None:
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self._write_to_queue("/camera/camera/imu", msg, stamp_ns)

    def _on_accel(self, msg: Imu) -> None:
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self._write_to_queue("/camera/camera/accel/sample", msg, stamp_ns)

    def _on_gyro(self, msg: Imu) -> None:
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self._write_to_queue("/camera/camera/gyro/sample", msg, stamp_ns)

    def _on_raw_depth(self, msg: Image) -> None:
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self._write_to_queue("/camera/camera/depth/image_rect_raw", msg, stamp_ns)

    def _on_aligned_depth_info(self, msg: CameraInfo) -> None:
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self._write_to_queue("/camera/camera/aligned_depth_to_color/camera_info", msg, stamp_ns)

    def _on_ext_odom(self, msg: Odometry) -> None:
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self._write_to_queue("/odom", msg, stamp_ns)

    def _on_odom(self, msg: Odometry) -> None:
        self.last_msg_time = self.node.get_clock().now()

        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec / 1_000_000_000.0
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        # Write to bag
        self._write_to_queue(self.odom_topic, msg, stamp_ns)

        # Write to CSV files
        pose = msg.pose.pose
        twist = msg.twist.twist

        self.tum_writer.writerow(
            [
                f"{stamp_sec:.9f}",
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        )
        self.full_writer.writerow(
            [
                f"{stamp_sec:.9f}",
                msg.header.frame_id,
                msg.child_frame_id,
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
                twist.linear.x,
                twist.linear.y,
                twist.linear.z,
                twist.angular.x,
                twist.angular.y,
                twist.angular.z,
                *msg.pose.covariance,
                *msg.twist.covariance,
            ]
        )
        self.count += 1
        if self.count % 100 == 0:
            self.tum_file.flush()
            self.full_file.flush()

    def _on_tf(self, msg: TFMessage) -> None:
        if msg.transforms:
            stamp = msg.transforms[0].header.stamp
            stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        else:
            stamp_ns = self.node.get_clock().now().nanoseconds
        self._write_to_queue("/tf", msg, stamp_ns)

    def _on_tf_static(self, msg: TFMessage) -> None:
        if msg.transforms:
            stamp = msg.transforms[0].header.stamp
            stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        else:
            stamp_ns = self.node.get_clock().now().nanoseconds
        self._write_to_queue("/tf_static", msg, stamp_ns)


def main() -> None:
    parser = argparse.ArgumentParser(description="Export nav_msgs/Odometry to TUM, full CSV files, and timesynced MCAP bag.")
    parser.add_argument("--topic", default="/rtabmap/odom")
    parser.add_argument("--tum-csv", required=True, type=Path)
    parser.add_argument("--full-csv", required=True, type=Path)
    parser.add_argument("--out-bag", required=True, type=Path)
    args = parser.parse_args()

    args.tum_csv.parent.mkdir(parents=True, exist_ok=True)
    args.full_csv.parent.mkdir(parents=True, exist_ok=True)
    args.out_bag.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    exporter = OdomCsvExporter(args.topic, args.tum_csv, args.full_csv, args.out_bag)
    try:
        rclpy.spin(exporter.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        exporter.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
