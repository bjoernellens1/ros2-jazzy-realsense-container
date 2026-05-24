#!/usr/bin/env python3
import argparse
import csv
import sys
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec / 1_000_000_000.0


class OdomTumRecorder:
    def __init__(
        self,
        topic: str,
        tum_path: Path,
        full_path: Path | None,
        startup_timeout: float,
        idle_timeout: float,
    ) -> None:
        self.node = rclpy.create_node("pseudo_gt_odom_tum_recorder")
        self.topic = topic
        self.startup_timeout = startup_timeout
        self.idle_timeout = idle_timeout
        self.count = 0
        self.first_wall = self.node.get_clock().now()
        self.last_msg_wall = None

        tum_path.parent.mkdir(parents=True, exist_ok=True)
        self.tum_file = tum_path.open("w", newline="")
        self.tum_writer = csv.writer(self.tum_file, delimiter=" ")

        self.full_file = None
        self.full_writer = None
        if full_path is not None:
            full_path.parent.mkdir(parents=True, exist_ok=True)
            self.full_file = full_path.open("w", newline="")
            self.full_writer = csv.writer(self.full_file)
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
                ]
            )

        self.sub = self.node.create_subscription(Odometry, topic, self._on_odom, 200)
        self.timer = self.node.create_timer(1.0, self._on_timer)

    def _on_odom(self, msg: Odometry) -> None:
        self.last_msg_wall = self.node.get_clock().now()
        pose = msg.pose.pose
        twist = msg.twist.twist
        ts = stamp_to_sec(msg.header.stamp)
        self.tum_writer.writerow(
            [
                f"{ts:.9f}",
                f"{pose.position.x:.9f}",
                f"{pose.position.y:.9f}",
                f"{pose.position.z:.9f}",
                f"{pose.orientation.x:.9f}",
                f"{pose.orientation.y:.9f}",
                f"{pose.orientation.z:.9f}",
                f"{pose.orientation.w:.9f}",
            ]
        )
        if self.full_writer is not None:
            self.full_writer.writerow(
                [
                    f"{ts:.9f}",
                    msg.header.frame_id,
                    msg.child_frame_id,
                    f"{pose.position.x:.9f}",
                    f"{pose.position.y:.9f}",
                    f"{pose.position.z:.9f}",
                    f"{pose.orientation.x:.9f}",
                    f"{pose.orientation.y:.9f}",
                    f"{pose.orientation.z:.9f}",
                    f"{pose.orientation.w:.9f}",
                    f"{twist.linear.x:.9f}",
                    f"{twist.linear.y:.9f}",
                    f"{twist.linear.z:.9f}",
                    f"{twist.angular.x:.9f}",
                    f"{twist.angular.y:.9f}",
                    f"{twist.angular.z:.9f}",
                ]
            )
        self.count += 1
        if self.count % 100 == 0:
            self.tum_file.flush()
            if self.full_file is not None:
                self.full_file.flush()

    def _on_timer(self) -> None:
        now = self.node.get_clock().now()
        if self.count == 0:
            elapsed = (now - self.first_wall).nanoseconds / 1_000_000_000.0
            if elapsed > self.startup_timeout:
                self.node.get_logger().error(
                    f"No odometry received on {self.topic} after {elapsed:.1f}s"
                )
                rclpy.shutdown()
            return

        idle = (now - self.last_msg_wall).nanoseconds / 1_000_000_000.0
        if idle > self.idle_timeout:
            self.node.get_logger().info(
                f"No odometry for {idle:.1f}s; recorded {self.count} poses."
            )
            rclpy.shutdown()

    def close(self) -> None:
        self.tum_file.flush()
        self.tum_file.close()
        if self.full_file is not None:
            self.full_file.flush()
            self.full_file.close()
        self.node.destroy_node()


def main() -> int:
    parser = argparse.ArgumentParser(description="Record nav_msgs/Odometry to TUM format.")
    parser.add_argument("--topic", default="/rtabmap/odom")
    parser.add_argument("--tum", required=True, type=Path)
    parser.add_argument("--full-csv", type=Path)
    parser.add_argument("--startup-timeout", default=60.0, type=float)
    parser.add_argument("--idle-timeout", default=15.0, type=float)
    args = parser.parse_args()

    rclpy.init()
    recorder = OdomTumRecorder(
        args.topic,
        args.tum,
        args.full_csv,
        args.startup_timeout,
        args.idle_timeout,
    )
    try:
        rclpy.spin(recorder.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        count = recorder.count
        recorder.close()
        if rclpy.ok():
            rclpy.shutdown()

    if count == 0:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
