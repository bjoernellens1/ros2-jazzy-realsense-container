#!/usr/bin/env python3
from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


DEFAULT_METHODS = "rtabmap_rgbd,rtabmap_rgbd_imu,colmap_sfm,orbslam3_rgbd"
VALID_METHODS = {"rtabmap_rgbd", "rtabmap_rgbd_imu", "colmap_sfm", "orbslam3_rgbd"}
TUM_FREIBURG_INTRINSICS = {
    "freiburg1": {"fx": 517.3, "fy": 516.5, "cx": 318.6, "cy": 255.3},
    "fr1": {"fx": 517.3, "fy": 516.5, "cx": 318.6, "cy": 255.3},
    "freiburg2": {"fx": 520.9, "fy": 521.0, "cx": 325.1, "cy": 249.7},
    "fr2": {"fx": 520.9, "fy": 521.0, "cx": 325.1, "cy": 249.7},
    "freiburg3": {"fx": 535.4, "fy": 539.2, "cx": 320.1, "cy": 247.6},
    "fr3": {"fx": 535.4, "fy": 539.2, "cx": 320.1, "cy": 247.6},
}


@dataclass
class CandidateResult:
    method: str
    status: str
    trajectory: Path | None
    log: Path | None
    metrics: dict[str, Any]
    reason: str = ""


def run(cmd: list[str], log: Path | None = None, env: dict[str, str] | None = None, cwd: Path | None = None) -> int:
    if log is None:
        return subprocess.run(cmd, env=env, cwd=cwd, check=False).returncode
    log.parent.mkdir(parents=True, exist_ok=True)
    with log.open("a", encoding="utf-8") as fh:
        fh.write("+ " + " ".join(cmd) + "\n")
        fh.flush()
        proc = subprocess.run(cmd, env=env, cwd=cwd, stdout=fh, stderr=subprocess.STDOUT, check=False)
        fh.write(f"[exit] {proc.returncode}\n")
        return proc.returncode


def load_yaml(path: Path) -> dict[str, Any]:
    import yaml

    with path.open("r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def load_profile(config: Path, name: str) -> dict[str, Any]:
    data = load_yaml(config)
    profiles = data.get("profiles", {})
    if name not in profiles:
        raise SystemExit(f"Unknown profile '{name}'. Available: {', '.join(sorted(profiles))}")
    profile = dict(profiles[name])
    profile["name"] = name
    return profile


def parse_methods(value: str) -> list[str]:
    methods = [m.strip() for m in value.split(",") if m.strip()]
    unknown = sorted(set(methods) - VALID_METHODS)
    if unknown:
        raise SystemExit(f"Unknown method(s): {', '.join(unknown)}")
    return methods


def choose_topic(available: set[str], candidates: list[str], required: bool = True) -> str | None:
    for topic in candidates:
        if topic in available:
            return topic
    if required:
        raise RuntimeError(f"None of the topic candidates were found: {', '.join(candidates)}")
    return None


def stamp_to_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def ns_to_sec(ns: int) -> float:
    return ns / 1_000_000_000.0


def msg_stamp_ns(msg: Any, fallback_ns: int) -> int:
    try:
        return stamp_to_ns(msg.header.stamp)
    except Exception:
        return fallback_ns


def ensure_empty_output(path: Path, force: bool) -> None:
    if path.exists():
        if not force:
            raise SystemExit(f"Output already exists: {path}. Re-run with --force to replace it.")
        if path.is_dir():
            shutil.rmtree(path)
        else:
            path.unlink()
    path.mkdir(parents=True, exist_ok=True)


def make_workspace(output: Path, mode: str, keep: bool) -> Path:
    run_id = f"{int(time.time())}_{uuid.uuid4().hex[:8]}"
    if mode == "ram":
        root = Path(os.environ.get("PSEUDO_GT_RAM_ROOT", "/dev/shm/pseudo_gt"))
        try:
            root.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            raise SystemExit(
                f"Could not create RAM workspace at {root}: {exc}. "
                "Use --workspace-mode disk or increase container tmpfs/shm size."
            )
        usage = shutil.disk_usage(root)
        min_free_gb = float(os.environ.get("PSEUDO_GT_MIN_FREE_GB", "2"))
        if usage.free < min_free_gb * 1024**3:
            raise SystemExit(
                f"RAM workspace {root} has only {usage.free / 1024**3:.1f} GiB free. "
                "Increase PSEUDO_GT_SHM_SIZE or use --workspace-mode disk."
            )
        workspace = root / run_id
    else:
        workspace = output / "_workspace"
        if workspace.exists() and not keep:
            shutil.rmtree(workspace)
    workspace.mkdir(parents=True, exist_ok=True)
    return workspace


def cleanup_workspace(workspace: Path, keep: bool) -> None:
    if keep:
        print(f"[pseudo-gt] Keeping workspace: {workspace}")
        return
    shutil.rmtree(workspace, ignore_errors=True)


def read_rosbag_storage_id(path: Path) -> str:
    if path.is_file() and path.suffix.lower() == ".mcap":
        return "mcap"
    if path.is_file() and path.suffix.lower() == ".db3":
        return "sqlite3"
    metadata = path / "metadata.yaml"
    if metadata.exists():
        try:
            data = load_yaml(metadata)
            return data["rosbag2_bagfile_information"]["storage_identifier"]
        except Exception:
            pass
    return "mcap"


def rosbag_topics(path: Path) -> dict[str, str]:
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id=read_rosbag_storage_id(path)),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    return {meta.name: meta.type for meta in reader.get_all_topics_and_types()}


def image_to_array(msg: Any, is_depth: bool) -> np.ndarray:
    encoding = msg.encoding.lower()
    height = int(msg.height)
    width = int(msg.width)
    step = int(msg.step)
    data = memoryview(msg.data)

    if encoding in {"rgb8", "bgr8"}:
        row_width = width * 3
        arr = np.frombuffer(data, dtype=np.uint8).reshape(height, step)[:, :row_width]
        arr = arr.reshape(height, width, 3)
        if encoding == "rgb8":
            arr = arr[:, :, ::-1]
        return arr.copy()
    if encoding in {"rgba8", "bgra8"}:
        row_width = width * 4
        arr = np.frombuffer(data, dtype=np.uint8).reshape(height, step)[:, :row_width]
        arr = arr.reshape(height, width, 4)
        if encoding == "rgba8":
            arr = arr[:, :, [2, 1, 0, 3]]
        return arr[:, :, :3].copy()
    if encoding in {"mono8", "8uc1"}:
        arr = np.frombuffer(data, dtype=np.uint8).reshape(height, step)[:, :width]
        return arr.copy()
    if encoding in {"16uc1", "mono16"}:
        row_width = width * 2
        arr = np.frombuffer(data, dtype=np.uint8).reshape(height, step)[:, :row_width]
        return arr.view(np.uint16).reshape(height, width).copy()
    if encoding == "32fc1":
        row_width = width * 4
        arr = np.frombuffer(data, dtype=np.uint8).reshape(height, step)[:, :row_width]
        return arr.view(np.float32).reshape(height, width).copy()

    kind = "depth" if is_depth else "color"
    raise RuntimeError(f"Unsupported {kind} image encoding: {msg.encoding}")


def normalize_depth(depth: np.ndarray, depth_factor: float) -> np.ndarray:
    if depth.dtype == np.float32 or depth.dtype == np.float64:
        depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
        depth = np.clip(depth * depth_factor, 0, np.iinfo(np.uint16).max)
        return depth.astype(np.uint16)
    if depth.dtype == np.uint16:
        return depth
    return np.clip(depth, 0, np.iinfo(np.uint16).max).astype(np.uint16)


def blur_score(color_bgr: np.ndarray) -> float:
    import cv2

    gray = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2GRAY) if color_bgr.ndim == 3 else color_bgr
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def exposure_clip_ratio(color_bgr: np.ndarray) -> float:
    if color_bgr.ndim == 3:
        gray = color_bgr.mean(axis=2)
    else:
        gray = color_bgr
    return float(((gray <= 2) | (gray >= 253)).sum() / gray.size)


def depth_valid_ratio(depth: np.ndarray) -> float:
    return float((depth > 0).sum() / depth.size)


def write_camera_info_json(msg: Any, path: Path) -> dict[str, Any]:
    info = {
        "width": int(msg.width),
        "height": int(msg.height),
        "distortion_model": getattr(msg, "distortion_model", ""),
        "d": list(getattr(msg, "d", [])),
        "k": list(getattr(msg, "k", [])),
        "r": list(getattr(msg, "r", [])),
        "p": list(getattr(msg, "p", [])),
        "frame_id": getattr(getattr(msg, "header", None), "frame_id", ""),
    }
    if info["k"]:
        info["fx"] = float(info["k"][0])
        info["fy"] = float(info["k"][4])
        info["cx"] = float(info["k"][2])
        info["cy"] = float(info["k"][5])
    path.write_text(json.dumps(info, indent=2, sort_keys=True), encoding="utf-8")
    return info


def write_camera_info_dict(info: dict[str, Any], path: Path) -> dict[str, Any]:
    path.write_text(json.dumps(info, indent=2, sort_keys=True), encoding="utf-8")
    return info


def write_frames(
    pairs: list[tuple[int, Any, int, Any, Any]],
    dataset: Path,
    target_fps: float,
    max_frames: int,
    depth_factor: float,
) -> dict[str, Any]:
    import cv2

    images_dir = dataset / "images"
    depth_dir = dataset / "depth"
    images_dir.mkdir(parents=True, exist_ok=True)
    depth_dir.mkdir(parents=True, exist_ok=True)

    frames_csv = dataset / "frames.csv"
    quality_csv = dataset / "quality.csv"
    rgb_txt = dataset / "rgb.txt"
    depth_txt = dataset / "depth.txt"
    assoc_txt = dataset / "associations.txt"
    camera_info_json = dataset / "camera_info.json"

    last_kept_ts = -math.inf
    min_dt = 1.0 / target_fps if target_fps > 0 else 0.0
    kept = 0
    camera_info = None

    with frames_csv.open("w", newline="", encoding="utf-8") as f_frames, \
        quality_csv.open("w", newline="", encoding="utf-8") as f_quality, \
        rgb_txt.open("w", encoding="utf-8") as f_rgb, \
        depth_txt.open("w", encoding="utf-8") as f_depth, \
        assoc_txt.open("w", encoding="utf-8") as f_assoc:
        frames_writer = csv.DictWriter(
            f_frames,
            fieldnames=[
                "index",
                "timestamp",
                "rgb_file",
                "depth_file",
                "color_stamp",
                "depth_stamp",
                "stamp_delta_sec",
            ],
        )
        quality_writer = csv.DictWriter(
            f_quality,
            fieldnames=[
                "index",
                "timestamp",
                "blur_score",
                "exposure_clip_ratio",
                "depth_valid_ratio",
                "accepted",
                "reason",
            ],
        )
        frames_writer.writeheader()
        quality_writer.writeheader()

        for color_ns, color_msg, depth_ns, depth_msg, info_msg in pairs:
            ts = ns_to_sec(color_ns)
            if ts - last_kept_ts < min_dt:
                continue
            color = image_to_array(color_msg, is_depth=False)
            depth = normalize_depth(image_to_array(depth_msg, is_depth=True), depth_factor)
            q_blur = blur_score(color)
            q_clip = exposure_clip_ratio(color)
            q_depth = depth_valid_ratio(depth)
            accepted = True
            reason = "ok"
            if q_depth < 0.05:
                accepted = False
                reason = "depth_valid_ratio_low"

            index = kept
            rgb_rel = f"images/frame_{index:06d}.png"
            depth_rel = f"depth/frame_{index:06d}.png"
            if accepted:
                cv2.imwrite(str(dataset / rgb_rel), color)
                cv2.imwrite(str(dataset / depth_rel), depth)
                f_rgb.write(f"{ts:.9f} {rgb_rel}\n")
                f_depth.write(f"{ts:.9f} {depth_rel}\n")
                f_assoc.write(f"{ts:.9f} {rgb_rel} {ts:.9f} {depth_rel}\n")
                frames_writer.writerow(
                    {
                        "index": index,
                        "timestamp": f"{ts:.9f}",
                        "rgb_file": rgb_rel,
                        "depth_file": depth_rel,
                        "color_stamp": color_ns,
                        "depth_stamp": depth_ns,
                        "stamp_delta_sec": f"{abs(color_ns - depth_ns) / 1e9:.9f}",
                    }
                )
                kept += 1
                last_kept_ts = ts
                if camera_info is None:
                    camera_info = write_camera_info_json(info_msg, camera_info_json)
                    camera_info["depth_factor"] = depth_factor
                    write_camera_info_dict(camera_info, camera_info_json)

            quality_writer.writerow(
                {
                    "index": index,
                    "timestamp": f"{ts:.9f}",
                    "blur_score": f"{q_blur:.6f}",
                    "exposure_clip_ratio": f"{q_clip:.6f}",
                    "depth_valid_ratio": f"{q_depth:.6f}",
                    "accepted": str(accepted).lower(),
                    "reason": reason,
                }
            )

            if max_frames > 0 and kept >= max_frames:
                break

    if kept == 0:
        raise RuntimeError("No synchronized RGB-D frames were extracted.")
    return {"frame_count": kept, "camera_info": camera_info}


def extract_ros2_bag(
    bag: Path,
    dataset: Path,
    profile: dict[str, Any],
    target_fps: float,
    max_frames: int,
) -> dict[str, Any]:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import rosbag2_py

    print(f"[pseudo-gt] Extracting ROS2 bag frames from {bag}")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=read_rosbag_storage_id(bag)),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topics = {meta.name: meta.type for meta in reader.get_all_topics_and_types()}
    available = set(topics)
    rgb_topic = choose_topic(available, profile["rgb_topics"])
    depth_topic = choose_topic(available, profile["depth_topics"])
    info_topic = choose_topic(available, profile["camera_info_topics"])

    colors: list[tuple[int, Any]] = []
    depths: list[tuple[int, Any]] = []
    infos: list[tuple[int, Any]] = []
    selected = {rgb_topic, depth_topic, info_topic}
    type_cache = {topic: get_message(topics[topic]) for topic in selected}

    while reader.has_next():
        topic, data, bag_ns = reader.read_next()
        if topic not in selected:
            continue
        msg = deserialize_message(data, type_cache[topic])
        stamp_ns_value = msg_stamp_ns(msg, bag_ns)
        if topic == rgb_topic:
            colors.append((stamp_ns_value, msg))
        elif topic == depth_topic:
            depths.append((stamp_ns_value, msg))
        elif topic == info_topic:
            infos.append((stamp_ns_value, msg))

    if not colors or not depths or not infos:
        raise RuntimeError(
            f"Missing RGB-D data: colors={len(colors)} depths={len(depths)} camera_info={len(infos)}"
        )

    colors.sort(key=lambda item: item[0])
    depths.sort(key=lambda item: item[0])
    infos.sort(key=lambda item: item[0])
    depth_times = [item[0] for item in depths]
    info_msg = infos[0][1]
    max_delta_ns = int(float(profile.get("association_max_dt", 0.05)) * 1_000_000_000)
    pairs: list[tuple[int, Any, int, Any, Any]] = []

    for color_ns, color_msg in colors:
        idx = bisect.bisect_left(depth_times, color_ns)
        choices = []
        if idx < len(depths):
            choices.append(depths[idx])
        if idx > 0:
            choices.append(depths[idx - 1])
        if not choices:
            continue
        depth_ns, depth_msg = min(choices, key=lambda item: abs(item[0] - color_ns))
        if abs(depth_ns - color_ns) <= max_delta_ns:
            pairs.append((color_ns, color_msg, depth_ns, depth_msg, info_msg))

    result = write_frames(
        pairs,
        dataset,
        target_fps=target_fps,
        max_frames=max_frames,
        depth_factor=float(profile.get("depth_factor", 1000.0)),
    )
    result.update(
        {
            "rgb_topic": rgb_topic,
            "depth_topic": depth_topic,
            "camera_info_topic": info_topic,
            "imu_topic": choose_topic(available, profile.get("imu_topics", []), required=False),
            "raw_color_count": len(colors),
            "raw_depth_count": len(depths),
            "associated_count": len(pairs),
        }
    )
    return result


def wait_for_topics(topics: list[str], timeout: float, log: Path | None = None) -> None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        proc = subprocess.run(["ros2", "topic", "list"], text=True, capture_output=True, check=False)
        available = set(proc.stdout.splitlines())
        if all(topic in available for topic in topics):
            return
        time.sleep(1)
    if log is not None:
        with log.open("a", encoding="utf-8") as fh:
            fh.write("[topics]\n")
            fh.write(subprocess.run(["ros2", "topic", "list"], text=True, capture_output=True).stdout)
    raise RuntimeError(f"Timed out waiting for topics: {', '.join(topics)}")


def launch_realsense_ros1_bag(bag: Path, profile: dict[str, Any], log: Path) -> subprocess.Popen:
    cmd = [
        "ros2",
        "launch",
        "realsense2_camera",
        "rs_launch.py",
        f"rosbag_filename:={bag}",
        f"camera_name:={profile.get('camera_name', 'camera')}",
        "enable_color:=true",
        "enable_depth:=true",
        "enable_sync:=false",
        "align_depth.enable:=true",
        "initial_reset:=false",
        "enable_accel:=true",
        "enable_gyro:=true",
        "unite_imu_method:=2",
        "pointcloud.enable:=false",
    ]
    log.parent.mkdir(parents=True, exist_ok=True)
    fh = log.open("a", encoding="utf-8")
    fh.write("+ " + " ".join(cmd) + "\n")
    fh.flush()
    return subprocess.Popen(cmd, stdout=fh, stderr=subprocess.STDOUT, text=True)


def terminate_processes(processes: list[subprocess.Popen]) -> None:
    for proc in processes:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
    time.sleep(2)
    for proc in processes:
        if proc.poll() is None:
            proc.terminate()
    for proc in processes:
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()


def extract_live_topics_from_realsense_bag(
    bag: Path,
    dataset: Path,
    profile: dict[str, Any],
    target_fps: float,
    max_frames: int,
    log: Path,
) -> dict[str, Any]:
    print(f"[pseudo-gt] Extracting RealSense ROS1 SDK bag through realsense2_camera: {bag}")
    proc = launch_realsense_ros1_bag(bag, profile, log)
    try:
        topics = [profile["rgb_topics"][0], profile["depth_topics"][0], profile["camera_info_topics"][0]]
        wait_for_topics(topics, timeout=60, log=log)
        result = run_live_extractor(dataset, profile, target_fps, max_frames, proc)
        result.update(
            {
                "rgb_topic": profile["rgb_topics"][0],
                "depth_topic": profile["depth_topics"][0],
                "camera_info_topic": profile["camera_info_topics"][0],
                "imu_topic": profile.get("imu_topics", [None])[0],
            }
        )
    finally:
        terminate_processes([proc])
    return result


def run_live_extractor(
    dataset: Path,
    profile: dict[str, Any],
    target_fps: float,
    max_frames: int,
    playback_proc: subprocess.Popen,
) -> dict[str, Any]:
    import rclpy
    from sensor_msgs.msg import CameraInfo, Image

    class LiveExtractor:
        def __init__(self) -> None:
            self.node = rclpy.create_node("pseudo_gt_live_rgbd_extractor")
            self.rgb_topic = profile["rgb_topics"][0]
            self.depth_topic = profile["depth_topics"][0]
            self.info_topic = profile["camera_info_topics"][0]
            self.max_delta_ns = int(float(profile.get("association_max_dt", 0.05)) * 1_000_000_000)
            self.depth_queue: list[tuple[int, Any]] = []
            self.color_queue: list[tuple[int, Any]] = []
            self.info_msg = None
            self.pairs: list[tuple[int, Any, int, Any, Any]] = []
            self.last_msg_time = time.time()
            self.sub_rgb = self.node.create_subscription(Image, self.rgb_topic, self.on_rgb, 50)
            self.sub_depth = self.node.create_subscription(Image, self.depth_topic, self.on_depth, 50)
            self.sub_info = self.node.create_subscription(CameraInfo, self.info_topic, self.on_info, 10)
            self.timer = self.node.create_timer(0.5, self.on_timer)

        def on_rgb(self, msg: Any) -> None:
            self.last_msg_time = time.time()
            self.color_queue.append((stamp_to_ns(msg.header.stamp), msg))
            self.try_pair()

        def on_depth(self, msg: Any) -> None:
            self.last_msg_time = time.time()
            self.depth_queue.append((stamp_to_ns(msg.header.stamp), msg))
            self.depth_queue = self.depth_queue[-200:]
            self.try_pair()

        def on_info(self, msg: Any) -> None:
            self.last_msg_time = time.time()
            if self.info_msg is None:
                self.info_msg = msg

        def try_pair(self) -> None:
            if self.info_msg is None or not self.depth_queue:
                return
            remaining = []
            depth_times = [item[0] for item in self.depth_queue]
            for color_ns, color_msg in self.color_queue:
                idx = bisect.bisect_left(depth_times, color_ns)
                choices = []
                if idx < len(self.depth_queue):
                    choices.append(self.depth_queue[idx])
                if idx > 0:
                    choices.append(self.depth_queue[idx - 1])
                if not choices:
                    remaining.append((color_ns, color_msg))
                    continue
                depth_ns, depth_msg = min(choices, key=lambda item: abs(item[0] - color_ns))
                if abs(depth_ns - color_ns) <= self.max_delta_ns:
                    self.pairs.append((color_ns, color_msg, depth_ns, depth_msg, self.info_msg))
                    if max_frames > 0 and len(self.pairs) >= max_frames:
                        rclpy.shutdown()
                        return
                else:
                    remaining.append((color_ns, color_msg))
            self.color_queue = remaining[-200:]

        def on_timer(self) -> None:
            if playback_proc.poll() is not None and time.time() - self.last_msg_time > 3:
                rclpy.shutdown()
            if time.time() - self.last_msg_time > 20 and self.pairs:
                rclpy.shutdown()

    rclpy.init()
    extractor = LiveExtractor()
    try:
        rclpy.spin(extractor.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        pairs = list(extractor.pairs)
        extractor.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return write_frames(
        pairs,
        dataset,
        target_fps=target_fps,
        max_frames=max_frames,
        depth_factor=float(profile.get("depth_factor", 1000.0)),
    )


def parse_tum_list(path: Path) -> list[tuple[float, str]]:
    rows = []
    with path.open("r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 2:
                continue
            try:
                rows.append((float(parts[0]), parts[1]))
            except ValueError:
                continue
    rows.sort(key=lambda item: item[0])
    return rows


def parse_tum_associations(path: Path) -> list[tuple[float, str, float, str]]:
    rows = []
    with path.open("r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 4:
                continue
            try:
                rows.append((float(parts[0]), parts[1], float(parts[2]), parts[3]))
            except ValueError:
                continue
    rows.sort(key=lambda item: item[0])
    return rows


def resolve_dataset_file(root: Path, value: str) -> Path:
    path = Path(value)
    return path if path.is_absolute() else root / path


def associate_tum_rgbd(root: Path, max_dt: float) -> list[tuple[float, str, float, str]]:
    assoc = root / "associations.txt"
    if assoc.exists():
        return parse_tum_associations(assoc)

    rgbs = parse_tum_list(root / "rgb.txt")
    depths = parse_tum_list(root / "depth.txt")
    depth_times = [item[0] for item in depths]
    pairs = []
    for rgb_ts, rgb_file in rgbs:
        idx = bisect.bisect_left(depth_times, rgb_ts)
        choices = []
        if idx < len(depths):
            choices.append(depths[idx])
        if idx > 0:
            choices.append(depths[idx - 1])
        if not choices:
            continue
        depth_ts, depth_file = min(choices, key=lambda item: abs(item[0] - rgb_ts))
        if abs(depth_ts - rgb_ts) <= max_dt:
            pairs.append((rgb_ts, rgb_file, depth_ts, depth_file))
    return pairs


def infer_tum_camera_info(root: Path, profile: dict[str, Any]) -> dict[str, Any]:
    info = dict(profile.get("intrinsics", {}))
    if not info:
        keyspace = " ".join(part.lower() for part in root.parts)
        for key, values in TUM_FREIBURG_INTRINSICS.items():
            if key in keyspace:
                info = dict(values)
                break
    if not info:
        raise RuntimeError(
            "Could not infer TUM RGB-D intrinsics from path. "
            "Use a profile with intrinsics.fx/fy/cx/cy."
        )
    info.setdefault("width", 640)
    info.setdefault("height", 480)
    info.setdefault("distortion_model", "none")
    info.setdefault("d", [])
    info.setdefault("depth_factor", float(profile.get("depth_factor", 5000.0)))
    return info


def normalize_tum_rgbd(
    root: Path,
    dataset: Path,
    profile: dict[str, Any],
    target_fps: float,
    max_frames: int,
) -> dict[str, Any]:
    import cv2

    if not (root / "rgb.txt").exists() or not (root / "depth.txt").exists():
        raise RuntimeError(f"TUM RGB-D input requires rgb.txt and depth.txt: {root}")

    print(f"[pseudo-gt] Normalizing TUM RGB-D sequence from {root}")
    images_dir = dataset / "images"
    depth_dir = dataset / "depth"
    images_dir.mkdir(parents=True, exist_ok=True)
    depth_dir.mkdir(parents=True, exist_ok=True)

    frames_csv = dataset / "frames.csv"
    quality_csv = dataset / "quality.csv"
    rgb_txt = dataset / "rgb.txt"
    depth_txt = dataset / "depth.txt"
    assoc_txt = dataset / "associations.txt"
    camera_info_json = dataset / "camera_info.json"
    camera_info = infer_tum_camera_info(root, profile)
    depth_factor = float(camera_info.get("depth_factor", profile.get("depth_factor", 5000.0)))
    pairs = associate_tum_rgbd(root, max_dt=float(profile.get("association_max_dt", 0.05)))

    kept = 0
    last_kept_ts = -math.inf
    min_dt = 1.0 / target_fps if target_fps > 0 else 0.0
    skipped_missing = 0
    skipped_unreadable = 0

    with frames_csv.open("w", newline="", encoding="utf-8") as f_frames, \
        quality_csv.open("w", newline="", encoding="utf-8") as f_quality, \
        rgb_txt.open("w", encoding="utf-8") as f_rgb, \
        depth_txt.open("w", encoding="utf-8") as f_depth, \
        assoc_txt.open("w", encoding="utf-8") as f_assoc:
        frames_writer = csv.DictWriter(
            f_frames,
            fieldnames=[
                "index",
                "timestamp",
                "rgb_file",
                "depth_file",
                "color_stamp",
                "depth_stamp",
                "stamp_delta_sec",
            ],
        )
        quality_writer = csv.DictWriter(
            f_quality,
            fieldnames=[
                "index",
                "timestamp",
                "blur_score",
                "exposure_clip_ratio",
                "depth_valid_ratio",
                "accepted",
                "reason",
            ],
        )
        frames_writer.writeheader()
        quality_writer.writeheader()

        for rgb_ts, rgb_rel_src, depth_ts, depth_rel_src in pairs:
            if rgb_ts - last_kept_ts < min_dt:
                continue
            rgb_src = resolve_dataset_file(root, rgb_rel_src)
            depth_src = resolve_dataset_file(root, depth_rel_src)
            if not rgb_src.exists() or not depth_src.exists():
                skipped_missing += 1
                continue

            color = cv2.imread(str(rgb_src), cv2.IMREAD_COLOR)
            depth = cv2.imread(str(depth_src), cv2.IMREAD_UNCHANGED)
            if color is None or depth is None:
                skipped_unreadable += 1
                continue
            if depth.ndim == 3:
                depth = depth[:, :, 0]
            depth = normalize_depth(depth, depth_factor=1.0)
            q_blur = blur_score(color)
            q_clip = exposure_clip_ratio(color)
            q_depth = depth_valid_ratio(depth)
            accepted = q_depth >= 0.05
            reason = "ok" if accepted else "depth_valid_ratio_low"
            index = kept

            if accepted:
                if kept == 0:
                    camera_info["height"], camera_info["width"] = int(color.shape[0]), int(color.shape[1])
                    write_camera_info_dict(camera_info, camera_info_json)
                rgb_rel = f"images/frame_{index:06d}.png"
                depth_rel = f"depth/frame_{index:06d}.png"
                cv2.imwrite(str(dataset / rgb_rel), color)
                cv2.imwrite(str(dataset / depth_rel), depth)
                f_rgb.write(f"{rgb_ts:.9f} {rgb_rel}\n")
                f_depth.write(f"{depth_ts:.9f} {depth_rel}\n")
                f_assoc.write(f"{rgb_ts:.9f} {rgb_rel} {depth_ts:.9f} {depth_rel}\n")
                frames_writer.writerow(
                    {
                        "index": index,
                        "timestamp": f"{rgb_ts:.9f}",
                        "rgb_file": rgb_rel,
                        "depth_file": depth_rel,
                        "color_stamp": f"{rgb_ts:.9f}",
                        "depth_stamp": f"{depth_ts:.9f}",
                        "stamp_delta_sec": f"{abs(rgb_ts - depth_ts):.9f}",
                    }
                )
                kept += 1
                last_kept_ts = rgb_ts

            quality_writer.writerow(
                {
                    "index": index,
                    "timestamp": f"{rgb_ts:.9f}",
                    "blur_score": f"{q_blur:.6f}",
                    "exposure_clip_ratio": f"{q_clip:.6f}",
                    "depth_valid_ratio": f"{q_depth:.6f}",
                    "accepted": str(accepted).lower(),
                    "reason": reason,
                }
            )
            if max_frames > 0 and kept >= max_frames:
                break

    if kept == 0:
        raise RuntimeError("No synchronized TUM RGB-D frames were normalized.")
    return {
        "input_format": "tum_rgbd",
        "frame_count": kept,
        "associated_count": len(pairs),
        "skipped_missing": skipped_missing,
        "skipped_unreadable": skipped_unreadable,
        "camera_info": camera_info,
        "depth_factor": depth_factor,
    }


def normalize_bag(
    bag: Path,
    dataset: Path,
    profile: dict[str, Any],
    target_fps: float,
    max_frames: int,
    log_dir: Path,
) -> dict[str, Any]:
    dataset.mkdir(parents=True, exist_ok=True)
    if profile.get("storage") == "realsense_ros1_bag":
        return extract_live_topics_from_realsense_bag(
            bag,
            dataset,
            profile,
            target_fps=target_fps,
            max_frames=max_frames,
            log=log_dir / "extract_realsense_ros1.log",
        )
    return extract_ros2_bag(
        bag,
        dataset,
        profile,
        target_fps=target_fps,
        max_frames=max_frames,
    )


def detect_input_format(path: Path, requested: str, profile: dict[str, Any]) -> str:
    if requested != "auto":
        return requested
    if profile.get("storage") == "tum_rgbd":
        return "tum_rgbd"
    if path.is_dir() and (path / "rgb.txt").exists() and (path / "depth.txt").exists():
        return "tum_rgbd"
    return "bag"


def normalize_input(
    source: Path,
    dataset: Path,
    profile: dict[str, Any],
    input_format: str,
    target_fps: float,
    max_frames: int,
    log_dir: Path,
) -> dict[str, Any]:
    dataset.mkdir(parents=True, exist_ok=True)
    if input_format == "tum_rgbd":
        return normalize_tum_rgbd(
            source,
            dataset,
            profile,
            target_fps=target_fps,
            max_frames=max_frames,
        )
    return normalize_bag(
        source,
        dataset,
        profile,
        target_fps=target_fps,
        max_frames=max_frames,
        log_dir=log_dir,
    )


def start_bag_playback(bag: Path, profile: dict[str, Any], log: Path) -> subprocess.Popen:
    if profile.get("storage") == "realsense_ros1_bag":
        return launch_realsense_ros1_bag(bag, profile, log)
    cmd = ["ros2", "bag", "play", str(bag), "--clock"]
    log.parent.mkdir(parents=True, exist_ok=True)
    fh = log.open("a", encoding="utf-8")
    fh.write("+ " + " ".join(cmd) + "\n")
    fh.flush()
    return subprocess.Popen(cmd, stdout=fh, stderr=subprocess.STDOUT, text=True)


def run_rtabmap_candidate(
    method: str,
    bag: Path,
    out_dir: Path,
    profile: dict[str, Any],
) -> CandidateResult:
    subscribe_imu = method == "rtabmap_rgbd_imu"
    log = out_dir / "run.log"
    out_dir.mkdir(parents=True, exist_ok=True)
    tum = out_dir / "trajectory_tum.csv"
    full = out_dir / "trajectory_full.csv"
    if subscribe_imu and not profile.get("imu_topics"):
        return CandidateResult(method, "failed", None, log, {}, "profile has no IMU topic")
    processes: list[subprocess.Popen] = []
    try:
        playback = start_bag_playback(bag, profile, log)
        processes.append(playback)
        rgb_topic = profile["rgb_topics"][0]
        depth_topic = profile["depth_topics"][0]
        info_topic = profile["camera_info_topics"][0]
        required_topics = [rgb_topic, depth_topic, info_topic]
        if subscribe_imu and profile.get("imu_topics"):
            required_topics.append(profile["imu_topics"][0])
        wait_for_topics(required_topics, timeout=60, log=log)

        env = os.environ.copy()
        env.update(
            {
                "PSEUDO_GT_RGB_TOPIC": rgb_topic,
                "PSEUDO_GT_DEPTH_TOPIC": depth_topic,
                "PSEUDO_GT_CAMERA_INFO_TOPIC": info_topic,
                "PSEUDO_GT_IMU_TOPIC": profile.get("imu_topics") and profile["imu_topics"][0] or "/camera/imu",
                "RTABMAP_SUBSCRIBE_IMU": "true" if subscribe_imu else "false",
                "RTABMAP_FRAME_ID": str(profile.get("frame_id", "camera_link")),
            }
        )
        with log.open("a", encoding="utf-8") as fh:
            fh.write("[rtabmap] starting odometry\n")
            fh.flush()
            rtab_proc = subprocess.Popen(
                ["/work/scripts/_inside_rtabmap_rgbd_odom.sh"],
                stdout=fh,
                stderr=subprocess.STDOUT,
                text=True,
                env=env,
            )
        processes.append(rtab_proc)

        record_cmd = [
            "python3",
            "/work/scripts/record_odom_tum.py",
            "--topic",
            "/rtabmap/odom",
            "--tum",
            str(tum),
            "--full-csv",
            str(full),
            "--startup-timeout",
            "90",
            "--idle-timeout",
            "15",
        ]
        rc = run(record_cmd, log=log)
        if rc != 0 or not tum.exists() or tum.stat().st_size == 0:
            return CandidateResult(method, "failed", None, log, {}, "rtabmap did not produce odometry")
        return CandidateResult(method, "ok", tum, log, {"subscribe_imu": subscribe_imu})
    except Exception as exc:
        return CandidateResult(method, "failed", None, log, {}, str(exc))
    finally:
        terminate_processes(processes)


def camera_params_from_info(camera_info: dict[str, Any]) -> tuple[float, float, float, float]:
    try:
        return (
            float(camera_info["fx"]),
            float(camera_info["fy"]),
            float(camera_info["cx"]),
            float(camera_info["cy"]),
        )
    except KeyError as exc:
        raise RuntimeError(f"Missing camera intrinsic field in camera_info.json: {exc}") from exc


def colmap_preset_args(preset: str) -> tuple[list[str], list[str]]:
    if preset == "fast":
        return (
            [
                "--FeatureExtraction.max_image_size",
                "960",
                "--SiftExtraction.max_num_features",
                "4096",
            ],
            ["--SequentialMatching.overlap", "10"],
        )
    if preset == "robust":
        return (
            [
                "--FeatureExtraction.max_image_size",
                "1600",
                "--SiftExtraction.max_num_features",
                "12000",
                "--SiftExtraction.peak_threshold",
                "0.003",
                "--SiftExtraction.estimate_affine_shape",
                "1",
                "--SiftExtraction.domain_size_pooling",
                "1",
            ],
            [
                "--SequentialMatching.overlap",
                "30",
                "--SequentialMatching.loop_detection",
                "1",
                "--SequentialMatching.loop_detection_num_images",
                "100",
            ],
        )
    return (
        [
            "--FeatureExtraction.max_image_size",
            "1280",
            "--SiftExtraction.max_num_features",
            "8192",
            "--SiftExtraction.estimate_affine_shape",
            "1",
            "--SiftExtraction.domain_size_pooling",
            "1",
        ],
        [
            "--SequentialMatching.overlap",
            "20",
            "--SequentialMatching.quadratic_overlap",
            "1",
            "--SequentialMatching.loop_detection",
            "1",
            "--SequentialMatching.loop_detection_period",
            "10",
            "--SequentialMatching.loop_detection_num_images",
            "50",
        ],
    )


def parse_colmap_images(images_txt: Path) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    from scipy.spatial.transform import Rotation

    poses: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    lines = images_txt.read_text(encoding="utf-8", errors="replace").splitlines()
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        i += 1
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) < 10:
            continue
        qw, qx, qy, qz = map(float, parts[1:5])
        t = np.array(list(map(float, parts[5:8])), dtype=float)
        name = parts[9]
        rot_wc = Rotation.from_quat([qx, qy, qz, qw])
        rot_cw = rot_wc.inv()
        center = -(rot_wc.as_matrix().T @ t)
        poses[name] = (center, rot_cw.as_quat())
        i += 1
    return poses


def frames_timestamp_map(frames_csv: Path) -> dict[str, float]:
    mapping: dict[str, float] = {}
    with frames_csv.open("r", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            mapping[Path(row["rgb_file"]).name] = float(row["timestamp"])
    return mapping


def export_colmap_tum(dataset: Path, sparse_root: Path, tum: Path) -> dict[str, Any]:
    submodels = sorted([p for p in sparse_root.iterdir() if (p / "images.txt").exists()])
    if not submodels:
        raise RuntimeError("COLMAP did not create a sparse model with images.txt")
    models = [(p, len(parse_colmap_images(p / "images.txt"))) for p in submodels]
    best_model = max(models, key=lambda item: item[1])[0]
    poses = parse_colmap_images(best_model / "images.txt")
    ts_map = frames_timestamp_map(dataset / "frames.csv")
    rows = []
    for name, (t, q_xyzw) in poses.items():
        ts = ts_map.get(Path(name).name)
        if ts is None:
            continue
        rows.append((ts, t, q_xyzw))
    rows.sort(key=lambda item: item[0])
    with tum.open("w", encoding="utf-8") as fh:
        for ts, t, q in rows:
            fh.write(
                f"{ts:.9f} {t[0]:.9f} {t[1]:.9f} {t[2]:.9f} "
                f"{q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n"
            )
    return {"registered_frames": len(rows), "selected_model": str(best_model.name)}


def convert_colmap_models_to_text(sparse_root: Path, text_root: Path, log: Path) -> Path:
    text_root.mkdir(parents=True, exist_ok=True)
    exported = 0
    for model in sorted([p for p in sparse_root.iterdir() if p.is_dir()]):
        if (model / "images.txt").exists():
            dest = text_root / model.name
            if dest.exists():
                shutil.rmtree(dest)
            shutil.copytree(model, dest)
            exported += 1
            continue
        if not (model / "images.bin").exists():
            continue
        dest = text_root / model.name
        dest.mkdir(parents=True, exist_ok=True)
        rc = run(
            [
                "colmap",
                "model_converter",
                "--input_path",
                str(model),
                "--output_path",
                str(dest),
                "--output_type",
                "TXT",
            ],
            log=log,
        )
        if rc == 0 and (dest / "images.txt").exists():
            exported += 1
    if exported == 0:
        raise RuntimeError("COLMAP did not produce a convertible sparse model")
    return text_root


def run_colmap_candidate(
    dataset: Path,
    out_dir: Path,
    preset: str,
    use_gpu: str,
) -> CandidateResult:
    method = "colmap_sfm"
    log = out_dir / "run.log"
    out_dir.mkdir(parents=True, exist_ok=True)
    tum = out_dir / "trajectory_tum.csv"
    if shutil.which("colmap") is None:
        return CandidateResult(method, "failed", None, log, {}, "colmap binary not found")
    try:
        camera_info = json.loads((dataset / "camera_info.json").read_text(encoding="utf-8"))
        fx, fy, cx, cy = camera_params_from_info(camera_info)
        feature_args, matching_args = colmap_preset_args(preset)
        database = out_dir / "database.db"
        sparse = out_dir / "sparse"
        sparse.mkdir(exist_ok=True)
        gpu_flag = "1" if use_gpu == "1" else "0"
        feature_cmd = [
            "colmap",
            "feature_extractor",
            "--database_path",
            str(database),
            "--image_path",
            str(dataset / "images"),
            "--ImageReader.camera_model",
            "PINHOLE",
            "--ImageReader.single_camera",
            "1",
            "--ImageReader.camera_params",
            f"{fx},{fy},{cx},{cy}",
            "--FeatureExtraction.use_gpu",
            gpu_flag,
            *feature_args,
        ]
        match_cmd = [
            "colmap",
            "sequential_matcher",
            "--database_path",
            str(database),
            "--FeatureMatching.guided_matching",
            "1",
            "--FeatureMatching.use_gpu",
            gpu_flag,
            *matching_args,
        ]
        mapper_cmd = [
            "colmap",
            "mapper",
            "--database_path",
            str(database),
            "--image_path",
            str(dataset / "images"),
            "--output_path",
            str(sparse),
            "--Mapper.ba_refine_focal_length",
            "0",
            "--Mapper.ba_refine_principal_point",
            "0",
            "--Mapper.ba_refine_extra_params",
            "0",
        ]
        for cmd in (feature_cmd, match_cmd, mapper_cmd):
            rc = run(cmd, log=log)
            if rc != 0:
                return CandidateResult(method, "failed", None, log, {}, f"COLMAP command failed: {cmd[1]}")
        sparse_txt = convert_colmap_models_to_text(sparse, out_dir / "sparse_txt", log)
        metrics = export_colmap_tum(dataset, sparse_txt, tum)
        metrics["preset"] = preset
        metrics["use_gpu"] = gpu_flag
        if not tum.exists() or tum.stat().st_size == 0:
            return CandidateResult(method, "failed", None, log, metrics, "COLMAP registered no exportable frames")
        return CandidateResult(method, "ok", tum, log, metrics)
    except Exception as exc:
        return CandidateResult(method, "failed", None, log, {}, str(exc))


def write_orbslam3_settings(dataset: Path, out_dir: Path) -> Path:
    camera_info = json.loads((dataset / "camera_info.json").read_text(encoding="utf-8"))
    fx, fy, cx, cy = camera_params_from_info(camera_info)
    width = int(camera_info.get("width", 640))
    height = int(camera_info.get("height", 480))
    depth_factor = float(camera_info.get("depth_factor", 1000.0))
    settings = out_dir / "orbslam3_rgbd.yaml"
    settings.write_text(
        "\n".join(
            [
                "%YAML:1.0",
                f"Camera.fx: {fx}",
                f"Camera.fy: {fy}",
                f"Camera.cx: {cx}",
                f"Camera.cy: {cy}",
                "Camera.k1: 0.0",
                "Camera.k2: 0.0",
                "Camera.p1: 0.0",
                "Camera.p2: 0.0",
                "Camera.k3: 0.0",
                f"Camera.width: {width}",
                f"Camera.height: {height}",
                "Camera.fps: 30.0",
                "Camera.bf: 40.0",
                "Camera.RGB: 1",
                "ThDepth: 8.0",
                f"DepthMapFactor: {depth_factor}",
                "ORBextractor.nFeatures: 1000",
                "ORBextractor.scaleFactor: 1.2",
                "ORBextractor.nLevels: 8",
                "ORBextractor.iniThFAST: 20",
                "ORBextractor.minThFAST: 7",
                "",
            ]
        ),
        encoding="utf-8",
    )
    return settings


def run_orbslam3_candidate(dataset: Path, out_dir: Path) -> CandidateResult:
    method = "orbslam3_rgbd"
    log = out_dir / "run.log"
    out_dir.mkdir(parents=True, exist_ok=True)
    tum = out_dir / "trajectory_tum.csv"
    binary = Path(os.environ.get("ORB_SLAM3_RGBD_BIN", "/opt/ORB_SLAM3/Examples/RGB-D/rgbd_tum"))
    vocab = Path(os.environ.get("ORB_SLAM3_VOCAB", "/opt/ORB_SLAM3/Vocabulary/ORBvoc.txt"))
    if not binary.exists():
        return CandidateResult(method, "failed", None, log, {}, f"ORB-SLAM3 RGB-D binary not found: {binary}")
    if not vocab.exists():
        return CandidateResult(method, "failed", None, log, {}, f"ORB-SLAM3 vocabulary not found: {vocab}")
    settings = write_orbslam3_settings(dataset, out_dir)
    cmd = [str(binary), str(vocab), str(settings), str(dataset), str(dataset / "associations.txt")]
    rc = run(cmd, log=log, cwd=out_dir)
    if rc != 0:
        return CandidateResult(method, "failed", None, log, {}, "ORB-SLAM3 command failed")
    for candidate in (out_dir / "CameraTrajectory.txt", out_dir / "KeyFrameTrajectory.txt"):
        if candidate.exists() and candidate.stat().st_size > 0:
            shutil.copy2(candidate, tum)
            return CandidateResult(method, "ok", tum, log, {"source": candidate.name})
    return CandidateResult(method, "failed", None, log, {}, "ORB-SLAM3 did not write a trajectory")


def read_tum(path: Path) -> dict[str, np.ndarray]:
    rows = []
    with path.open("r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.replace(",", " ").split()
            if len(parts) < 8:
                continue
            try:
                rows.append([float(v) for v in parts[:8]])
            except ValueError:
                continue
    if not rows:
        return {"t": np.empty((0,)), "p": np.empty((0, 3)), "q": np.empty((0, 4))}
    arr = np.asarray(rows, dtype=float)
    finite = np.isfinite(arr).all(axis=1)
    arr = arr[finite]
    return {"t": arr[:, 0], "p": arr[:, 1:4], "q": arr[:, 4:8]}


def associate_trajectories(a: dict[str, np.ndarray], b: dict[str, np.ndarray], max_dt: float = 0.05) -> tuple[np.ndarray, np.ndarray]:
    idx_a = []
    idx_b = []
    bt = b["t"]
    for i, ts in enumerate(a["t"]):
        j = bisect.bisect_left(bt, ts)
        choices = []
        if j < len(bt):
            choices.append(j)
        if j > 0:
            choices.append(j - 1)
        if not choices:
            continue
        best = min(choices, key=lambda k: abs(bt[k] - ts))
        if abs(bt[best] - ts) <= max_dt:
            idx_a.append(i)
            idx_b.append(best)
    return np.asarray(idx_a, dtype=int), np.asarray(idx_b, dtype=int)


def umeyama_sim3(src: np.ndarray, dst: np.ndarray) -> tuple[float, np.ndarray, np.ndarray]:
    if src.shape != dst.shape or src.shape[0] < 3:
        raise ValueError("Need at least 3 paired points for Sim(3) alignment")
    mu_src = src.mean(axis=0)
    mu_dst = dst.mean(axis=0)
    src_c = src - mu_src
    dst_c = dst - mu_dst
    cov = (dst_c.T @ src_c) / src.shape[0]
    u, d, vt = np.linalg.svd(cov)
    s_mat = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        s_mat[-1, -1] = -1
    rot = u @ s_mat @ vt
    var_src = (src_c**2).sum() / src.shape[0]
    scale = float(np.trace(np.diag(d) @ s_mat) / var_src) if var_src > 0 else 1.0
    trans = mu_dst - scale * rot @ mu_src
    return scale, rot, trans


def apply_sim3(points: np.ndarray, scale: float, rot: np.ndarray, trans: np.ndarray) -> np.ndarray:
    return (scale * (rot @ points.T)).T + trans


def yaw_from_quat_xyzw(q: np.ndarray) -> float:
    from scipy.spatial.transform import Rotation

    return float(Rotation.from_quat(q).as_euler("zyx", degrees=True)[0])


def yaw_drift_deg(a: dict[str, np.ndarray], b: dict[str, np.ndarray], ia: np.ndarray, ib: np.ndarray) -> float | None:
    if len(ia) < 2:
        return None
    try:
        a0 = yaw_from_quat_xyzw(a["q"][ia[0]])
        a1 = yaw_from_quat_xyzw(a["q"][ia[-1]])
        b0 = yaw_from_quat_xyzw(b["q"][ib[0]])
        b1 = yaw_from_quat_xyzw(b["q"][ib[-1]])
        diff = ((a1 - a0) - (b1 - b0) + 180.0) % 360.0 - 180.0
        return abs(float(diff))
    except Exception:
        return None


def evaluate_pair(
    name_a: str,
    traj_a: dict[str, np.ndarray],
    name_b: str,
    traj_b: dict[str, np.ndarray],
    run_duration: float,
) -> dict[str, Any]:
    ia, ib = associate_trajectories(traj_a, traj_b)
    result: dict[str, Any] = {
        "method_a": name_a,
        "method_b": name_b,
        "pairs": int(len(ia)),
        "agree": False,
    }
    if len(ia) < 3:
        result["reason"] = "not_enough_pairs"
        return result
    src = traj_a["p"][ia]
    dst = traj_b["p"][ib]
    scale, rot, trans = umeyama_sim3(src, dst)
    aligned = apply_sim3(src, scale, rot, trans)
    errors = np.linalg.norm(aligned - dst, axis=1)
    overlap = float(min(traj_a["t"][ia[-1]], traj_b["t"][ib[-1]]) - max(traj_a["t"][ia[0]], traj_b["t"][ib[0]]))
    gaps = np.diff(traj_a["t"][ia]) if len(ia) > 1 else np.array([0.0])
    yaw_drift = yaw_drift_deg(traj_a, traj_b, ia, ib)
    rmse = float(np.sqrt(np.mean(errors**2)))
    median = float(np.median(errors))
    min_overlap = min(10.0, max(0.0, run_duration * 0.2))
    max_gap = float(gaps.max()) if len(gaps) else 0.0
    agree = (
        len(ia) >= 30
        and overlap >= min_overlap
        and rmse <= 0.20
        and median <= 0.10
        and max_gap <= 5.0
        and (yaw_drift is None or yaw_drift <= 5.0)
    )
    result.update(
        {
            "rmse": rmse,
            "median": median,
            "overlap_sec": overlap,
            "min_overlap_sec": min_overlap,
            "max_gap_sec": max_gap,
            "yaw_drift_deg": yaw_drift,
            "scale_a_to_b": scale,
            "agree": agree,
            "reason": "ok" if agree else "gate_failed",
        }
    )
    return result


def trajectory_duration(traj: dict[str, np.ndarray]) -> float:
    if len(traj["t"]) < 2:
        return 0.0
    return float(traj["t"][-1] - traj["t"][0])


def evaluate_agreement(results: list[CandidateResult], diagnostics: Path, allow_unreliable: bool) -> dict[str, Any]:
    diagnostics.mkdir(parents=True, exist_ok=True)
    healthy: dict[str, dict[str, np.ndarray]] = {}
    health: dict[str, dict[str, Any]] = {}
    for result in results:
        if result.status != "ok" or result.trajectory is None:
            health[result.method] = {"status": result.status, "reason": result.reason, "poses": 0}
            continue
        traj = read_tum(result.trajectory)
        poses = int(len(traj["t"]))
        ok = poses >= 30
        health[result.method] = {
            "status": "ok" if ok else "unhealthy",
            "reason": "ok" if ok else "too_few_poses",
            "poses": poses,
            "duration_sec": trajectory_duration(traj),
        }
        if ok:
            healthy[result.method] = traj

    run_duration = max((trajectory_duration(t) for t in healthy.values()), default=0.0)
    pairwise = []
    names = sorted(healthy)
    for i, name_a in enumerate(names):
        for name_b in names[i + 1 :]:
            pairwise.append(evaluate_pair(name_a, healthy[name_a], name_b, healthy[name_b], run_duration))

    support = {name: 0 for name in names}
    errors = {name: [] for name in names}
    for pair in pairwise:
        if pair.get("agree"):
            support[pair["method_a"]] += 1
            support[pair["method_b"]] += 1
            errors[pair["method_a"]].append(pair.get("median", math.inf))
            errors[pair["method_b"]].append(pair.get("median", math.inf))

    supported = [name for name, count in support.items() if count > 0]
    winner = None
    confidence = "none"
    if supported:
        winner = sorted(
            supported,
            key=lambda name: (
                -support[name],
                float(np.median(errors[name])) if errors[name] else math.inf,
                -health[name].get("poses", 0),
                name,
            ),
        )[0]
        confidence = "high" if support[winner] >= 2 else "medium"
    elif allow_unreliable and names:
        winner = sorted(names, key=lambda name: (-health[name].get("poses", 0), name))[0]
        confidence = "low"

    agreement = {
        "status": "ok" if supported else "agreement_failed",
        "winner": winner,
        "confidence": confidence,
        "support": support,
        "health": health,
        "pairwise": pairwise,
        "policy": {
            "min_pairs": 30,
            "rmse_max_m": 0.20,
            "median_max_m": 0.10,
            "yaw_drift_max_deg": 5.0,
            "max_gap_sec": 5.0,
            "min_overlap": "min(10s, 20% of run duration)",
        },
    }
    (diagnostics / "agreement.json").write_text(json.dumps(agreement, indent=2, sort_keys=True), encoding="utf-8")

    with (diagnostics / "pairwise_agreement.csv").open("w", newline="", encoding="utf-8") as fh:
        fieldnames = [
            "method_a",
            "method_b",
            "pairs",
            "agree",
            "rmse",
            "median",
            "overlap_sec",
            "max_gap_sec",
            "yaw_drift_deg",
            "reason",
        ]
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        for pair in pairwise:
            writer.writerow({key: pair.get(key, "") for key in fieldnames})

    write_summary(diagnostics / "summary.md", agreement)
    write_plots(diagnostics, healthy)
    return agreement


def write_summary(path: Path, agreement: dict[str, Any]) -> None:
    lines = [
        "# Pseudo-GT Agreement Summary",
        "",
        f"Status: `{agreement['status']}`",
        f"Winner: `{agreement.get('winner')}`",
        f"Confidence: `{agreement.get('confidence')}`",
        "",
        "## Method Health",
        "",
    ]
    for method, health in sorted(agreement["health"].items()):
        lines.append(f"- `{method}`: {health.get('status')} ({health.get('poses', 0)} poses, {health.get('reason')})")
    lines.extend(["", "## Pairwise Agreement", ""])
    for pair in agreement["pairwise"]:
        lines.append(
            f"- `{pair['method_a']}` vs `{pair['method_b']}`: agree={pair.get('agree')} "
            f"pairs={pair.get('pairs')} rmse={pair.get('rmse', '')} median={pair.get('median', '')}"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_plots(diagnostics: Path, trajectories: dict[str, dict[str, np.ndarray]]) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return
    if not trajectories:
        return

    plt.figure(figsize=(8, 6))
    for name, traj in sorted(trajectories.items()):
        if len(traj["p"]):
            plt.plot(traj["p"][:, 0], traj["p"][:, 1], label=name)
    plt.axis("equal")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.legend()
    plt.tight_layout()
    plt.savefig(diagnostics / "trajectory_xy.png")
    plt.close()

    plt.figure(figsize=(8, 3))
    for name, traj in sorted(trajectories.items()):
        if len(traj["t"]):
            rel = traj["t"] - traj["t"][0]
            plt.plot(rel, np.full_like(rel, len(name)), ".", label=name)
    plt.yticks([])
    plt.xlabel("seconds from first pose")
    plt.legend()
    plt.tight_layout()
    plt.savefig(diagnostics / "coverage.png")
    plt.close()


def persist_outputs(
    workspace: Path,
    output: Path,
    results: list[CandidateResult],
    agreement: dict[str, Any],
    persist_intermediates: bool,
) -> None:
    (output / "candidates").mkdir(parents=True, exist_ok=True)
    (output / "diagnostics").mkdir(parents=True, exist_ok=True)
    manifest = {
        "workspace": str(workspace),
        "results": [
            {
                "method": r.method,
                "status": r.status,
                "reason": r.reason,
                "metrics": r.metrics,
            }
            for r in results
        ],
    }
    for result in results:
        dest_dir = output / "candidates" / result.method
        dest_dir.mkdir(parents=True, exist_ok=True)
        if result.trajectory is not None and result.trajectory.exists():
            shutil.copy2(result.trajectory, dest_dir / "trajectory_tum.csv")
        if result.log is not None and result.log.exists():
            shutil.copy2(result.log, dest_dir / "run.log")

    diag_src = workspace / "diagnostics"
    if diag_src.exists():
        for item in diag_src.iterdir():
            dest = output / "diagnostics" / item.name
            if item.is_file():
                shutil.copy2(item, dest)

    winner = agreement.get("winner")
    if agreement.get("status") == "ok" and winner:
        src = output / "candidates" / winner / "trajectory_tum.csv"
        if src.exists():
            shutil.copy2(src, output / "best_pseudo_gt_tum.csv")
    elif winner:
        src = output / "candidates" / winner / "trajectory_tum.csv"
        if src.exists():
            shutil.copy2(src, output / "candidate_best_unreliable_tum.csv")

    if persist_intermediates:
        for name in ("dataset",):
            src = workspace / name
            if src.exists():
                dest = output / name
                if dest.exists():
                    shutil.rmtree(dest)
                shutil.copytree(src, dest)
    (output / "run_manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8")


def run_candidates(
    methods: list[str],
    bag: Path,
    workspace: Path,
    dataset: Path,
    profile: dict[str, Any],
    colmap_preset: str,
    colmap_use_gpu: str,
) -> list[CandidateResult]:
    results = []
    for method in methods:
        out_dir = workspace / "candidates" / method
        print(f"[pseudo-gt] Running candidate: {method}")
        if method in {"rtabmap_rgbd", "rtabmap_rgbd_imu"}:
            if profile.get("input_format") == "tum_rgbd":
                log = out_dir / "run.log"
                out_dir.mkdir(parents=True, exist_ok=True)
                log.write_text("RTAB-Map candidates require ROS bag playback; skipped for TUM RGB-D input.\n", encoding="utf-8")
                result = CandidateResult(method, "failed", None, log, {}, "requires ROS bag playback")
            else:
                result = run_rtabmap_candidate(method, bag, out_dir, profile)
        elif method == "colmap_sfm":
            result = run_colmap_candidate(dataset, out_dir, colmap_preset, colmap_use_gpu)
        elif method == "orbslam3_rgbd":
            result = run_orbslam3_candidate(dataset, out_dir)
        else:
            result = CandidateResult(method, "failed", None, None, {}, "unknown method")
        print(f"[pseudo-gt] {method}: {result.status} {result.reason}")
        results.append(result)
    return results


def write_extraction_manifest(workspace: Path, extraction: dict[str, Any], profile: dict[str, Any], args: argparse.Namespace) -> None:
    manifest = {
        "profile": profile["name"],
        "input_format": profile.get("input_format", args.input_format),
        "workspace_mode": args.workspace_mode,
        "target_fps": args.target_fps,
        "max_frames": args.max_frames,
        "extraction": extraction,
    }
    (workspace / "extraction_manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Build a best pseudo-GT trajectory from RGB-D bags or TUM RGB-D sequences.")
    parser.add_argument("bag", type=Path)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--profile-config", default=Path("/work/config/pseudo_gt_profiles.yaml"), type=Path)
    parser.add_argument("--input-format", default="auto", choices=["auto", "bag", "tum_rgbd"])
    parser.add_argument("--methods", default=DEFAULT_METHODS)
    parser.add_argument("--colmap-preset", default="stable", choices=["fast", "stable", "robust"])
    parser.add_argument("--colmap-use-gpu", default=os.environ.get("PSEUDO_GT_COLMAP_USE_GPU", "0"), choices=["0", "1"])
    parser.add_argument("--workspace-mode", default="ram", choices=["ram", "disk"])
    parser.add_argument("--keep-workspace", action="store_true")
    parser.add_argument("--persist-intermediates", action="store_true")
    parser.add_argument("--allow-unreliable-best", action="store_true")
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--max-frames", default=0, type=int)
    parser.add_argument("--target-fps", default=3.0, type=float)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    bag = args.bag.resolve()
    if not bag.exists():
        raise SystemExit(f"Input bag does not exist: {bag}")
    output = args.output.resolve()
    methods = parse_methods(args.methods)
    profile = load_profile(args.profile_config, args.profile)
    input_format = detect_input_format(bag, args.input_format, profile)
    profile["input_format"] = input_format
    ensure_empty_output(output, args.force)
    workspace = make_workspace(output, args.workspace_mode, args.keep_workspace)
    print(f"[pseudo-gt] Workspace: {workspace}")
    print(f"[pseudo-gt] Output: {output}")
    print(f"[pseudo-gt] Input format: {input_format}")

    try:
        dataset = workspace / "dataset"
        extraction = normalize_input(
            bag,
            dataset,
            profile,
            input_format,
            target_fps=args.target_fps,
            max_frames=args.max_frames,
            log_dir=workspace / "logs",
        )
        if extraction.get("rgb_topic"):
            profile["rgb_topics"] = [extraction["rgb_topic"]]
        if extraction.get("depth_topic"):
            profile["depth_topics"] = [extraction["depth_topic"]]
        if extraction.get("camera_info_topic"):
            profile["camera_info_topics"] = [extraction["camera_info_topic"]]
        if "imu_topic" in extraction:
            profile["imu_topics"] = [extraction["imu_topic"]] if extraction["imu_topic"] else []
        write_extraction_manifest(workspace, extraction, profile, args)
        results = run_candidates(
            methods,
            bag,
            workspace,
            dataset,
            profile,
            colmap_preset=args.colmap_preset,
            colmap_use_gpu=args.colmap_use_gpu,
        )
        agreement = evaluate_agreement(results, workspace / "diagnostics", args.allow_unreliable_best)
        persist_outputs(workspace, output, results, agreement, args.persist_intermediates)
        if agreement.get("status") != "ok" and not args.allow_unreliable_best:
            print("[pseudo-gt] Agreement failed; no reliable best_pseudo_gt_tum.csv was written.", file=sys.stderr)
            return 3
        return 0
    finally:
        cleanup_workspace(workspace, args.keep_workspace)


if __name__ == "__main__":
    sys.exit(main())
