#!/usr/bin/env python3
from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import os
import random
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


DEFAULT_METHODS = "rtabmap_rgbd,colmap_sfm,orbslam3_rgbd,orbslam3_rgbd_imu"
VALID_METHODS = {"rtabmap_rgbd", "rtabmap_rgbd_imu", "colmap_sfm", "orbslam3_rgbd", "orbslam3_rgbd_imu"}
TUM_FREIBURG_INTRINSICS = {
    "freiburg1": {
        "fx": 517.306408,
        "fy": 516.469215,
        "cx": 318.643040,
        "cy": 255.313989,
        "d": [0.262383, -0.953104, -0.005358, 0.002628, 1.163314],
        "depth_factor": 5000.0,
        "stereo_b": 0.07732,
    },
    "fr1": {
        "fx": 517.306408,
        "fy": 516.469215,
        "cx": 318.643040,
        "cy": 255.313989,
        "d": [0.262383, -0.953104, -0.005358, 0.002628, 1.163314],
        "depth_factor": 5000.0,
        "stereo_b": 0.07732,
    },
    "freiburg2": {
        "fx": 520.908620,
        "fy": 521.007327,
        "cx": 325.141442,
        "cy": 249.701764,
        "d": [0.231222, -0.784899, -0.003257, -0.000105, 0.917205],
        "depth_factor": 5208.0,
        "stereo_b": 0.0767,
    },
    "fr2": {
        "fx": 520.908620,
        "fy": 521.007327,
        "cx": 325.141442,
        "cy": 249.701764,
        "d": [0.231222, -0.784899, -0.003257, -0.000105, 0.917205],
        "depth_factor": 5208.0,
        "stereo_b": 0.0767,
    },
    "freiburg3": {
        "fx": 535.4,
        "fy": 539.2,
        "cx": 320.1,
        "cy": 247.6,
        "d": [0.0, 0.0, 0.0, 0.0, 0.0],
        "depth_factor": 5000.0,
        "stereo_b": 0.0747,
    },
    "fr3": {
        "fx": 535.4,
        "fy": 539.2,
        "cx": 320.1,
        "cy": 247.6,
        "d": [0.0, 0.0, 0.0, 0.0, 0.0],
        "depth_factor": 5000.0,
        "stereo_b": 0.0747,
    },
}


@dataclass
class CandidateResult:
    method: str
    status: str
    trajectory: Path | None
    log: Path | None
    metrics: dict[str, Any]
    reason: str = ""


@dataclass
class ProgressStep:
    name: str
    weight: float
    expected_sec: float


def format_duration(seconds: float) -> str:
    seconds = max(0, int(round(seconds)))
    minutes, sec = divmod(seconds, 60)
    hours, minutes = divmod(minutes, 60)
    if hours:
        return f"{hours:d}h{minutes:02d}m{sec:02d}s"
    return f"{minutes:d}m{sec:02d}s"


class ProgressReporter:
    def __init__(self, steps: list[ProgressStep], interval_sec: float = 20.0) -> None:
        self.steps = {step.name: step for step in steps}
        self.total_weight = sum(step.weight for step in steps) or 1.0
        self.total_expected = sum(step.expected_sec for step in steps)
        self.completed_weight = 0.0
        self.completed_steps: set[str] = set()
        self.active_name: str | None = None
        self.active_started = time.monotonic()
        self.started = time.monotonic()
        self.last_emit = 0.0
        self.interval_sec = interval_sec

    def start(self, name: str) -> None:
        self.active_name = name
        self.active_started = time.monotonic()
        self.emit("start", name, force=True)

    def done(self, name: str) -> None:
        if name not in self.completed_steps:
            self.completed_steps.add(name)
            self.completed_weight += self.steps.get(name, ProgressStep(name, 0.0, 0.0)).weight
        if self.active_name == name:
            self.active_name = None
        self.emit("done", name, force=True)

    def fail(self, name: str) -> None:
        self.emit("failed", name, force=True)

    def pulse(self, label: str | None = None) -> None:
        self.emit("running", label or self.active_name or "pipeline", force=False)

    def fraction(self) -> float:
        weight = self.completed_weight
        if self.active_name and self.active_name not in self.completed_steps:
            step = self.steps.get(self.active_name)
            if step:
                elapsed_active = time.monotonic() - self.active_started
                active_fraction = min(0.90, elapsed_active / max(step.expected_sec, 1.0))
                weight += step.weight * active_fraction
        return min(0.999, max(0.001, weight / self.total_weight))

    def eta_sec(self, fraction: float) -> float:
        elapsed = time.monotonic() - self.started
        if fraction > 0.01:
            return elapsed * (1.0 - fraction) / fraction
        return max(0.0, self.total_expected - elapsed)

    def emit(self, status: str, label: str, force: bool) -> None:
        now = time.monotonic()
        if not force and now - self.last_emit < self.interval_sec:
            return
        self.last_emit = now
        fraction = self.fraction()
        elapsed = now - self.started
        eta = self.eta_sec(fraction)
        print(
            f"[progress] {fraction * 100:5.1f}% stage={label} status={status} "
            f"elapsed={format_duration(elapsed)} eta={format_duration(eta)}",
            flush=True,
        )


def make_progress_reporter(methods: list[str]) -> ProgressReporter:
    candidate_weight = 0.60 / max(1, len(methods))
    candidate_expected = {
        "rtabmap_rgbd": 180.0,
        "rtabmap_rgbd_imu": 180.0,
        "colmap_sfm": 120.0,
        "orbslam3_rgbd": 90.0,
        "orbslam3_rgbd_imu": 120.0,
    }
    steps = [ProgressStep("normalize_input", 0.25, 90.0)]
    for method in methods:
        steps.append(ProgressStep(f"candidate:{method}", candidate_weight, candidate_expected.get(method, 120.0)))
    steps.extend(
        [
            ProgressStep("agreement", 0.10, 10.0),
            ProgressStep("persist_outputs", 0.05, 5.0),
        ]
    )
    return ProgressReporter(steps)


def run(
    cmd: list[str],
    log: Path | None = None,
    env: dict[str, str] | None = None,
    cwd: Path | None = None,
    progress: ProgressReporter | None = None,
    progress_label: str | None = None,
) -> int:
    if log is None:
        proc = subprocess.Popen(cmd, env=env, cwd=cwd)
        while proc.poll() is None:
            if progress is not None:
                progress.pulse(progress_label or Path(cmd[0]).name)
            time.sleep(1.0)
        return int(proc.returncode)
    log.parent.mkdir(parents=True, exist_ok=True)
    with log.open("a", encoding="utf-8") as fh:
        fh.write("+ " + " ".join(cmd) + "\n")
        fh.flush()
        proc = subprocess.Popen(cmd, env=env, cwd=cwd, stdout=fh, stderr=subprocess.STDOUT, text=True)
        while proc.poll() is None:
            if progress is not None:
                progress.pulse(progress_label or Path(cmd[0]).name)
            time.sleep(1.0)
        fh.write(f"[exit] {proc.returncode}\n")
        return int(proc.returncode)


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


def associate_streams_by_stamp(
    left: list[tuple[int, Any]],
    right: list[tuple[int, Any]],
    max_delta_ns: int,
) -> list[tuple[int, int]]:
    right_times = [item[0] for item in right]
    candidates = []
    for left_idx, (left_ns, _) in enumerate(left):
        idx = bisect.bisect_left(right_times, left_ns)
        for right_idx in {idx - 1, idx}:
            if 0 <= right_idx < len(right):
                delta = abs(right[right_idx][0] - left_ns)
                if delta <= max_delta_ns:
                    candidates.append((delta, left_idx, right_idx))

    assigned_left: set[int] = set()
    assigned_right: set[int] = set()
    assignments = []
    for delta, left_idx, right_idx in sorted(candidates, key=lambda item: item[0]):
        if left_idx in assigned_left or right_idx in assigned_right:
            continue
        assigned_left.add(left_idx)
        assigned_right.add(right_idx)
        assignments.append((left_idx, right_idx))
    assignments.sort(key=lambda item: item[0])
    return assignments


def nearest_message_by_stamp(stream: list[tuple[int, Any]], stamp_ns_value: int) -> Any:
    if not stream:
        raise RuntimeError("Cannot select nearest message from an empty stream")
    times = [item[0] for item in stream]
    idx = bisect.bisect_left(times, stamp_ns_value)
    choices = []
    if idx < len(stream):
        choices.append(stream[idx])
    if idx > 0:
        choices.append(stream[idx - 1])
    return min(choices, key=lambda item: abs(item[0] - stamp_ns_value))[1]


def sync_delta_stats(deltas_sec: list[float]) -> dict[str, Any]:
    if not deltas_sec:
        return {"count": 0}
    arr = np.asarray(deltas_sec, dtype=float)
    return {
        "count": int(arr.size),
        "min_abs_dt_sec": float(np.min(arr)),
        "median_abs_dt_sec": float(np.median(arr)),
        "p95_abs_dt_sec": float(np.percentile(arr, 95)),
        "max_abs_dt_sec": float(np.max(arr)),
    }


def build_sync_report(
    rgb_topic: str,
    depth_topic: str,
    info_topic: str,
    raw_color_count: int,
    raw_depth_count: int,
    raw_info_count: int,
    assignments: list[tuple[int, int]],
    colors: list[tuple[int, Any]],
    depths: list[tuple[int, Any]],
    max_delta_ns: int,
    profile: dict[str, Any],
) -> dict[str, Any]:
    deltas = [abs(colors[color_idx][0] - depths[depth_idx][0]) / 1e9 for color_idx, depth_idx in assignments]
    association_ratio = len(assignments) / raw_color_count if raw_color_count else 0.0
    min_ratio = float(profile.get("min_association_ratio", 0.8))
    status = "ok"
    reasons = []
    if not assignments:
        status = "failed"
        reasons.append("no_rgb_depth_pairs")
    if association_ratio < min_ratio:
        status = "failed"
        reasons.append(f"association_ratio_below_{min_ratio:g}")
    stats = sync_delta_stats(deltas)
    return {
        "status": status,
        "reasons": reasons,
        "rgb_topic": rgb_topic,
        "depth_topic": depth_topic,
        "camera_info_topic": info_topic,
        "raw_color_count": raw_color_count,
        "raw_depth_count": raw_depth_count,
        "raw_camera_info_count": raw_info_count,
        "associated_count": len(assignments),
        "dropped_color_count": max(0, raw_color_count - len(assignments)),
        "association_ratio": association_ratio,
        "max_allowed_dt_sec": max_delta_ns / 1e9,
        **stats,
        "policy": {
            "one_to_one_depth_assignment": True,
            "timestamp_source": "message header stamp with bag timestamp fallback",
            "min_association_ratio": min_ratio,
        },
    }


def write_sync_report(dataset: Path, report: dict[str, Any]) -> None:
    (dataset / "sync_report.json").write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")


def imu_to_row(msg: Any, fallback_ns: int) -> tuple[int, float, float, float, float, float, float]:
    ts_ns = msg_stamp_ns(msg, fallback_ns)
    acc = msg.linear_acceleration
    gyro = msg.angular_velocity
    return (
        ts_ns,
        float(acc.x),
        float(acc.y),
        float(acc.z),
        float(gyro.x),
        float(gyro.y),
        float(gyro.z),
    )


def write_imu_csv(rows: list[tuple[int, float, float, float, float, float, float]], dataset: Path) -> dict[str, Any]:
    path = dataset / "imu.csv"
    rows = sorted(rows, key=lambda item: item[0])
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["timestamp", "ax", "ay", "az", "gx", "gy", "gz"])
        for ts_ns, ax, ay, az, gx, gy, gz in rows:
            writer.writerow([f"{ns_to_sec(ts_ns):.9f}", ax, ay, az, gx, gy, gz])
    return {
        "imu_count": len(rows),
        "imu_first_timestamp": ns_to_sec(rows[0][0]) if rows else None,
        "imu_last_timestamp": ns_to_sec(rows[-1][0]) if rows else None,
    }


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
    """Check/clean the output location but do NOT create it yet.
    The caller is responsible for creating it after normalize succeeds,
    so a normalize crash doesn't leave an empty output directory behind.
    """
    if path.exists():
        if not force:
            raise SystemExit(f"Output already exists: {path}. Re-run with --force to replace it.")
        if path.is_dir():
            shutil.rmtree(path)
        else:
            path.unlink()


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
    if hasattr(msg, "format") and not hasattr(msg, "height"):
        return compressed_image_to_array(msg, is_depth=is_depth)

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


def compressed_image_to_array(msg: Any, is_depth: bool) -> np.ndarray:
    import cv2

    data = np.frombuffer(msg.data, dtype=np.uint8)
    fmt = getattr(msg, "format", "").lower()
    if is_depth and "compresseddepth" in fmt and data.size > 12:
        decoded = cv2.imdecode(data[12:], cv2.IMREAD_UNCHANGED)
    else:
        flags = cv2.IMREAD_UNCHANGED if is_depth else cv2.IMREAD_COLOR
        decoded = cv2.imdecode(data, flags)
    if decoded is None:
        kind = "depth" if is_depth else "color"
        raise RuntimeError(f"Could not decode compressed {kind} image with format '{getattr(msg, 'format', '')}'")
    if not is_depth and decoded.ndim == 2:
        return cv2.cvtColor(decoded, cv2.COLOR_GRAY2BGR)
    if is_depth and decoded.ndim == 3:
        decoded = decoded[:, :, 0]
    return decoded.copy()


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

        # Resolve target (w, h) from the first info_msg so every frame is
        # resized consistently when the SDK republishes at a different resolution
        # than what was originally calibrated (e.g. D435i bag at 640×480 but
        # SDK defaults to 1280×720 on replay).
        _ci_ref = write_camera_info_json(pairs[0][4], camera_info_json)
        _target_w = int(_ci_ref.get("width", 0))
        _target_h = int(_ci_ref.get("height", 0))
        if _target_w > 640 or _target_h > 480:
            _target_w = 640
            _target_h = 480

        for color_ns, color_msg, depth_ns, depth_msg, info_msg in pairs:
            ts = ns_to_sec(color_ns)
            if ts - last_kept_ts < min_dt:
                continue
            color = image_to_array(color_msg, is_depth=False)
            depth = normalize_depth(image_to_array(depth_msg, is_depth=True), depth_factor)
            # Resize to calibrated dimensions if SDK republished at a different resolution.
            if _target_w > 0 and _target_h > 0:
                ih, iw = color.shape[:2]
                if iw != _target_w or ih != _target_h:
                    color = cv2.resize(color, (_target_w, _target_h), interpolation=cv2.INTER_LINEAR)
                    depth = cv2.resize(depth, (_target_w, _target_h), interpolation=cv2.INTER_NEAREST)
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
    if camera_info is not None:
        write_calibration_yaml(dataset, camera_info)
    create_rtabmap_sync_dirs(dataset)
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
    imu_topic = choose_topic(available, profile.get("imu_topics", []), required=False)
    if imu_topic and topics.get(imu_topic) != "sensor_msgs/msg/Imu":
        imu_topic = None

    colors: list[tuple[int, Any]] = []
    depths: list[tuple[int, Any]] = []
    infos: list[tuple[int, Any]] = []
    imus: list[tuple[int, float, float, float, float, float, float]] = []
    selected = {rgb_topic, depth_topic, info_topic}
    if imu_topic:
        selected.add(imu_topic)
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
        elif topic == imu_topic:
            imus.append(imu_to_row(msg, bag_ns))

    if not colors or not depths or not infos:
        raise RuntimeError(
            f"Missing RGB-D data: colors={len(colors)} depths={len(depths)} camera_info={len(infos)}"
        )

    colors.sort(key=lambda item: item[0])
    depths.sort(key=lambda item: item[0])
    infos.sort(key=lambda item: item[0])
    max_delta_ns = int(float(profile.get("association_max_dt", 0.05)) * 1_000_000_000)
    assignments = associate_streams_by_stamp(colors, depths, max_delta_ns)
    sync_report = build_sync_report(
        rgb_topic,
        depth_topic,
        info_topic,
        raw_color_count=len(colors),
        raw_depth_count=len(depths),
        raw_info_count=len(infos),
        assignments=assignments,
        colors=colors,
        depths=depths,
        max_delta_ns=max_delta_ns,
        profile=profile,
    )
    write_sync_report(dataset, sync_report)
    if sync_report["status"] != "ok":
        raise RuntimeError(
            "ROS bag RGB-D sync failed: "
            + ", ".join(sync_report.get("reasons", []))
            + f" ({sync_report['associated_count']}/{sync_report['raw_color_count']} color frames associated)"
        )
    print(
        "[pseudo-gt] Sync ok: "
        f"{sync_report['associated_count']}/{sync_report['raw_color_count']} RGB frames associated, "
        f"median_dt={sync_report.get('median_abs_dt_sec', 0.0):.6f}s, "
        f"max_dt={sync_report.get('max_abs_dt_sec', 0.0):.6f}s",
        flush=True,
    )
    pairs: list[tuple[int, Any, int, Any, Any]] = []

    for color_idx, depth_idx in assignments:
        color_ns, color_msg = colors[color_idx]
        depth_ns, depth_msg = depths[depth_idx]
        info_msg = nearest_message_by_stamp(infos, color_ns)
        pairs.append((color_ns, color_msg, depth_ns, depth_msg, info_msg))

    result = write_frames(
        pairs,
        dataset,
        target_fps=target_fps,
        max_frames=max_frames,
        depth_factor=float(profile.get("depth_factor", 1000.0)),
    )
    imu_result = write_imu_csv(imus, dataset) if imus else {"imu_count": 0}
    result.update(
        {
            "rgb_topic": rgb_topic,
            "depth_topic": depth_topic,
            "camera_info_topic": info_topic,
            "imu_topic": imu_topic,
            "raw_color_count": len(colors),
            "raw_depth_count": len(depths),
            "raw_camera_info_count": len(infos),
            "raw_imu_count": len(imus),
            "associated_count": len(pairs),
            "sync": sync_report,
            **imu_result,
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
    return subprocess.Popen(cmd, stdout=fh, stderr=subprocess.STDOUT, text=True, start_new_session=True)


def terminate_processes(processes: list[subprocess.Popen]) -> None:
    for proc in processes:
        if proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
    time.sleep(2)
    for proc in processes:
        if proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass
    for proc in processes:
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            proc.wait()


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
        imu_topics = profile.get("imu_topics") or []
        if imu_topics:
            topics.append(imu_topics[0])
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
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image, Imu

    class LiveExtractor:
        def __init__(self) -> None:
            self.node = rclpy.create_node("pseudo_gt_live_rgbd_extractor")
            self.rgb_topic = profile["rgb_topics"][0]
            self.depth_topic = profile["depth_topics"][0]
            self.info_topic = profile["camera_info_topics"][0]
            self.imu_topic = profile.get("imu_topics", [None])[0]
            self.max_delta_ns = int(float(profile.get("association_max_dt", 0.05)) * 1_000_000_000)
            self.depths: list[tuple[int, Any]] = []
            self.colors: list[tuple[int, Any]] = []
            self.infos: list[tuple[int, Any]] = []
            self.imus: list[tuple[int, float, float, float, float, float, float]] = []
            self.last_msg_time = time.time()
            self.sub_rgb = self.node.create_subscription(Image, self.rgb_topic, self.on_rgb, qos_profile_sensor_data)
            self.sub_depth = self.node.create_subscription(Image, self.depth_topic, self.on_depth, qos_profile_sensor_data)
            self.sub_info = self.node.create_subscription(CameraInfo, self.info_topic, self.on_info, qos_profile_sensor_data)
            self.sub_imu = (
                self.node.create_subscription(Imu, self.imu_topic, self.on_imu, qos_profile_sensor_data)
                if self.imu_topic
                else None
            )
            self.timer = self.node.create_timer(0.5, self.on_timer)

        def on_rgb(self, msg: Any) -> None:
            self.last_msg_time = time.time()
            self.colors.append((stamp_to_ns(msg.header.stamp), msg))

        def on_depth(self, msg: Any) -> None:
            self.last_msg_time = time.time()
            self.depths.append((stamp_to_ns(msg.header.stamp), msg))

        def on_info(self, msg: Any) -> None:
            self.last_msg_time = time.time()
            self.infos.append((stamp_to_ns(msg.header.stamp), msg))

        def on_imu(self, msg: Any) -> None:
            self.last_msg_time = time.time()
            self.imus.append(imu_to_row(msg, stamp_to_ns(msg.header.stamp)))

        def has_enough_for_frame_cap(self) -> bool:
            if max_frames <= 0 or not self.infos or not self.colors or not self.depths:
                return False
            assignments = associate_streams_by_stamp(self.colors, self.depths, self.max_delta_ns)
            if len(assignments) < max_frames:
                return False
            if target_fps <= 0:
                return True
            span_sec = max(0.0, (self.colors[-1][0] - self.colors[0][0]) / 1e9)
            return span_sec >= max_frames / target_fps

        def on_timer(self) -> None:
            if self.has_enough_for_frame_cap():
                rclpy.shutdown()
            if playback_proc.poll() is not None and time.time() - self.last_msg_time > 3:
                rclpy.shutdown()
            if time.time() - self.last_msg_time > 20 and self.colors and self.depths:
                rclpy.shutdown()

    rclpy.init()
    extractor = LiveExtractor()
    try:
        rclpy.spin(extractor.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        colors = sorted(extractor.colors, key=lambda item: item[0])
        depths = sorted(extractor.depths, key=lambda item: item[0])
        infos = sorted(extractor.infos, key=lambda item: item[0])
        imus = sorted(extractor.imus, key=lambda item: item[0])
        extractor.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if not colors or not depths or not infos:
        raise RuntimeError(
            f"Missing live RGB-D data: colors={len(colors)} depths={len(depths)} camera_info={len(infos)}"
        )
    max_delta_ns = int(float(profile.get("association_max_dt", 0.05)) * 1_000_000_000)
    assignments = associate_streams_by_stamp(colors, depths, max_delta_ns)
    sync_report = build_sync_report(
        profile["rgb_topics"][0],
        profile["depth_topics"][0],
        profile["camera_info_topics"][0],
        raw_color_count=len(colors),
        raw_depth_count=len(depths),
        raw_info_count=len(infos),
        assignments=assignments,
        colors=colors,
        depths=depths,
        max_delta_ns=max_delta_ns,
        profile=profile,
    )
    write_sync_report(dataset, sync_report)
    if sync_report["status"] != "ok":
        raise RuntimeError(
            "Live RealSense RGB-D sync failed: "
            + ", ".join(sync_report.get("reasons", []))
            + f" ({sync_report['associated_count']}/{sync_report['raw_color_count']} color frames associated)"
        )
    print(
        "[pseudo-gt] Sync ok: "
        f"{sync_report['associated_count']}/{sync_report['raw_color_count']} RGB frames associated, "
        f"median_dt={sync_report.get('median_abs_dt_sec', 0.0):.6f}s, "
        f"max_dt={sync_report.get('max_abs_dt_sec', 0.0):.6f}s",
        flush=True,
    )

    pairs = []
    for color_idx, depth_idx in assignments:
        color_ns, color_msg = colors[color_idx]
        depth_ns, depth_msg = depths[depth_idx]
        info_msg = nearest_message_by_stamp(infos, color_ns)
        pairs.append((color_ns, color_msg, depth_ns, depth_msg, info_msg))

    result = write_frames(
        pairs,
        dataset,
        target_fps=target_fps,
        max_frames=max_frames,
        depth_factor=float(profile.get("depth_factor", 1000.0)),
    )
    imu_result = write_imu_csv(imus, dataset) if imus else {"imu_count": 0}
    result.update(
        {
            "raw_color_count": len(colors),
            "raw_depth_count": len(depths),
            "raw_camera_info_count": len(infos),
            "raw_imu_count": len(imus),
            "associated_count": len(pairs),
            "sync": sync_report,
            **imu_result,
        }
    )
    return result


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


def write_calibration_yaml(dataset: Path, camera_info: dict[str, Any]) -> None:
    """Write a ROS camera_info YAML for rtabmap-rgbd_dataset to read."""
    fx = float(camera_info["fx"])
    fy = float(camera_info["fy"])
    cx = float(camera_info["cx"])
    cy = float(camera_info["cy"])
    w = int(camera_info.get("width", 640))
    h = int(camera_info.get("height", 480))
    d = [float(v) for v in camera_info.get("d", [])]
    while len(d) < 5:
        d.append(0.0)
    d_str = "[" + ", ".join(f"{v}" for v in d) + "]"
    content = (
        f"image_width: {w}\n"
        f"image_height: {h}\n"
        f"camera_name: camera\n"
        f"camera_matrix:\n"
        f"  rows: 3\n"
        f"  cols: 3\n"
        f"  data: [{fx}, 0, {cx}, 0, {fy}, {cy}, 0, 0, 1]\n"
        f"distortion_model: plumb_bob\n"
        f"distortion_coefficients:\n"
        f"  rows: 1\n"
        f"  cols: 5\n"
        f"  data: {d_str}\n"
        # RTAB-Map's CameraModel parser historically checks this misspelled key.
        f"distorsion_coefficients:\n"
        f"  rows: 1\n"
        f"  cols: 5\n"
        f"  data: {d_str}\n"
        f"rectification_matrix:\n"
        f"  rows: 3\n"
        f"  cols: 3\n"
        f"  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]\n"
        f"projection_matrix:\n"
        f"  rows: 3\n"
        f"  cols: 4\n"
        f"  data: [{fx}, 0, {cx}, 0, 0, {fy}, {cy}, 0, 0, 0, 1, 0]\n"
    )
    (dataset / "calibration.yaml").write_text(content, encoding="utf-8")
    # rtabmap-rgbd_dataset with --output_name rtabmap looks for rtabmap_calib.yaml
    (dataset / "rtabmap_calib.yaml").write_text(content, encoding="utf-8")


def create_rtabmap_sync_dirs(dataset: Path) -> None:
    """Create rgb_sync/ and depth_sync/ with timestamp-named symlinks for rtabmap-rgbd_dataset.

    rtabmap-rgbd_dataset expects filenames to be parseable as floating-point timestamps.
    We symlink from timestamp names back to the frame_NNNNNN.png files in images/ and depth/.
    """
    assoc_txt = dataset / "associations.txt"
    if not assoc_txt.exists():
        return
    pairs = parse_tum_associations(assoc_txt)
    rgb_sync = dataset / "rgb_sync"
    depth_sync = dataset / "depth_sync"
    rgb_sync.mkdir(exist_ok=True)
    depth_sync.mkdir(exist_ok=True)
    for rgb_ts, rgb_rel, _depth_ts, depth_rel in pairs:
        rgb_src = dataset / rgb_rel
        dep_src = dataset / depth_rel
        rgb_link = rgb_sync / f"{rgb_ts:.6f}.png"
        dep_link = depth_sync / f"{rgb_ts:.6f}.png"
        if not rgb_link.exists() and rgb_src.exists():
            os.symlink(os.path.relpath(rgb_src, rgb_sync), rgb_link)
        if not dep_link.exists() and dep_src.exists():
            os.symlink(os.path.relpath(dep_src, depth_sync), dep_link)


def count_tum_rows(path: Path) -> int:
    if not path.exists():
        return 0
    count = 0
    with path.open("r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if line and not line.startswith("#"):
                count += 1
    return count


def normalized_frame_count(dataset: Path) -> int:
    assoc = dataset / "associations.txt"
    if assoc.exists():
        return count_tum_rows(assoc)
    frames = dataset / "frames.csv"
    if frames.exists():
        return max(0, count_tum_rows(frames) - 1)
    return 0


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


def normalize_hypersim(
    zip_path: Path,
    dataset: Path,
    profile: dict[str, Any],
    target_fps: float,
    max_frames: int,
) -> dict[str, Any]:
    import io
    import cv2
    import h5py
    import zipfile

    print(f"[pseudo-gt] Normalizing Hypersim scene from {zip_path}")

    # All Hypersim scenes use the same camera: 1024x768, fov_x = pi/3 (60°).
    # These match _vray_user_params.py in the dataset distribution.
    W, H = 1024, 768
    fov_x = math.pi / 3.0
    fx = (W / 2.0) / math.tan(fov_x / 2.0)
    fy = fx
    cx = W / 2.0
    cy = H / 2.0

    # Per-pixel factor converting Euclidean ray distance → perpendicular Z-depth.
    # Hypersim depth_meters is the Euclidean camera-to-surface distance along the
    # ray, not the Z-component. RGBD SLAM needs Z-depth (perpendicular to image
    # plane). Formula: z = d / sqrt(1 + ((u-cx)/fx)^2 + ((v-cy)/fy)^2).
    uu, vv = np.meshgrid(np.arange(W, dtype=np.float32), np.arange(H, dtype=np.float32))
    ray_to_z: np.ndarray = 1.0 / np.sqrt(1.0 + ((uu - cx) / fx) ** 2 + ((vv - cy) / fy) ** 2)

    depth_scale = float(profile.get("depth_factor", 1000.0))  # mm

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

    # Discover frame indices and ground-truth poses from the zip.
    color_pattern = "final_hdf5"
    depth_pattern = "geometry_hdf5"
    with zipfile.ZipFile(zip_path) as zf:
        names = zf.namelist()
        color_entries = sorted(
            n for n in names
            if color_pattern in n and n.endswith("color.hdf5")
        )
        depth_entries = {
            int(Path(n).name.split(".")[1]): n
            for n in names
            if depth_pattern in n and n.endswith("depth_meters.hdf5")
        }
        # Ground-truth: camera positions (asset units) and orientations (3×3 R).
        # Pick the camera that matches the color entries' camera name.
        cam_name_color = None
        if color_entries:
            # e.g. "ai_001_001/images/scene_cam_00_final_hdf5/frame.0000.color.hdf5"
            cam_name_color = color_entries[0].split("/")[2].replace("scene_", "").replace("_final_hdf5", "")
        pos_entry = next(
            (n for n in names if f"/{cam_name_color}/camera_keyframe_positions" in n),
            next((n for n in names if "camera_keyframe_positions" in n), None),
        )
        ori_entry = next(
            (n for n in names if f"/{cam_name_color}/camera_keyframe_orientations" in n),
            next((n for n in names if "camera_keyframe_orientations" in n), None),
        )
        gt_positions = None
        gt_orientations = None
        if pos_entry and ori_entry:
            gt_positions = h5py.File(io.BytesIO(zf.read(pos_entry)), "r")["dataset"][()].astype(np.float64)
            gt_orientations = h5py.File(io.BytesIO(zf.read(ori_entry)), "r")["dataset"][()].astype(np.float64)

    if not color_entries:
        raise RuntimeError(f"No color frames found in {zip_path}")

    kept = 0
    skipped = 0
    last_kept_ts = -math.inf
    min_dt = 1.0 / target_fps if target_fps > 0 else 0.0
    frame_time = float(profile.get("frame_time_seconds", 1.0))

    with (zipfile.ZipFile(zip_path) as zf,
          frames_csv.open("w", newline="", encoding="utf-8") as f_frames,
          quality_csv.open("w", newline="", encoding="utf-8") as f_quality,
          rgb_txt.open("w", encoding="utf-8") as f_rgb,
          depth_txt.open("w", encoding="utf-8") as f_depth,
          assoc_txt.open("w", encoding="utf-8") as f_assoc):

        frames_writer = csv.DictWriter(
            f_frames,
            fieldnames=["index", "timestamp", "rgb_file", "depth_file",
                        "color_stamp", "depth_stamp", "stamp_delta_sec"],
        )
        quality_writer = csv.DictWriter(
            f_quality,
            fieldnames=["index", "timestamp", "blur_score",
                        "exposure_clip_ratio", "depth_valid_ratio",
                        "accepted", "reason"],
        )
        frames_writer.writeheader()
        quality_writer.writeheader()

        for color_entry in color_entries:
            frame_idx = int(Path(color_entry).name.split(".")[1])
            ts = (frame_idx + 1) * frame_time  # +1 so ts>0; rtabmap rejects ts=0
            if ts - last_kept_ts < min_dt:
                continue
            depth_entry = depth_entries.get(frame_idx)
            if depth_entry is None:
                skipped += 1
                continue

            # Read color HDF5 (linear float16, shape HxWx3 RGB).
            color_bytes = zf.read(color_entry)
            with h5py.File(io.BytesIO(color_bytes), "r") as hf:
                color_linear = hf["dataset"][()].astype(np.float32)  # HxWx3

            # Read depth HDF5 (float16, Euclidean meters).
            depth_bytes = zf.read(depth_entry)
            with h5py.File(io.BytesIO(depth_bytes), "r") as hf:
                depth_ray = hf["dataset"][()].astype(np.float32)  # HxW

            # Convert linear HDR to uint8: per-image percentile tone map + gamma.
            valid_mask = np.isfinite(color_linear) & (color_linear >= 0)
            if valid_mask.any():
                p99 = float(np.percentile(color_linear[valid_mask], 99.5))
                scale = p99 if p99 > 1e-6 else 1.0
            else:
                scale = 1.0
            color_srgb = np.clip(color_linear / scale, 0.0, 1.0) ** (1.0 / 2.2)
            color_uint8 = (color_srgb * 255).astype(np.uint8)
            # HDF5 channel order is RGB; OpenCV uses BGR.
            color_bgr = color_uint8[:, :, ::-1]

            # Convert Euclidean depth to Z-depth in mm (uint16).
            depth_valid = np.isfinite(depth_ray) & (depth_ray > 0)
            depth_z = np.where(depth_valid, depth_ray * ray_to_z, 0.0)
            depth_mm = np.clip(depth_z * depth_scale, 0, 65535).astype(np.uint16)

            q_blur = blur_score(color_bgr)
            q_clip = exposure_clip_ratio(color_bgr)
            q_depth = float(depth_valid.mean())
            accepted = q_depth >= 0.05

            index = kept
            quality_writer.writerow({
                "index": index,
                "timestamp": f"{ts:.9f}",
                "blur_score": f"{q_blur:.6f}",
                "exposure_clip_ratio": f"{q_clip:.6f}",
                "depth_valid_ratio": f"{q_depth:.6f}",
                "accepted": str(accepted).lower(),
                "reason": "ok" if accepted else "depth_valid_ratio_low",
            })

            if not accepted:
                skipped += 1
                continue

            if kept == 0:
                write_camera_info_dict({
                    "width": W, "height": H,
                    "fx": fx, "fy": fy, "cx": cx, "cy": cy,
                    "distortion_model": "none", "d": [],
                    "depth_factor": depth_scale,
                }, camera_info_json)

            rgb_rel = f"images/frame_{index:06d}.png"
            dep_rel = f"depth/frame_{index:06d}.png"
            cv2.imwrite(str(dataset / rgb_rel), color_bgr)
            cv2.imwrite(str(dataset / dep_rel), depth_mm)
            f_rgb.write(f"{ts:.9f} {rgb_rel}\n")
            f_depth.write(f"{ts:.9f} {dep_rel}\n")
            f_assoc.write(f"{ts:.9f} {rgb_rel} {ts:.9f} {dep_rel}\n")
            frames_writer.writerow({
                "index": index, "timestamp": f"{ts:.9f}",
                "rgb_file": rgb_rel, "depth_file": dep_rel,
                "color_stamp": f"{ts:.9f}", "depth_stamp": f"{ts:.9f}",
                "stamp_delta_sec": "0.000000000",
            })
            kept += 1
            last_kept_ts = ts
            if max_frames > 0 and kept >= max_frames:
                break

    if kept == 0:
        raise RuntimeError("No Hypersim frames were successfully normalized.")

    # Write ground-truth trajectory in TUM format if camera poses are available.
    # Hypersim orientations are 3×3 rotation matrices (world-to-camera, OpenGL
    # convention: Y-up, -Z forward). Convert to quaternion for TUM format.
    has_gt = False
    if gt_positions is not None and gt_orientations is not None:
        gt_path = dataset / "groundtruth.txt"
        meters_per_unit = float(profile.get("meters_per_asset_unit", 0.0254))
        with gt_path.open("w", encoding="utf-8") as fgt:
            fgt.write("# Hypersim ground-truth camera trajectory (TUM format)\n")
            fgt.write("# timestamp tx ty tz qx qy qz qw\n")
            n_gt = min(len(gt_positions), len(gt_orientations))
            for fi in range(n_gt):
                ts_gt = (fi + 1) * frame_time  # +1 matches image timestamps (also +1)
                t = gt_positions[fi] * meters_per_unit
                R = gt_orientations[fi].reshape(3, 3)
                # Rotation matrix → quaternion (Hamilton convention).
                trace = R[0, 0] + R[1, 1] + R[2, 2]
                if trace > 0:
                    s = 0.5 / math.sqrt(trace + 1.0)
                    qw = 0.25 / s
                    qx = (R[2, 1] - R[1, 2]) * s
                    qy = (R[0, 2] - R[2, 0]) * s
                    qz = (R[1, 0] - R[0, 1]) * s
                elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                    s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                    qw = (R[2, 1] - R[1, 2]) / s
                    qx = 0.25 * s
                    qy = (R[0, 1] + R[1, 0]) / s
                    qz = (R[0, 2] + R[2, 0]) / s
                elif R[1, 1] > R[2, 2]:
                    s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                    qw = (R[0, 2] - R[2, 0]) / s
                    qx = (R[0, 1] + R[1, 0]) / s
                    qy = 0.25 * s
                    qz = (R[1, 2] + R[2, 1]) / s
                else:
                    s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                    qw = (R[1, 0] - R[0, 1]) / s
                    qx = (R[0, 2] + R[2, 0]) / s
                    qy = (R[1, 2] + R[2, 1]) / s
                    qz = 0.25 * s
                fgt.write(f"{ts_gt:.9f} {t[0]:.9f} {t[1]:.9f} {t[2]:.9f} "
                          f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n")
        print(f"[pseudo-gt] Hypersim: wrote {n_gt} GT poses to groundtruth.txt")
        has_gt = True

    print(f"[pseudo-gt] Hypersim: kept {kept} frames, skipped {skipped}")
    cam_info = {"width": W, "height": H, "fx": fx, "fy": fy, "cx": cx, "cy": cy, "depth_factor": depth_scale}
    write_calibration_yaml(dataset, cam_info)
    create_rtabmap_sync_dirs(dataset)
    return {
        "input_format": "hypersim",
        "frame_count": kept,
        "skipped": skipped,
        "has_groundtruth": has_gt,
        "camera_info": cam_info,
    }


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
            # Rescale to mm (1000 units/m) — canonical depth unit for all methods.
            if depth_factor != 1000.0:
                depth = (depth.astype(np.float32) * (1000.0 / depth_factor)).clip(0, 65535).astype(np.uint16)
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
    # Canonical invariant: depth_factor is always 1000 (mm) after normalization.
    camera_info["depth_factor"] = 1000.0
    write_camera_info_dict(camera_info, camera_info_json)
    write_calibration_yaml(dataset, camera_info)
    create_rtabmap_sync_dirs(dataset)
    return {
        "input_format": "tum_rgbd",
        "frame_count": kept,
        "associated_count": len(pairs),
        "skipped_missing": skipped_missing,
        "skipped_unreadable": skipped_unreadable,
        "camera_info": camera_info,
        "depth_factor": 1000.0,
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


def _zip_is_rosbag2(path: Path) -> bool:
    """Return True if the zip contains a rosbag2 archive (metadata.yaml + mcap/db3)."""
    try:
        import zipfile
        with zipfile.ZipFile(path) as zf:
            names = zf.namelist()
        return any(
            n == "metadata.yaml" or n.endswith("/metadata.yaml")
            or n.endswith(".mcap") or n.endswith(".db3")
            for n in names
        )
    except Exception:
        return False


def detect_input_format(path: Path, requested: str, profile: dict[str, Any]) -> str:
    if requested != "auto":
        return requested
    if profile.get("storage") == "tum_rgbd":
        return "tum_rgbd"
    if path.is_dir() and (path / "rgb.txt").exists() and (path / "depth.txt").exists():
        return "tum_rgbd"
    if path.suffix == ".zip":
        # Peek inside: a zip that contains metadata.yaml or mcap/db3 files is a
        # rosbag2 archive, not a Hypersim scene.
        if _zip_is_rosbag2(path):
            return "bag"
        return "hypersim"
    if profile.get("storage") == "hypersim":
        return "hypersim"
    return "bag"


def _find_and_copy_groundtruth(source: Path, dataset: Path) -> bool:
    """Look for a groundtruth file alongside source and copy it to dataset."""
    if (dataset / "groundtruth.txt").exists():
        return True  # already written by the normalizer (e.g. Hypersim)
    candidates: list[Path] = []
    if source.is_dir():
        candidates = [
            source / "groundtruth.txt",
            source / "groundtruth_tum.txt",
            source.parent / "groundtruth.txt",
            source.parent / "groundtruth_tum.txt",
        ]
    else:
        candidates = [
            source.parent / "groundtruth.txt",
            source.parent / "groundtruth_tum.txt",
            source.with_suffix(".groundtruth.txt"),
        ]
    for candidate in candidates:
        if candidate.exists():
            shutil.copy2(candidate, dataset / "groundtruth.txt")
            print(f"[pseudo-gt] GT trajectory: {candidate}")
            return True
    return False


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
    if input_format == "hypersim":
        result = normalize_hypersim(
            source,
            dataset,
            profile,
            target_fps=target_fps,
            max_frames=max_frames,
        )
    elif input_format == "tum_rgbd":
        result = normalize_tum_rgbd(
            source,
            dataset,
            profile,
            target_fps=target_fps,
            max_frames=max_frames,
        )
    else:
        result = normalize_bag(
            source,
            dataset,
            profile,
            target_fps=target_fps,
            max_frames=max_frames,
            log_dir=log_dir,
        )
    has_gt = _find_and_copy_groundtruth(source, dataset)
    result["has_groundtruth"] = result.get("has_groundtruth", has_gt)
    return result



def rtabmap_preset_args(preset: str, profile: dict[str, Any]) -> list[str]:
    if preset == "default":
        return [
            "--Vis/MinInliers", str(profile.get("rtabmap_vis_min_inliers", 8)),
            "--Kp/MaxFeatures", str(profile.get("rtabmap_kp_max_features", 500)),
            "--Rtabmap/DetectionRate", "0",
        ]
    # fast: halved features (500→250) vs default.
    # Empirically measured effect (freiburg1_desk 20fps, same 290 frames):
    #   odom: 31→26ms (-16%)  slam: 21→25ms (+18%)  total: 58→57ms (-2%)
    #   median frame: 52→48ms  on-budget (≤50ms): 44%→51%  wall fps: 16.8→17.1
    # The slam step self-compensates: fewer features → sparser local map →
    # higher KF insertion rate (45%→67%) → same total backend compute.
    # Mem/STMSize omitted — empirically worsened slam spikes (20→41 >50ms frames)
    # by triggering frequent STM→LTM evictions.
    # Net gain is modest (~2% wall fps, 7pp more on-budget frames).
    # For real-time at 20fps, reduce input to ≤10fps via --target-fps instead.
    if preset == "fast":
        return [
            "--Vis/MinInliers", str(profile.get("rtabmap_vis_min_inliers", 6)),
            "--Kp/MaxFeatures", "250",
            "--Rtabmap/DetectionRate", "0",
        ]
    args = [
        "--Vis/MinInliers", "20",
        "--Kp/MaxFeatures", "1500",
        "--Vis/MaxFeatures", "2000",
        "--Odom/Strategy", "0",
        "--Rtabmap/DetectionRate", "0",
    ]
    if preset == "robust":
        return args
    if preset == "f2f":
        f2f_args = list(args)
        f2f_args[f2f_args.index("--Odom/Strategy") + 1] = "1"
        return f2f_args
    if preset == "dense-keyframes":
        return [*args, "--Odom/KeyFrameThr", "0", "--Odom/VisKeyFrameThr", "0"]
    raise ValueError(f"Unknown RTAB-Map preset: {preset}")


def _rtabmap_export_candidates(db: Path) -> list[Path]:
    return [
        db.with_name(f"{db.stem}_odom.txt"),
        db.with_name(f"{db.name}_odom.txt"),
        db.parent / "rtabmap_odom.txt",
    ]


def export_rtabmap_node_poses(db: Path, output: Path) -> tuple[int, str]:
    import sqlite3
    import struct

    from scipy.spatial.transform import Rotation

    rows: list[tuple[float, np.ndarray, np.ndarray]] = []
    try:
        con = sqlite3.connect(str(db))
        cur = con.cursor()
        for stamp, blob in cur.execute("select stamp, pose from Node where pose is not null order by stamp"):
            if blob is None or len(blob) != 48:
                continue
            mat = np.asarray(struct.unpack("12f", blob), dtype=float).reshape(3, 4)
            q = Rotation.from_matrix(mat[:, :3]).as_quat()
            rows.append((float(stamp), mat[:, 3], q))
    except Exception as exc:
        return 0, str(exc)
    finally:
        try:
            con.close()  # type: ignore[name-defined]
        except Exception:
            pass

    if not rows:
        return 0, "no Node.pose rows found"
    with output.open("w", encoding="utf-8") as fh:
        fh.write("#timestamp x y z qx qy qz qw\n")
        for ts, t, q in rows:
            fh.write(
                f"{ts:.9f} {t[0]:.9f} {t[1]:.9f} {t[2]:.9f} "
                f"{q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n"
            )
    return len(rows), ""


def parse_rtabmap_log_metrics(log: Path) -> dict[str, Any]:
    metrics: dict[str, Any] = {
        "calibration_warning_count": 0,
        "local_transform": "",
    }
    if not log.exists():
        return metrics
    text = log.read_text(encoding="utf-8", errors="replace")
    metrics["calibration_warning_count"] = sum(
        1
        for line in text.splitlines()
        if "CameraModel.cpp" in line and "Missing" in line
    )
    import re

    match = re.search(r"Using local transform from calibration file \(([^)]*)\)", text)
    if match:
        metrics["local_transform"] = match.group(1)
    m_kp = re.search(r"Kp/MaxFeatures=(\d+)", text)
    if m_kp:
        metrics["kp_max_features"] = int(m_kp.group(1))
    trans = re.findall(r"translational_rmse=\s*([0-9.eE+-]+)", text)
    rot = re.findall(r"rotational_rmse=\s*([0-9.eE+-]+)", text)
    if trans:
        metrics["rtabmap_reported_translational_rmse"] = float(trans[-1])
    if rot:
        metrics["rtabmap_reported_rotational_rmse_deg"] = float(rot[-1])
    # Total wall-clock time reported by rtabmap-rgbd_dataset
    m_total = re.search(r"Total time=([0-9.]+)s", text)
    if m_total:
        total_sec = float(m_total.group(1))
        metrics["total_tracking_time_sec"] = total_sec
    # Per-step averages from the rtabmap-report summary line:
    #   slam: avg=34 ms (max=...), odom: avg=26ms (max=...), camera: avg=5ms
    m_slam = re.search(r"slam:\s*avg=([0-9]+)\s*ms", text)
    m_odom = re.search(r"odom:\s*avg=([0-9]+)\s*ms", text)
    m_cam = re.search(r"camera:\s*avg=([0-9]+)\s*ms", text)
    if m_slam:
        metrics["slam_avg_ms"] = int(m_slam.group(1))
    if m_odom:
        metrics["odom_avg_ms"] = int(m_odom.group(1))
    if m_cam:
        metrics["camera_avg_ms"] = int(m_cam.group(1))
    # runtime_fps: frames processed per wall-clock second (total_time basis)
    frame_match = re.search(r"Iteration (\d+)/(\d+):", text)
    if frame_match and m_total and total_sec > 0:
        n_frames = int(frame_match.group(2))
        metrics["runtime_fps"] = round(n_frames / total_sec, 2)
    elif m_slam and m_odom and m_cam:
        per_frame_ms = int(m_slam.group(1)) + int(m_odom.group(1)) + int(m_cam.group(1))
        if per_frame_ms > 0:
            metrics["runtime_fps"] = round(1000.0 / per_frame_ms, 2)
    return metrics


def parse_orbslam3_log_metrics(log: Path) -> dict[str, Any]:
    metrics: dict[str, Any] = {}
    if not log.exists():
        return metrics
    import re
    text = log.read_text(encoding="utf-8", errors="replace")
    m_mean = re.search(r"mean tracking time:\s*([0-9.eE+-]+)", text)
    m_med = re.search(r"median tracking time:\s*([0-9.eE+-]+)", text)
    if m_mean:
        mean_sec = float(m_mean.group(1))
        metrics["mean_tracking_time_sec"] = mean_sec
        if mean_sec > 0:
            metrics["runtime_fps"] = round(1.0 / mean_sec, 2)
    if m_med:
        metrics["median_tracking_time_sec"] = float(m_med.group(1))
    return metrics


def parse_colmap_log_metrics(log: Path) -> dict[str, Any]:
    """Extract per-stage elapsed times from COLMAP log (minutes → seconds).

    feature_extractor emits 1 elapsed-time line.
    sequential_matcher emits 2 (sift matching + geometric verification).
    mapper emits 1.
    Strategy: first line → extractor, last line → mapper, middle lines → matcher.
    """
    metrics: dict[str, Any] = {}
    if not log.exists():
        return metrics
    import re
    text = log.read_text(encoding="utf-8", errors="replace")
    elapsed = [float(v) for v in re.findall(r"Elapsed time:\s*([0-9.]+)\s*\[minutes\]", text)]
    if len(elapsed) >= 1:
        metrics["feature_extraction_sec"] = round(elapsed[0] * 60.0, 1)
    if len(elapsed) >= 2:
        # last = mapper; middle = matcher (sum of any sub-steps)
        metrics["mapper_sec"] = round(elapsed[-1] * 60.0, 1)
        middle = elapsed[1:-1]
        if middle:
            metrics["sequential_matching_sec"] = round(sum(middle) * 60.0, 1)
    return metrics


def export_rtabmap_dense_odom(dataset: Path, out_dir: Path, log: Path) -> tuple[Path | None, dict[str, Any], str]:
    db = out_dir / "rtabmap.db"
    metrics: dict[str, Any] = {
        "database": str(db),
        "expected_frame_count": normalized_frame_count(dataset),
    }
    if not db.exists() or db.stat().st_size == 0:
        return None, metrics, "rtabmap.db was not written"
    report_bin = shutil.which("rtabmap-report")
    if report_bin is None:
        return None, metrics, "rtabmap-report binary not found"

    cmd = [report_bin, "--poses_raw"]
    gt = dataset / "groundtruth.txt"
    if gt.exists():
        cmd += ["--gt", str(gt)]
    cmd.append(str(db))
    rc = run(cmd, log=log, progress_label="rtabmap_rgbd:export_odom")
    metrics["rtabmap_report_exit_code"] = rc
    if rc != 0:
        return None, metrics, "rtabmap-report --poses_raw failed"

    candidates = [p for p in _rtabmap_export_candidates(db) if p.exists() and p.stat().st_size > 0]
    if not candidates:
        candidates = sorted(out_dir.glob("*_odom.txt"))
    expected = int(metrics["expected_frame_count"])
    min_dense = math.ceil(expected * 0.90) if expected > 0 else 30

    dense = candidates[0] if candidates else None
    dense_count = count_tum_rows(dense) if dense is not None else 0
    source = "rtabmap-report"
    if dense_count < min_dense:
        node_dense = out_dir / "rtabmap_node_odom.txt"
        node_count, node_reason = export_rtabmap_node_poses(db, node_dense)
        metrics["database_node_pose_count"] = node_count
        if node_reason:
            metrics["database_node_pose_reason"] = node_reason
        if node_count >= min_dense:
            dense = node_dense
            dense_count = node_count
            source = "database_node_pose"
    if dense is None:
        return None, metrics, "dense odometry export was not written"

    metrics.update(
        {
            "dense_odom_export": str(dense),
            "dense_odom_source": source,
            "dense_odom_pose_count": dense_count,
            "dense_odom_min_pose_count": min_dense,
            "dense_odom_coverage_ratio": (dense_count / expected) if expected > 0 else None,
        }
    )
    if dense_count < min_dense:
        return None, metrics, "dense_odom_too_sparse"
    return dense, metrics, ""


def run_rtabmap_candidate(
    method: str,
    dataset: Path,
    out_dir: Path,
    profile: dict[str, Any],
    rtabmap_preset: str,
    progress: ProgressReporter | None = None,
) -> CandidateResult:
    if method == "rtabmap_rgbd_imu":
        log = out_dir / "run.log"
        out_dir.mkdir(parents=True, exist_ok=True)
        log.write_text("rtabmap_rgbd_imu is not supported in dataset mode (rtabmap-rgbd_dataset has no IMU path).\n", encoding="utf-8")
        return CandidateResult(method, "failed", None, log, {}, "rtabmap_rgbd_imu not supported in dataset mode")
    log = out_dir / "run.log"
    out_dir.mkdir(parents=True, exist_ok=True)
    tum = out_dir / "trajectory_tum.csv"
    binary = Path(os.environ.get("RTABMAP_RGBD_DATASET_BIN", "/opt/ros/jazzy/bin/rtabmap-rgbd_dataset"))
    if not binary.exists():
        return CandidateResult(method, "failed", None, log, {}, f"rtabmap-rgbd_dataset binary not found: {binary}")
    if not (dataset / "rgb_sync").exists() or not (dataset / "depth_sync").exists():
        return CandidateResult(method, "failed", None, log, {}, "dataset missing rgb_sync/ or depth_sync/ (normalization incomplete)")
    try:
        camera_info = json.loads((dataset / "camera_info.json").read_text(encoding="utf-8"))
        depth_factor = float(camera_info.get("depth_factor", 1000.0))
        cmd = [
            str(binary),
            "--output", str(out_dir),
            "--output_name", "rtabmap",
            *rtabmap_preset_args(rtabmap_preset, profile),
        ]
        # depth_factor should be 1000 (mm) after normalization; pass it explicitly to be safe
        if depth_factor != 1000.0:
            cmd += ["--RGBD/DepthScalingFactor", str(1000.0 / depth_factor)]
        cmd.append(str(dataset))
        if not os.environ.get("DISPLAY"):
            cmd = ["xvfb-run", "-a", "--server-args=-screen 0 1280x720x24", *cmd]
        rc = run(cmd, log=log, progress=progress, progress_label=f"{method}:track")
        dense, export_metrics, export_reason = export_rtabmap_dense_odom(dataset, out_dir, log)
        metrics = {
            "exit_code": rc,
            "preset": rtabmap_preset,
            **parse_rtabmap_log_metrics(log),
            **export_metrics,
        }
        if export_reason:
            metrics["dense_odom_export_reason"] = export_reason
        (out_dir / "rtabmap_metrics.json").write_text(json.dumps(metrics, indent=2, sort_keys=True), encoding="utf-8")
        if dense is None:
            reason = "dense_odom_export_failed" if export_reason != "dense_odom_too_sparse" else export_reason
            return CandidateResult(method, "failed", None, log, metrics, reason)
        poses_file = out_dir / "rtabmap_poses.txt"
        if not poses_file.exists() or poses_file.stat().st_size == 0:
            metrics["sparse_graph_pose_count"] = 0
        else:
            metrics["sparse_graph_pose_count"] = count_tum_rows(poses_file)
        (out_dir / "rtabmap_metrics.json").write_text(json.dumps(metrics, indent=2, sort_keys=True), encoding="utf-8")
        shutil.copy2(dense, tum)
        return CandidateResult(method, "ok", tum, log, metrics)
    except Exception as exc:
        return CandidateResult(method, "failed", None, log, {}, str(exc))


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
                "--SiftExtraction.max_image_size",
                "960",
                "--SiftExtraction.max_num_features",
                "4096",
            ],
            ["--SequentialMatching.overlap", "10"],
        )
    if preset == "robust":
        return (
            [
                "--SiftExtraction.max_image_size",
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
            ],
        )
    return (
        [
            "--SiftExtraction.max_image_size",
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
    progress: ProgressReporter | None = None,
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
            "--SiftExtraction.use_gpu",
            gpu_flag,
            *feature_args,
        ]
        match_cmd = [
            "colmap",
            "sequential_matcher",
            "--database_path",
            str(database),
            "--SiftMatching.guided_matching",
            "1",
            "--SiftMatching.use_gpu",
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
            rc = run(cmd, log=log, progress=progress, progress_label=f"colmap_sfm:{cmd[1]}")
            if rc != 0:
                return CandidateResult(method, "failed", None, log, {}, f"COLMAP command failed: {cmd[1]}")
        sparse_txt = convert_colmap_models_to_text(sparse, out_dir / "sparse_txt", log)
        metrics = export_colmap_tum(dataset, sparse_txt, tum)
        metrics["preset"] = preset
        metrics["use_gpu"] = gpu_flag
        metrics.update(parse_colmap_log_metrics(log))
        if not tum.exists() or tum.stat().st_size == 0:
            return CandidateResult(method, "failed", None, log, metrics, "COLMAP registered no exportable frames")
        return CandidateResult(method, "ok", tum, log, metrics)
    except Exception as exc:
        return CandidateResult(method, "failed", None, log, {}, str(exc))


def estimate_camera_fps(dataset: Path, default: float = 30.0) -> float:
    assoc = dataset / "associations.txt"
    if not assoc.exists():
        return default
    timestamps: list[float] = []
    with assoc.open("r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if not parts:
                continue
            try:
                timestamps.append(float(parts[0]))
            except ValueError:
                continue
    if len(timestamps) < 2:
        return default
    diffs = np.diff(np.asarray(timestamps, dtype=float))
    diffs = diffs[diffs > 0]
    if diffs.size == 0:
        return default
    return float(1.0 / float(np.median(diffs)))


def write_orbslam3_settings(
    dataset: Path,
    out_dir: Path,
    profile: dict[str, Any] | None = None,
    with_imu: bool = False,
) -> Path:
    camera_info = json.loads((dataset / "camera_info.json").read_text(encoding="utf-8"))
    profile = profile or {}
    fx, fy, cx, cy = camera_params_from_info(camera_info)
    width = int(camera_info.get("width", 640))
    height = int(camera_info.get("height", 480))
    depth_factor = float(camera_info.get("depth_factor", 1000.0))
    distortion = [float(v) for v in camera_info.get("d", [])]
    while len(distortion) < 5:
        distortion.append(0.0)
    stereo_b = float(camera_info.get("stereo_b", 0.07732))
    fps = max(1, int(round(estimate_camera_fps(dataset, default=30.0))))
    settings = out_dir / "orbslam3_rgbd.yaml"
    lines = [
        "%YAML:1.0",
        'File.version: "1.0"',
        'Camera.type: "PinHole"',
        f"Camera1.fx: {fx}",
        f"Camera1.fy: {fy}",
        f"Camera1.cx: {cx}",
        f"Camera1.cy: {cy}",
        f"Camera1.k1: {distortion[0]}",
        f"Camera1.k2: {distortion[1]}",
        f"Camera1.p1: {distortion[2]}",
        f"Camera1.p2: {distortion[3]}",
        f"Camera1.k3: {distortion[4]}",
        f"Camera.width: {width}",
        f"Camera.height: {height}",
        f"Camera.fps: {fps}",
        "Camera.RGB: 1",
        "Stereo.ThDepth: 40.0",
        f"Stereo.b: {stereo_b}",
        f"RGBD.DepthMapFactor: {depth_factor}",
    ]
    if with_imu:
        orb_imu = profile.get("orbslam3_rgbd_imu")
        if not orb_imu:
            raise ValueError(
                "orbslam3_rgbd_imu requires an 'orbslam3_rgbd_imu' block in the "
                "profile (T_b_c1, noise_gyro, noise_acc, gyro_walk, acc_walk, frequency). "
                "Refusing to fall back to D435i defaults on a non-D435i sensor."
            )
        t_b_c1 = orb_imu.get("T_b_c1")
        if t_b_c1 is None or len(list(t_b_c1)) != 16:
            raise ValueError(
                f"profile.orbslam3_rgbd_imu.T_b_c1 must be a 16-element row-major 4x4 matrix; "
                f"got {len(list(t_b_c1)) if t_b_c1 is not None else 'None'} elements"
            )
        lines.extend(
            [
                "IMU.T_b_c1: !!opencv-matrix",
                "   rows: 4",
                "   cols: 4",
                "   dt: f",
                "   data: [" + ", ".join(str(float(v)) for v in t_b_c1) + "]",
                f"IMU.InsertKFsWhenLost: {int(orb_imu.get('insert_kfs_when_lost', 0))}",
                f"IMU.NoiseGyro: {float(orb_imu.get('noise_gyro', 1e-2))}",
                f"IMU.NoiseAcc: {float(orb_imu.get('noise_acc', 1e-1))}",
                f"IMU.GyroWalk: {float(orb_imu.get('gyro_walk', 1e-6))}",
                f"IMU.AccWalk: {float(orb_imu.get('acc_walk', 1e-4))}",
                f"IMU.Frequency: {float(orb_imu.get('frequency', 200.0))}",
            ]
        )
    # Hypersim renders are locally low-gradient; more features and lower FAST thresholds
    # improve tracking on pre-rendered synthetic imagery.
    is_hypersim = profile.get("name") == "hypersim"
    lines.extend(
        [
            "ORBextractor.nFeatures: 2000" if is_hypersim else ("ORBextractor.nFeatures: 1250" if with_imu else "ORBextractor.nFeatures: 1000"),
            "ORBextractor.scaleFactor: 1.2",
            "ORBextractor.nLevels: 10" if is_hypersim else "ORBextractor.nLevels: 8",
            "ORBextractor.iniThFAST: 12" if is_hypersim else "ORBextractor.iniThFAST: 20",
            "ORBextractor.minThFAST: 5" if is_hypersim else "ORBextractor.minThFAST: 7",
            "Viewer.KeyFrameSize: 0.05",
            "Viewer.KeyFrameLineWidth: 1.0",
            "Viewer.GraphLineWidth: 0.9",
            "Viewer.PointSize: 2.0",
            "Viewer.CameraSize: 0.08",
            "Viewer.CameraLineWidth: 3.0",
            "Viewer.ViewpointX: 0.0",
            "Viewer.ViewpointY: -0.7",
            "Viewer.ViewpointZ: -1.8",
            "Viewer.ViewpointF: 500.0",
            "",
        ]
    )
    settings.write_text("\n".join(lines), encoding="utf-8")
    return settings


def run_orbslam3_candidate(
    dataset: Path,
    out_dir: Path,
    profile: dict[str, Any] | None = None,
    method: str = "orbslam3_rgbd",
    progress: ProgressReporter | None = None,
) -> CandidateResult:
    with_imu = method == "orbslam3_rgbd_imu"
    log = out_dir / "run.log"
    out_dir.mkdir(parents=True, exist_ok=True)
    tum = out_dir / "trajectory_tum.csv"
    binary_env = "ORB_SLAM3_RGBD_IMU_BIN" if with_imu else "ORB_SLAM3_RGBD_BIN"
    binary_default = (
        "/opt/ORB_SLAM3/Examples/RGB-D-Inertial/rgbd_inertial_dataset"
        if with_imu
        else "/opt/ORB_SLAM3/Examples/RGB-D/rgbd_tum"
    )
    binary = Path(os.environ.get(binary_env, binary_default))
    vocab = Path(os.environ.get("ORB_SLAM3_VOCAB", "/opt/ORB_SLAM3/Vocabulary/ORBvoc.txt"))
    if with_imu and not (dataset / "imu.csv").exists():
        return CandidateResult(method, "failed", None, log, {}, "normalized dataset has no imu.csv")
    if with_imu and not (profile or {}).get("orbslam3_rgbd_imu"):
        return CandidateResult(
            method,
            "failed",
            None,
            log,
            {},
            "profile has no orbslam3_rgbd_imu block (required: T_b_c1, noise_gyro, noise_acc, gyro_walk, acc_walk, frequency)",
        )
    if not binary.exists():
        return CandidateResult(method, "failed", None, log, {}, f"ORB-SLAM3 binary not found: {binary}")
    if not vocab.exists():
        return CandidateResult(method, "failed", None, log, {}, f"ORB-SLAM3 vocabulary not found: {vocab}")
    try:
        settings = write_orbslam3_settings(dataset, out_dir, profile=profile, with_imu=with_imu)
    except ValueError as exc:
        return CandidateResult(method, "failed", None, log, {}, str(exc))
    cmd = [str(binary), str(vocab), str(settings), str(dataset), str(dataset / "associations.txt")]
    if with_imu:
        cmd.append(str(dataset / "imu.csv"))
    if not os.environ.get("DISPLAY"):
        cmd = ["xvfb-run", "-a", "--server-args=-screen 0 1280x720x24", *cmd]
    rc = run(cmd, log=log, cwd=out_dir, progress=progress, progress_label=f"{method}:track")
    timing = parse_orbslam3_log_metrics(log)
    for candidate in (out_dir / "CameraTrajectory.txt", out_dir / "KeyFrameTrajectory.txt"):
        if candidate.exists() and candidate.stat().st_size > 0:
            shutil.copy2(candidate, tum)
            return CandidateResult(method, "ok", tum, log, {"source": candidate.name, "exit_code": rc, **timing})
    if rc != 0:
        return CandidateResult(method, "failed", None, log, {"exit_code": rc, **timing}, "ORB-SLAM3 command failed")
    return CandidateResult(method, "failed", None, log, {**timing}, "ORB-SLAM3 did not write a trajectory")


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
        and 1e-3 < scale < 1e3  # reject collapsed/exploded Sim3 fits
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


def evaluate_agreement(
    results: list[CandidateResult],
    diagnostics: Path,
    allow_unreliable: bool,
    dataset: Path | None = None,
) -> dict[str, Any]:
    diagnostics.mkdir(parents=True, exist_ok=True)
    healthy: dict[str, dict[str, np.ndarray]] = {}
    health: dict[str, dict[str, Any]] = {}
    for result in results:
        if result.status != "ok" or result.trajectory is None:
            health[result.method] = {"status": result.status, "reason": result.reason, "poses": 0}
            continue
        traj = read_tum(result.trajectory)
        poses = int(len(traj["t"]))
        if poses < 30:
            health[result.method] = {
                "status": "unhealthy",
                "reason": "too_few_poses",
                "poses": poses,
                "duration_sec": trajectory_duration(traj),
            }
            continue
        # Reject degenerate (all-identity) trajectories: max extent < 5 cm
        pos_extent = float(np.linalg.norm(np.max(traj["p"], axis=0) - np.min(traj["p"], axis=0)))
        if pos_extent < 0.05:
            health[result.method] = {
                "status": "unhealthy",
                "reason": "degenerate_trajectory",
                "poses": poses,
                "duration_sec": trajectory_duration(traj),
                "pos_extent_m": pos_extent,
            }
            continue
        health[result.method] = {
            "status": "ok",
            "reason": "ok",
            "poses": poses,
            "duration_sec": trajectory_duration(traj),
        }
        healthy[result.method] = traj

    run_duration = max((trajectory_duration(t) for t in healthy.values()), default=0.0)
    pairwise = []
    names = sorted(healthy)
    for i, name_a in enumerate(names):
        for name_b in names[i + 1 :]:
            pairwise.append(evaluate_pair(name_a, healthy[name_a], name_b, healthy[name_b], run_duration))

    # A winner must cover at least 25 % of the longest healthy trajectory's
    # duration. This prevents two short-lived variants of the same algorithm
    # (e.g. rtabmap_rgbd + rtabmap_rgbd_imu both losing tracking at 24 s) from
    # agreeing with each other and falsely winning over longer-coverage methods.
    max_duration = max((health[n].get("duration_sec", 0.0) for n in names), default=0.0)
    min_winner_duration = max_duration * 0.25

    support = {name: 0 for name in names}
    errors = {name: [] for name in names}
    for pair in pairwise:
        if pair.get("agree"):
            support[pair["method_a"]] += 1
            support[pair["method_b"]] += 1
            errors[pair["method_a"]].append(pair.get("median", math.inf))
            errors[pair["method_b"]].append(pair.get("median", math.inf))

    # Compare each healthy candidate against ground truth if available.
    gt_comparisons: list[dict[str, Any]] = []
    if dataset is not None and healthy:
        gt_path = dataset / "groundtruth.txt"
        if gt_path.exists():
            gt_traj = read_tum(gt_path)
            _gt_nonzero = int(np.any(gt_traj["p"] != 0, axis=1).sum()) if len(gt_traj["t"]) else 0
            _gt_ok = len(gt_traj["t"]) >= 3 and _gt_nonzero >= max(3, len(gt_traj["t"]) * 0.05)
            if not _gt_ok:
                print(
                    f"[pseudo-gt] Skipping GT comparison: only {_gt_nonzero}/{len(gt_traj['t'])} "
                    "non-zero poses (degenerate GT file)"
                )
            if _gt_ok:
                print(f"[pseudo-gt] Comparing {len(healthy)} method(s) against ground truth ({len(gt_traj['t'])} poses)")
                for name, traj in sorted(healthy.items()):
                    comp = evaluate_pair(name, traj, "ground_truth", gt_traj, run_duration)
                    comp["method"] = name
                    gt_comparisons.append(comp)

    gt_errors = {
        comp["method"]: (float(comp.get("rmse", math.inf)), float(comp.get("median", math.inf)))
        for comp in gt_comparisons
        if comp.get("method") in healthy and math.isfinite(float(comp.get("rmse", math.inf)))
    }

    supported = [
        name
        for name, count in support.items()
        if count > 0 and health[name].get("duration_sec", 0.0) >= min_winner_duration
    ]
    winner = None
    confidence = "none"
    if supported:
        winner = sorted(
            supported,
            key=lambda name: (
                -support[name],
                gt_errors.get(name, (math.inf, math.inf))[0],
                gt_errors.get(name, (math.inf, math.inf))[1],
                float(np.median(errors[name])) if errors[name] else math.inf,
                -health[name].get("duration_sec", 0.0),
                name,
            ),
        )[0]
        confidence = "high" if support[winner] >= 2 else "medium"
    elif allow_unreliable and names:
        winner = sorted(
            names,
            key=lambda name: (
                gt_errors.get(name, (math.inf, math.inf))[0],
                gt_errors.get(name, (math.inf, math.inf))[1],
                -health[name].get("poses", 0),
                name,
            ),
        )[0]
        confidence = "low"

    # Per-method runtime performance extracted from candidate metrics.
    _timing_keys = [
        "runtime_fps", "mean_tracking_time_sec", "median_tracking_time_sec",
        "total_tracking_time_sec", "slam_avg_ms", "odom_avg_ms", "camera_avg_ms",
        "kp_max_features",
        "feature_extraction_sec", "sequential_matching_sec", "mapper_sec",
    ]
    timing: dict[str, dict[str, Any]] = {}
    for result in results:
        m = {k: result.metrics[k] for k in _timing_keys if k in result.metrics}
        if m:
            timing[result.method] = m

    agreement = {
        "status": "ok" if supported else "agreement_failed",
        "winner": winner,
        "confidence": confidence,
        "support": support,
        "health": health,
        "pairwise": pairwise,
        "gt_comparisons": gt_comparisons,
        "timing": timing,
        "policy": {
            "min_pairs": 30,
            "rmse_max_m": 0.20,
            "median_max_m": 0.10,
            "yaw_drift": "diagnostic_only",
            "max_gap_sec": 5.0,
            "min_overlap": "min(10s, 20% of run duration)",
        },
    }
    (diagnostics / "agreement.json").write_text(json.dumps(agreement, indent=2, sort_keys=True), encoding="utf-8")

    if gt_comparisons:
        gt_fields = ["method", "pairs", "rmse", "median", "overlap_sec", "max_gap_sec", "yaw_drift_deg", "scale_a_to_b", "reason"]
        with (diagnostics / "gt_comparison.csv").open("w", newline="", encoding="utf-8") as fh:
            writer = csv.DictWriter(fh, fieldnames=gt_fields)
            writer.writeheader()
            for comp in gt_comparisons:
                writer.writerow({k: comp.get(k, "") for k in gt_fields})

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
    gt_comps = agreement.get("gt_comparisons", [])
    if gt_comps:
        lines.extend(["", "## Comparison vs Ground Truth (Sim3-aligned ATE)", ""])
        for comp in gt_comps:
            rmse = comp.get("rmse")
            median = comp.get("median")
            pairs = comp.get("pairs", 0)
            rmse_str = f"{rmse:.4f}m" if rmse is not None else "n/a"
            med_str = f"{median:.4f}m" if median is not None else "n/a"
            lines.append(f"- `{comp['method']}`: rmse={rmse_str} median={med_str} pairs={pairs}")
    timing = agreement.get("timing", {})
    if timing:
        lines.extend(["", "## Runtime Performance (tracking throughput)", ""])
        for method in sorted(timing):
            t = timing[method]
            parts = []
            fps = t.get("runtime_fps")
            if fps is not None:
                parts.append(f"**{fps:.1f} fps**")
            # ORB-SLAM3 per-frame tracking time
            mean_t = t.get("mean_tracking_time_sec")
            med_t = t.get("median_tracking_time_sec")
            if mean_t is not None:
                parts.append(f"mean={mean_t*1000:.1f}ms/frame")
            if med_t is not None:
                parts.append(f"median={med_t*1000:.1f}ms/frame")
            # RTAB-Map per-step averages
            slam_ms = t.get("slam_avg_ms")
            odom_ms = t.get("odom_avg_ms")
            cam_ms = t.get("camera_avg_ms")
            kp = t.get("kp_max_features")
            if kp is not None:
                parts.append(f"features={kp}")
            if slam_ms is not None or odom_ms is not None:
                step_parts = []
                if cam_ms is not None:
                    step_parts.append(f"camera={cam_ms}ms")
                if odom_ms is not None:
                    step_parts.append(f"odom={odom_ms}ms")
                if slam_ms is not None:
                    step_parts.append(f"slam={slam_ms}ms")
                parts.append(f"per-step: {', '.join(step_parts)}")
            total_sec = t.get("total_tracking_time_sec")
            if total_sec is not None:
                parts.append(f"total={total_sec:.1f}s")
            # COLMAP stage timings
            feat_sec = t.get("feature_extraction_sec")
            match_sec = t.get("sequential_matching_sec")
            map_sec = t.get("mapper_sec")
            if feat_sec is not None or match_sec is not None or map_sec is not None:
                colmap_parts = []
                if feat_sec is not None:
                    colmap_parts.append(f"feat={feat_sec:.0f}s")
                if match_sec is not None:
                    colmap_parts.append(f"match={match_sec:.0f}s")
                if map_sec is not None:
                    colmap_parts.append(f"map={map_sec:.0f}s")
                parts.append(f"stages: {', '.join(colmap_parts)}")
            lines.append(f"- `{method}`: {' | '.join(parts) if parts else 'n/a'}")
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

    # Sim3-align all trajectories to a common depth-based reference so that
    # monocular methods (colmap_sfm) appear at the correct metric scale.
    # When a trajectory has no temporal overlap with the reference, we align it
    # transitively through any already-aligned trajectory that does overlap.

    def _try_sim3(src_p: np.ndarray, src_traj: dict, dst_traj: dict):
        """Return (scale, R, t) aligning src to dst, or None if insufficient overlap."""
        ia, ib = associate_trajectories(src_traj, dst_traj)
        if len(ia) < 5:
            return None
        try:
            s, R, t = umeyama_sim3(src_p[ia], dst_traj["p"][ib])
            return (s, R, t) if 1e-4 < s < 1e4 else None
        except Exception:
            return None

    # Pick the reference as the depth-based method that overlaps with the most others.
    _ref_pref = ["rtabmap_rgbd_imu", "rtabmap_rgbd", "orbslam3_rgbd_imu", "orbslam3_rgbd", "colmap_sfm"]

    def _overlap_count(name: str) -> int:
        traj = trajectories[name]
        return sum(
            1
            for other, other_traj in trajectories.items()
            if other != name and len(associate_trajectories(traj, other_traj)[0]) >= 5
        )

    _ref_pref_present = [n for n in _ref_pref if n in trajectories]
    if _ref_pref_present:
        # Among depth-based candidates prefer the one with most overlapping neighbours.
        ref_name = max(_ref_pref_present, key=_overlap_count)
    else:
        ref_name = max(trajectories, key=lambda n: len(trajectories[n]["t"]))

    ref_traj = trajectories[ref_name]
    # Aligned dict stores trajectories remapped into ref_name's coordinate frame.
    aligned: dict[str, dict[str, np.ndarray]] = {ref_name: ref_traj}

    # Pass 1: direct alignment to reference.
    unaligned = {}
    for name, traj in trajectories.items():
        if name == ref_name or len(traj["t"]) == 0:
            continue
        result = _try_sim3(traj["p"], traj, ref_traj)
        if result:
            s, R, t = result
            aligned[name] = {**traj, "p": apply_sim3(traj["p"], s, R, t)}
        else:
            unaligned[name] = traj

    # Pass 2: transitive alignment — align remaining trajectories through any
    # trajectory that was already aligned in pass 1.
    for name, traj in unaligned.items():
        best = None
        for pivot_name, pivot_traj in aligned.items():
            if pivot_name == ref_name:
                continue  # already tried direct alignment to ref
            result = _try_sim3(traj["p"], traj, pivot_traj)
            if result:
                best = result
                break
        if best:
            s, R, t = best
            aligned[name] = {**traj, "p": apply_sim3(traj["p"], s, R, t)}
        else:
            aligned[name] = traj  # give up, show raw

    plt.figure(figsize=(8, 6))
    for name, traj in sorted(aligned.items()):
        if len(traj["p"]):
            plt.plot(traj["p"][:, 0], traj["p"][:, 1], label=name)
    plt.axis("equal")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title(f"Sim3-aligned to {ref_name}", fontsize=9)
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
        if result.method.startswith("rtabmap_") and result.log is not None:
            candidate_src = result.log.parent
            for pattern in ("rtabmap.db", "rtabmap_poses.txt", "*_odom.txt", "*_slam.txt", "*_gt.txt", "rtabmap_metrics.json"):
                for item in sorted(candidate_src.glob(pattern)):
                    if item.is_file():
                        shutil.copy2(item, dest_dir / item.name)

    diag_src = workspace / "diagnostics"
    if diag_src.exists():
        for item in diag_src.iterdir():
            dest = output / "diagnostics" / item.name
            if item.is_file():
                shutil.copy2(item, dest)
    for item in (workspace / "extraction_manifest.json", workspace / "dataset" / "sync_report.json"):
        if item.exists():
            shutil.copy2(item, output / "diagnostics" / item.name)
    gt_file = workspace / "dataset" / "groundtruth.txt"
    if gt_file.exists():
        shutil.copy2(gt_file, output / "diagnostics" / "groundtruth.txt")

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
    dataset: Path,
    workspace: Path,
    profile: dict[str, Any],
    colmap_preset: str,
    colmap_use_gpu: str,
    rtabmap_preset: str,
    progress: ProgressReporter | None = None,
) -> list[CandidateResult]:
    results = []
    disabled = set(profile.get("disabled_methods", []))
    for method in methods:
        out_dir = workspace / "candidates" / method
        if progress is not None:
            progress.start(f"candidate:{method}")
        if method in disabled:
            log = out_dir / "run.log"
            out_dir.mkdir(parents=True, exist_ok=True)
            log.write_text(f"{method} disabled by profile.\n", encoding="utf-8")
            result = CandidateResult(method, "failed", None, log, {}, "disabled by profile")
            print(f"[pseudo-gt] {method}: skipped (disabled by profile)")
            if progress is not None:
                progress.done(f"candidate:{method}")
            results.append(result)
            continue
        print(f"[pseudo-gt] Running candidate: {method}")
        if method in {"rtabmap_rgbd", "rtabmap_rgbd_imu"}:
            result = run_rtabmap_candidate(method, dataset, out_dir, profile, rtabmap_preset, progress=progress)
        elif method == "colmap_sfm":
            result = run_colmap_candidate(dataset, out_dir, colmap_preset, colmap_use_gpu, progress=progress)
        elif method in {"orbslam3_rgbd", "orbslam3_rgbd_imu"}:
            result = run_orbslam3_candidate(dataset, out_dir, profile=profile, method=method, progress=progress)
        else:
            result = CandidateResult(method, "failed", None, None, {}, "unknown method")
        print(f"[pseudo-gt] {method}: {result.status} {result.reason}")
        if progress is not None:
            progress.done(f"candidate:{method}")
        results.append(result)
    return results


def write_extraction_manifest(workspace: Path, extraction: dict[str, Any], profile: dict[str, Any], args: argparse.Namespace) -> None:
    manifest = {
        "profile": profile["name"],
        "input_format": profile.get("input_format", args.input_format),
        "workspace_mode": args.workspace_mode,
        "target_fps": args.target_fps,
        "max_frames": args.max_frames,
        "rtabmap_preset": getattr(args, "rtabmap_preset", "default"),
        "extraction": extraction,
    }
    (workspace / "extraction_manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Build a best pseudo-GT trajectory from RGB-D bags or TUM RGB-D sequences.")
    parser.add_argument("bag", type=Path)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--profile-config", default=Path("/work/config/pseudo_gt_profiles.yaml"), type=Path)
    parser.add_argument("--input-format", default="auto", choices=["auto", "bag", "tum_rgbd", "hypersim"])
    parser.add_argument("--methods", default=DEFAULT_METHODS)
    parser.add_argument("--colmap-preset", default="stable", choices=["fast", "stable", "robust"])
    parser.add_argument("--colmap-use-gpu", default=os.environ.get("PSEUDO_GT_COLMAP_USE_GPU", "0"), choices=["0", "1"])
    parser.add_argument("--rtabmap-preset", default="default", choices=["default", "fast", "robust", "f2f", "dense-keyframes"])
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
    progress = make_progress_reporter(methods)
    ensure_empty_output(output, args.force)
    workspace = make_workspace(output, args.workspace_mode, args.keep_workspace)
    # Assign a unique ROS domain ID so concurrent pipeline runs on the same host
    # don't cross-contaminate each other's /rtabmap/odom or other topics.
    domain_id = random.randint(1, 101)
    os.environ["ROS_DOMAIN_ID"] = str(domain_id)
    print(f"[pseudo-gt] Workspace: {workspace}")
    print(f"[pseudo-gt] Output: {output}")
    print(f"[pseudo-gt] Input format: {input_format}")
    print(f"[pseudo-gt] ROS_DOMAIN_ID: {domain_id}")

    try:
        dataset = workspace / "dataset"
        progress.start("normalize_input")
        extraction = normalize_input(
            bag,
            dataset,
            profile,
            input_format,
            target_fps=args.target_fps,
            max_frames=args.max_frames,
            log_dir=workspace / "logs",
        )
        progress.done("normalize_input")
        # Create the output dir only after normalize succeeds so that a
        # normalize crash doesn't leave an empty directory in the output tree.
        output.mkdir(parents=True, exist_ok=True)
        if extraction.get("rgb_topic"):
            profile["rgb_topics"] = [extraction["rgb_topic"]]
        if extraction.get("depth_topic"):
            profile["depth_topics"] = [extraction["depth_topic"]]
        if extraction.get("camera_info_topic"):
            profile["camera_info_topics"] = [extraction["camera_info_topic"]]
        if "imu_topic" in extraction:
            profile["imu_topics"] = [extraction["imu_topic"]] if extraction["imu_topic"] else []
        write_extraction_manifest(workspace, extraction, profile, args)
        # Profile YAML may specify rtabmap_preset to tune speed for a given
        # camera type (e.g. "fast" for 30fps RealSense). CLI flag always wins.
        rtabmap_preset = args.rtabmap_preset
        if rtabmap_preset == "default" and profile.get("rtabmap_preset"):
            rtabmap_preset = profile["rtabmap_preset"]
        results = run_candidates(
            methods,
            dataset,
            workspace,
            profile,
            colmap_preset=args.colmap_preset,
            colmap_use_gpu=args.colmap_use_gpu,
            rtabmap_preset=rtabmap_preset,
            progress=progress,
        )
        progress.start("agreement")
        agreement = evaluate_agreement(results, workspace / "diagnostics", args.allow_unreliable_best, dataset=dataset)
        progress.done("agreement")
        progress.start("persist_outputs")
        persist_outputs(workspace, output, results, agreement, args.persist_intermediates)
        progress.done("persist_outputs")
        if agreement.get("status") != "ok" and not args.allow_unreliable_best:
            print("[pseudo-gt] Agreement failed; no reliable best_pseudo_gt_tum.csv was written.", file=sys.stderr)
            return 3
        return 0
    except Exception:
        if progress.active_name is not None:
            progress.fail(progress.active_name)
        raise
    finally:
        cleanup_workspace(workspace, args.keep_workspace)


if __name__ == "__main__":
    sys.exit(main())
