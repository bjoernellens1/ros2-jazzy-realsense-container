#!/usr/bin/env python3
"""
Self-contained RGB-D Visual Odometry from MCAP bag using Open3D.
Usage: _inside_pyslam_vo.py <camera.yaml> <input.mcap> <output_tum.csv>
"""
import sys, os, time, struct
import numpy as np
import yaml
import open3d as o3d
import open3d.core as o3c

from mcap.reader import make_reader

# ── Camera intrinsics from YAML ────────────────────────────────────────
def load_camera(yaml_path):
    with open(yaml_path) as f:
        d = yaml.safe_load(f)
    cam = {}
    for k in ("fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "k3",
              "width", "height", "fps", "bf", "ThDepth", "DepthMapFactor", "RGB"):
        if f"Camera.{k}" in d:
            cam[k] = d[f"Camera.{k}"]
    cam.setdefault("k1", 0); cam.setdefault("k2", 0); cam.setdefault("k3", 0)
    cam.setdefault("p1", 0); cam.setdefault("p2", 0)
    cam.setdefault("DepthMapFactor", 1000.0)
    cam["K"] = np.array([[cam["fx"], 0, cam["cx"]],
                          [0, cam["fy"], cam["cy"]],
                          [0, 0, 1]], dtype=np.float64)
    D = np.array([cam["k1"], cam["k2"], cam["p1"], cam["p2"], cam["k3"]], dtype=np.float64)
    cam["D"] = D
    cam["depth_factor"] = cam["DepthMapFactor"]
    return cam

# ── MCAP reader, yields (timestamp_ns, rgb_uint8_hwc, depth_m_float32_hw) ──
def read_mcap_rgbd(mcap_path, color_topic, depth_topic):
    """Generator yielding (ts_ns, rgb, depth_m) tuples from MCAP with approximate sync."""
    color_list = []
    depth_list = []

    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        for schema, channel, msg in reader.iter_messages():
            ts = msg.publish_time
            topic = channel.topic
            if topic == color_topic:
                color_list.append((ts, msg.data))
            elif topic == depth_topic:
                depth_list.append((ts, msg.data))

    # Approximate matching: for each color frame, find closest depth frame
    # within tolerance (typically depth arrives slightly before/after color)
    di = 0
    max_d = len(depth_list)
    TOLERANCE_NS = 50000000  # 50ms tolerance for matching

    for ci, (cts, color_data) in enumerate(color_list):
        # Advance depth pointer to nearest
        while di < max_d and depth_list[di][0] < cts - TOLERANCE_NS:
            di += 1
        if di >= max_d:
            break
        dts, depth_data = depth_list[di]
        if abs(dts - cts) > TOLERANCE_NS:
            continue

        rgb = _decode_image_cdr(color_data)
        if rgb is None:
            continue
        depth_mm = _decode_depth_cdr(depth_data)
        if depth_mm is None:
            continue

        yield cts, rgb, depth_mm.astype(np.float32) / 1000.0

# ── CDR decoder for ROS2 sensor_msgs/Image ──────────────────────────────
import struct as _struct

def _decode_image_cdr(data):
    """Decode ROS2 CDR-encoded sensor_msgs/Image. Returns np array (H,W,C) or None."""
    try:
        offset = 4  # CDR encapsulation header
        offset += 8  # header.stamp: int32 sec + uint32 nanosec
        # header.frame_id: string (uint32 length + chars, 4-byte aligned)
        frame_len = _struct.unpack_from('<I', data, offset)[0]; offset += 4 + frame_len
        offset = (offset + 3) & ~3
        height = _struct.unpack_from('<I', data, offset)[0]; offset += 4
        width = _struct.unpack_from('<I', data, offset)[0]; offset += 4
        # encoding: string (uint32 length + chars, 4-byte aligned)
        enc_len = _struct.unpack_from('<I', data, offset)[0]; offset += 4 + enc_len
        offset = (offset + 3) & ~3
        # is_bigendian (uint8, then pad to 4)
        offset += 1; offset = (offset + 3) & ~3
        # step (uint32) — in this bag, step = total bytes = width * height * ch
        total_bytes = _struct.unpack_from('<I', data, offset)[0]; offset += 4
        # data: unbounded uint8[], size = total_bytes
        img_data = data[offset:offset + total_bytes]
        # Infer channels from step/height/width
        row_bytes = total_bytes // height if height > 0 else 0
        ch = row_bytes // width if width > 0 else 3
        return np.frombuffer(img_data, dtype=np.uint8).reshape((height, width, ch))
    except Exception:
        return None

def _decode_depth_cdr(data):
    """Decode ROS2 CDR depth Image (16UC1). Returns np uint16 array (H,W) or None."""
    try:
        offset = 4  # CDR header
        offset += 8  # header.stamp
        frame_len = _struct.unpack_from('<I', data, offset)[0]; offset += 4 + frame_len
        offset = (offset + 3) & ~3
        height = _struct.unpack_from('<I', data, offset)[0]; offset += 4
        width = _struct.unpack_from('<I', data, offset)[0]; offset += 4
        enc_len = _struct.unpack_from('<I', data, offset)[0]; offset += 4 + enc_len
        offset = (offset + 3) & ~3
        offset += 1; offset = (offset + 3) & ~3  # is_bigendian + align
        total_bytes = _struct.unpack_from('<I', data, offset)[0]; offset += 4
        # unbounded uint8[] — size = total_bytes (stored as total, not per-row)
        img_data = data[offset:offset + total_bytes]
        return np.frombuffer(img_data, dtype=np.uint16).reshape((height, width))
    except Exception:
        return None

# ── TUM trajectory writer ───────────────────────────────────────────────
def write_tum_line(f, ts_ns, t, R):
    """Write one TUM-format line: timestamp x y z qx qy qz qw"""
    from scipy.spatial.transform import Rotation
    q = Rotation.from_matrix(R).as_quat()  # xyzw
    f.write(f"{ts_ns/1e9:.9f} {t[0]:.6f} {t[1]:.6f} {t[2]:.6f} {q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}\n")

# ── Main ────────────────────────────────────────────────────────────────
def main():
    if len(sys.argv) < 4:
        print(f"Usage: {sys.argv[0]} <camera.yaml> <input.mcap> <output_tum.csv>")
        sys.exit(1)

    cam_path, mcap_path, out_path = sys.argv[1], sys.argv[2], sys.argv[3]
    cam = load_camera(cam_path)

    print(f"Camera: {cam['width']}x{cam['height']} fx={cam['fx']:.2f} fy={cam['fy']:.2f}")
    print(f"MCAP: {mcap_path}")
    print(f"Output: {out_path}")

    # Open3D intrinsics
    intrinsics = o3d.core.Tensor(cam["K"], o3d.core.Dtype.Float64)

    # Odometry method
    device = o3c.Device("CUDA:0" if o3d.core.cuda.is_available() else "CPU:0")
    method = o3d.t.pipelines.odometry.Method.Hybrid
    criteria = [o3d.t.pipelines.odometry.OdometryConvergenceCriteria(max_iteration=30)]

    max_depth = cam.get("ThDepth", 10.0)
    depth_scale = 1.0

    traj = []  # list of (ts_ns, T_4x4)
    prev_rgbd = None
    img_id = 0
    t_start = time.time()
    last_print = t_start

    for ts, rgb, depth_m in read_mcap_rgbd(mcap_path,
            "/camera/camera/color/image_raw",
            "/camera/camera/aligned_depth_to_color/image_raw"):
        if img_id % 100 == 0:
            now = time.time()
            fps = img_id / (now - t_start) if (now - t_start) > 0 else 0
            print(f"Frame {img_id} ({fps:.1f} fps)")
            last_print = now

        # Filter depth
        depth_m[depth_m <= 0] = 0
        depth_m[depth_m > max_depth] = 0

        # Open3D tensor RGBD image (depth in meters, RGB in uint8)
        try:
            rgb_o3d = o3d.t.geometry.Image(rgb.astype(np.uint8)).to(device)
            depth_o3d = o3d.t.geometry.Image(depth_m.astype(np.float32)).to(device)
            cur_rgbd = o3d.t.geometry.RGBDImage(rgb_o3d, depth_o3d)
        except Exception:
            continue

        if prev_rgbd is None:
            prev_rgbd = cur_rgbd
            img_id += 1
            # First pose = identity
            traj.append((ts, np.eye(4)))
            continue

        result = o3d.t.pipelines.odometry.rgbd_odometry_multi_scale(
            prev_rgbd, cur_rgbd, intrinsics, np.eye(4), depth_scale, max_depth, criteria, method)

        if result.transformation is not None and result.fitness > 0:
            T_rel = result.transformation.cpu().numpy()
            T_prev = traj[-1][1]
            T_cur = T_prev @ T_rel
        else:
            # Tracking lost — keep previous pose
            T_cur = traj[-1][1] if traj else np.eye(4)

        traj.append((ts, T_cur))
        prev_rgbd = cur_rgbd
        img_id += 1

    # Write TUM file
    with open(out_path, "w") as f:
        f.write("# timestamp x y z qx qy qz qw\n")
        for ts, T in traj:
            write_tum_line(f, ts, T[:3, 3], T[:3, :3])

    elapsed = time.time() - t_start
    print(f"Done. {len(traj)} frames in {elapsed:.1f}s ({len(traj)/elapsed:.1f} fps)")
    print(f"Trajectory saved to {out_path}")

if __name__ == "__main__":
    main()
