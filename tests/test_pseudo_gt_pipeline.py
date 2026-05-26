from pathlib import Path
from types import SimpleNamespace

import numpy as np

import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

import pseudo_gt_pipeline as pg


def write_tum(path: Path, offset=(0.0, 0.0, 0.0), scale=1.0, n=40) -> None:
    with path.open("w", encoding="utf-8") as fh:
        for i in range(n):
            ts = i * 0.1
            x = scale * i * 0.02 + offset[0]
            y = offset[1]
            z = offset[2]
            fh.write(f"{ts:.9f} {x:.9f} {y:.9f} {z:.9f} 0 0 0 1\n")


def write_tum_with_yaw(path: Path, yaw_per_frame_deg: float, n=40) -> None:
    with path.open("w", encoding="utf-8") as fh:
        for i in range(n):
            ts = i * 0.1
            x = i * 0.02
            yaw = np.deg2rad(i * yaw_per_frame_deg)
            qz = np.sin(yaw / 2.0)
            qw = np.cos(yaw / 2.0)
            fh.write(f"{ts:.9f} {x:.9f} 0.0 0.0 0 0 {qz:.9f} {qw:.9f}\n")


def write_tum_with_y_noise(path: Path, amplitude: float, n=40) -> None:
    with path.open("w", encoding="utf-8") as fh:
        for i in range(n):
            ts = i * 0.1
            x = i * 0.02
            y = amplitude * np.sin(i * 0.4)
            fh.write(f"{ts:.9f} {x:.9f} {y:.9f} 0.0 0 0 0 1\n")


def test_umeyama_recovers_scaled_translation() -> None:
    src = np.array([[0, 0, 0], [1, 0, 0], [2, 1, 0], [3, 1, 1]], dtype=float)
    dst = src * 2.0 + np.array([1.0, -2.0, 0.5])
    scale, rot, trans = pg.umeyama_sim3(src, dst)
    aligned = pg.apply_sim3(src, scale, rot, trans)
    assert np.allclose(aligned, dst)


def test_agreement_selects_supported_candidate(tmp_path: Path) -> None:
    a = tmp_path / "a.txt"
    b = tmp_path / "b.txt"
    c = tmp_path / "c.txt"
    write_tum(a)
    write_tum(b, offset=(0.05, 0.0, 0.0))
    write_tum(c, offset=(10.0, 0.0, 0.0), n=10)

    results = [
        pg.CandidateResult("rtabmap_rgbd", "ok", a, None, {}),
        pg.CandidateResult("colmap_sfm", "ok", b, None, {}),
        pg.CandidateResult("orbslam3_rgbd", "ok", c, None, {}),
    ]
    agreement = pg.evaluate_agreement(results, tmp_path / "diag", allow_unreliable=False)
    assert agreement["status"] == "ok"
    assert agreement["winner"] in {"rtabmap_rgbd", "colmap_sfm"}
    assert agreement["support"][agreement["winner"]] == 1


def test_groundtruth_quality_breaks_supported_candidate_ties(tmp_path: Path) -> None:
    dataset = tmp_path / "dataset"
    dataset.mkdir()
    gt = dataset / "groundtruth.txt"
    colmap = tmp_path / "colmap.txt"
    orb = tmp_path / "orb.txt"
    write_tum(gt)
    write_tum_with_y_noise(colmap, amplitude=0.04)
    write_tum_with_y_noise(orb, amplitude=0.01)

    results = [
        pg.CandidateResult("colmap_sfm", "ok", colmap, None, {}),
        pg.CandidateResult("orbslam3_rgbd", "ok", orb, None, {}),
    ]
    agreement = pg.evaluate_agreement(results, tmp_path / "diag", allow_unreliable=False, dataset=dataset)
    assert agreement["status"] == "ok"
    assert agreement["support"] == {"colmap_sfm": 1, "orbslam3_rgbd": 1}
    assert agreement["winner"] == "orbslam3_rgbd"
    gt_rmse = {row["method"]: row["rmse"] for row in agreement["gt_comparisons"]}
    assert gt_rmse["orbslam3_rgbd"] < gt_rmse["colmap_sfm"]


def test_agreement_fails_without_two_methods(tmp_path: Path) -> None:
    a = tmp_path / "a.txt"
    write_tum(a)
    results = [pg.CandidateResult("rtabmap_rgbd", "ok", a, None, {})]
    agreement = pg.evaluate_agreement(results, tmp_path / "diag", allow_unreliable=False)
    assert agreement["status"] == "agreement_failed"
    assert agreement["winner"] is None


def test_yaw_drift_is_diagnostic_not_hard_gate(tmp_path: Path) -> None:
    a = tmp_path / "a.txt"
    b = tmp_path / "b.txt"
    write_tum_with_yaw(a, yaw_per_frame_deg=0.0)
    write_tum_with_yaw(b, yaw_per_frame_deg=1.0)
    results = [
        pg.CandidateResult("rtabmap_rgbd", "ok", a, None, {}),
        pg.CandidateResult("rtabmap_rgbd_imu", "ok", b, None, {}),
    ]
    agreement = pg.evaluate_agreement(results, tmp_path / "diag", allow_unreliable=False)
    assert agreement["status"] == "ok"
    assert agreement["pairwise"][0]["yaw_drift_deg"] > 5.0
    assert agreement["policy"]["yaw_drift"] == "diagnostic_only"


def test_colmap_binary_model_is_converted_to_text(tmp_path: Path, monkeypatch) -> None:
    sparse = tmp_path / "sparse"
    model = sparse / "0"
    model.mkdir(parents=True)
    (model / "images.bin").write_bytes(b"fake")

    def fake_run(cmd, log=None, env=None, cwd=None):
        assert cmd[1] == "model_converter"
        dest = Path(cmd[cmd.index("--output_path") + 1])
        dest.mkdir(parents=True, exist_ok=True)
        (dest / "images.txt").write_text("# converted\n", encoding="utf-8")
        return 0

    monkeypatch.setattr(pg, "run", fake_run)
    text_root = pg.convert_colmap_models_to_text(sparse, tmp_path / "sparse_txt", tmp_path / "run.log")
    assert (text_root / "0" / "images.txt").exists()


def test_tum_rgbd_sequence_normalizes_to_common_layout(tmp_path: Path) -> None:
    cv2 = __import__("pytest").importorskip("cv2")

    root = tmp_path / "rgbd_dataset_freiburg1_xyz"
    (root / "rgb").mkdir(parents=True)
    (root / "depth").mkdir()
    with (root / "rgb.txt").open("w", encoding="utf-8") as f_rgb, (root / "depth.txt").open(
        "w", encoding="utf-8"
    ) as f_depth:
        for i in range(3):
            ts = 1305031102.0 + i * 0.1
            rgb_rel = f"rgb/{i:06d}.png"
            depth_rel = f"depth/{i:06d}.png"
            image = np.full((48, 64, 3), 64 + i, dtype=np.uint8)
            depth = np.full((48, 64), 1000 + i, dtype=np.uint16)
            cv2.imwrite(str(root / rgb_rel), image)
            cv2.imwrite(str(root / depth_rel), depth)
            f_rgb.write(f"{ts:.6f} {rgb_rel}\n")
            f_depth.write(f"{ts:.6f} {depth_rel}\n")

    dataset = tmp_path / "normalized"
    result = pg.normalize_tum_rgbd(root, dataset, {"depth_factor": 5000.0}, target_fps=0, max_frames=2)
    assert result["frame_count"] == 2
    assert (dataset / "images" / "frame_000000.png").exists()
    assert (dataset / "depth" / "frame_000000.png").exists()
    assert (dataset / "associations.txt").read_text(encoding="utf-8").count("\n") == 2
    # depth_factor is always normalised to 1000 (mm) after normalization
    assert result["depth_factor"] == 1000.0
    info = __import__("json").loads((dataset / "camera_info.json").read_text(encoding="utf-8"))
    assert abs(info["fx"] - 517.306408) < 1e-6
    assert info["depth_factor"] == 1000.0
    # rtabmap sync dirs must exist with timestamp-named symlinks
    assert (dataset / "calibration.yaml").exists()
    assert (dataset / "rgb_sync").is_dir()
    assert (dataset / "depth_sync").is_dir()
    assert any((dataset / "rgb_sync").iterdir())
    # depth values must be rescaled from 5000 to 1000 units/m
    depth_img = cv2.imread(str(dataset / "depth" / "frame_000000.png"), cv2.IMREAD_UNCHANGED)
    assert depth_img is not None
    # original value was 1000 at 5000 units/m == 0.2 m == 200 mm
    assert abs(int(depth_img.flat[0]) - 200) < 5


def test_compressed_image_decode_supports_rgb_and_depth() -> None:
    cv2 = __import__("pytest").importorskip("cv2")

    color = np.full((8, 10, 3), (10, 20, 30), dtype=np.uint8)
    ok, encoded_color = cv2.imencode(".png", color)
    assert ok
    decoded_color = pg.image_to_array(SimpleNamespace(format="png", data=encoded_color.tobytes()), is_depth=False)
    assert decoded_color.shape == color.shape

    depth = np.full((8, 10), 1234, dtype=np.uint16)
    ok, encoded_depth = cv2.imencode(".png", depth)
    assert ok
    decoded_depth = pg.image_to_array(SimpleNamespace(format="16UC1; compressed png", data=encoded_depth.tobytes()), is_depth=True)
    assert decoded_depth.dtype == np.uint16
    assert int(decoded_depth[0, 0]) == 1234


def test_stream_association_is_one_to_one_and_reports_sync() -> None:
    left = [(0, "a"), (10, "b"), (20, "c")]
    right = [(1, "d0"), (11, "d1")]
    assignments = pg.associate_streams_by_stamp(left, right, max_delta_ns=3)
    assert assignments == [(0, 0), (1, 1)]

    report = pg.build_sync_report(
        "/rgb",
        "/depth",
        "/info",
        raw_color_count=len(left),
        raw_depth_count=len(right),
        raw_info_count=1,
        assignments=assignments,
        colors=left,
        depths=right,
        max_delta_ns=3,
        profile={"min_association_ratio": 0.5},
    )
    assert report["status"] == "ok"
    assert report["association_ratio"] == 2 / 3
    assert report["max_abs_dt_sec"] == 1e-09


def _write_camera_info(dataset: Path) -> None:
    import json

    dataset.mkdir(parents=True, exist_ok=True)
    (dataset / "camera_info.json").write_text(
        json.dumps(
            {
                "fx": 600.0,
                "fy": 600.0,
                "cx": 320.0,
                "cy": 240.0,
                "width": 640,
                "height": 480,
                "d": [0.0, 0.0, 0.0, 0.0, 0.0],
                "depth_factor": 1000.0,
                "stereo_b": 0.05,
            }
        ),
        encoding="utf-8",
    )


def _write_assoc_30hz(dataset: Path, n: int = 10) -> None:
    with (dataset / "associations.txt").open("w", encoding="utf-8") as fh:
        for i in range(n):
            ts = i / 30.0
            fh.write(f"{ts:.9f} images/{i:06d}.png {ts:.9f} depth/{i:06d}.png\n")


def test_write_imu_csv_sorts_and_formats_rows(tmp_path: Path) -> None:
    rows = [
        (2_000_000_000, 0.0, 0.1, 9.8, 0.01, 0.02, 0.03),
        (1_000_000_000, -1.0, 0.0, 9.81, 0.0, 0.0, 0.0),
    ]
    summary = pg.write_imu_csv(rows, tmp_path)
    text = (tmp_path / "imu.csv").read_text(encoding="utf-8").splitlines()
    assert text[0] == "timestamp,ax,ay,az,gx,gy,gz"
    assert text[1].startswith("1.000000000,")
    assert text[2].startswith("2.000000000,")
    assert summary["imu_count"] == 2
    assert summary["imu_first_timestamp"] == 1.0
    assert summary["imu_last_timestamp"] == 2.0


def test_orbslam3_rgbd_imu_skipped_when_imu_csv_missing(tmp_path: Path) -> None:
    dataset = tmp_path / "dataset"
    _write_camera_info(dataset)
    out_dir = tmp_path / "out"
    result = pg.run_orbslam3_candidate(dataset, out_dir, profile={}, method="orbslam3_rgbd_imu")
    assert result.status == "failed"
    assert "imu.csv" in (result.reason or "")


def test_orbslam3_rgbd_imu_refuses_default_extrinsics_without_profile(tmp_path: Path) -> None:
    import pytest

    dataset = tmp_path / "dataset"
    _write_camera_info(dataset)
    _write_assoc_30hz(dataset)
    with pytest.raises(ValueError, match="orbslam3_rgbd_imu"):
        pg.write_orbslam3_settings(dataset, tmp_path, profile={}, with_imu=True)


def test_orbslam3_rgbd_imu_rejects_malformed_t_b_c1(tmp_path: Path) -> None:
    import pytest

    dataset = tmp_path / "dataset"
    _write_camera_info(dataset)
    _write_assoc_30hz(dataset)
    profile = {"orbslam3_rgbd_imu": {"T_b_c1": [1.0, 0.0, 0.0]}}
    with pytest.raises(ValueError, match="16-element"):
        pg.write_orbslam3_settings(dataset, tmp_path, profile=profile, with_imu=True)


def test_orbslam3_settings_emits_imu_block_when_requested(tmp_path: Path) -> None:
    dataset = tmp_path / "dataset"
    _write_camera_info(dataset)
    _write_assoc_30hz(dataset)
    profile = {
        "orbslam3_rgbd_imu": {
            "T_b_c1": [1.0, 0.0, 0.0, 0.0,
                       0.0, 1.0, 0.0, 0.0,
                       0.0, 0.0, 1.0, 0.0,
                       0.0, 0.0, 0.0, 1.0],
            "noise_gyro": 0.02,
            "noise_acc": 0.2,
            "gyro_walk": 2e-6,
            "acc_walk": 2e-4,
            "frequency": 250.0,
        },
    }
    out_with = tmp_path / "with"
    out_with.mkdir()
    settings_with = pg.write_orbslam3_settings(dataset, out_with, profile=profile, with_imu=True)
    text_with = settings_with.read_text(encoding="utf-8")
    assert "IMU.T_b_c1" in text_with
    assert "IMU.NoiseGyro: 0.02" in text_with
    assert "IMU.Frequency: 250.0" in text_with
    assert "Camera.fps: 30" in text_with

    out_without = tmp_path / "without"
    out_without.mkdir()
    settings_without = pg.write_orbslam3_settings(dataset, out_without, profile=profile, with_imu=False)
    text_without = settings_without.read_text(encoding="utf-8")
    assert "IMU." not in text_without


def test_estimate_camera_fps_handles_non_30hz_dataset(tmp_path: Path) -> None:
    dataset = tmp_path / "dataset"
    dataset.mkdir()
    with (dataset / "associations.txt").open("w", encoding="utf-8") as fh:
        for i in range(10):
            ts = i * 0.1  # 10 Hz
            fh.write(f"{ts:.9f} images/{i}.png {ts:.9f} depth/{i}.png\n")
    assert abs(pg.estimate_camera_fps(dataset) - 10.0) < 1e-6


def test_rtabmap_dense_odom_export_is_selected(tmp_path: Path, monkeypatch) -> None:
    dataset = tmp_path / "dataset"
    dataset.mkdir()
    _write_assoc_30hz(dataset, n=40)
    write_tum(dataset / "groundtruth.txt", n=40)
    out = tmp_path / "rtabmap"
    out.mkdir()
    (out / "rtabmap.db").write_bytes(b"sqlite")
    write_tum(out / "rtabmap_poses.txt", n=5)

    def fake_run(cmd, log=None, env=None, cwd=None, progress=None, progress_label=None):
        assert "--poses_raw" in cmd
        assert "--gt" in cmd
        write_tum(out / "rtabmap_odom.txt", n=40)
        write_tum(out / "rtabmap_slam.txt", n=20)
        write_tum(out / "rtabmap_gt.txt", n=40)
        return 0

    monkeypatch.setattr(pg.shutil, "which", lambda name: "/usr/bin/rtabmap-report" if name == "rtabmap-report" else None)
    monkeypatch.setattr(pg, "run", fake_run)

    dense, metrics, reason = pg.export_rtabmap_dense_odom(dataset, out, out / "run.log")
    assert reason == ""
    assert dense == out / "rtabmap_odom.txt"
    assert metrics["dense_odom_pose_count"] == 40
    assert metrics["dense_odom_coverage_ratio"] == 1.0


def test_rtabmap_sparse_pose_file_is_not_accepted_as_dense_export(tmp_path: Path, monkeypatch) -> None:
    dataset = tmp_path / "dataset"
    dataset.mkdir()
    _write_assoc_30hz(dataset, n=40)
    out = tmp_path / "rtabmap"
    out.mkdir()
    (out / "rtabmap.db").write_bytes(b"sqlite")
    write_tum(out / "rtabmap_poses.txt", n=5)

    def fake_run(cmd, log=None, env=None, cwd=None, progress=None, progress_label=None):
        return 0

    monkeypatch.setattr(pg.shutil, "which", lambda name: "/usr/bin/rtabmap-report" if name == "rtabmap-report" else None)
    monkeypatch.setattr(pg, "run", fake_run)

    dense, metrics, reason = pg.export_rtabmap_dense_odom(dataset, out, out / "run.log")
    assert dense is None
    assert reason == "dense odometry export was not written"
    assert "dense_odom_pose_count" not in metrics


def test_rtabmap_database_node_poses_are_dense_fallback(tmp_path: Path, monkeypatch) -> None:
    import sqlite3
    import struct

    dataset = tmp_path / "dataset"
    dataset.mkdir()
    _write_assoc_30hz(dataset, n=40)
    out = tmp_path / "rtabmap"
    out.mkdir()
    db = out / "rtabmap.db"
    con = sqlite3.connect(db)
    con.execute("create table Node (id integer primary key, stamp float, pose blob)")
    for i in range(40):
        pose = struct.pack(
            "12f",
            1.0, 0.0, 0.0, i * 0.02,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
        )
        con.execute("insert into Node(id, stamp, pose) values (?, ?, ?)", (i + 1, i / 30.0, pose))
    con.commit()
    con.close()

    def fake_run(cmd, log=None, env=None, cwd=None, progress=None, progress_label=None):
        write_tum(out / "rtabmap_odom.txt", n=5)
        return 0

    monkeypatch.setattr(pg.shutil, "which", lambda name: "/usr/bin/rtabmap-report" if name == "rtabmap-report" else None)
    monkeypatch.setattr(pg, "run", fake_run)

    dense, metrics, reason = pg.export_rtabmap_dense_odom(dataset, out, out / "run.log")
    assert reason == ""
    assert dense == out / "rtabmap_node_odom.txt"
    assert metrics["dense_odom_source"] == "database_node_pose"
    assert metrics["dense_odom_pose_count"] == 40
    assert pg.read_tum(dense)["p"].shape[0] == 40


def test_rtabmap_preset_args() -> None:
    assert pg.rtabmap_preset_args("default", {"rtabmap_vis_min_inliers": 9, "rtabmap_kp_max_features": 700}) == [
        "--Vis/MinInliers", "9",
        "--Kp/MaxFeatures", "700",
        "--Rtabmap/DetectionRate", "0",
    ]
    robust = pg.rtabmap_preset_args("robust", {})
    assert "--Vis/MaxFeatures" in robust
    assert robust[robust.index("--Odom/Strategy") + 1] == "0"
    f2f = pg.rtabmap_preset_args("f2f", {})
    assert f2f[f2f.index("--Odom/Strategy") + 1] == "1"
    dense = pg.rtabmap_preset_args("dense-keyframes", {})
    assert dense[dense.index("--Odom/KeyFrameThr") + 1] == "0"
    assert dense[dense.index("--Odom/VisKeyFrameThr") + 1] == "0"


def test_pairwise_agreement_csv_contains_yaw_drift_column(tmp_path: Path) -> None:
    a = tmp_path / "a.txt"
    b = tmp_path / "b.txt"
    write_tum_with_yaw(a, yaw_per_frame_deg=0.0)
    write_tum_with_yaw(b, yaw_per_frame_deg=1.0)
    results = [
        pg.CandidateResult("rtabmap_rgbd", "ok", a, None, {}),
        pg.CandidateResult("rtabmap_rgbd_imu", "ok", b, None, {}),
    ]
    diag = tmp_path / "diag"
    pg.evaluate_agreement(results, diag, allow_unreliable=False)
    csv_text = (diag / "pairwise_agreement.csv").read_text(encoding="utf-8").splitlines()
    header = csv_text[0].split(",")
    assert "yaw_drift_deg" in header
    yaw_idx = header.index("yaw_drift_deg")
    row = csv_text[1].split(",")
    assert float(row[yaw_idx]) > 5.0


def _write_degenerate_tum(path: Path, n: int = 40) -> None:
    """Write a TUM file where all poses are at the origin (all-identity)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as fh:
        for i in range(n):
            ts = i * 0.1
            fh.write(f"{ts:.9f} 0.000000000 0.000000000 0.000000000 0 0 0 1\n")


def test_degenerate_trajectory_rejected_by_health(tmp_path: Path) -> None:
    good = tmp_path / "good.txt"
    bad = tmp_path / "bad.txt"
    write_tum(good)
    _write_degenerate_tum(bad)

    results = [
        pg.CandidateResult("colmap_sfm", "ok", good, None, {}),
        pg.CandidateResult("rtabmap_rgbd", "ok", bad, None, {}),
    ]
    diag = tmp_path / "diag"
    agreement = pg.evaluate_agreement(results, diag, allow_unreliable=False)
    assert agreement["health"]["rtabmap_rgbd"]["status"] == "unhealthy"
    assert agreement["health"]["rtabmap_rgbd"]["reason"] == "degenerate_trajectory"
    assert agreement["winner"] is None or agreement["winner"] == "colmap_sfm"


def test_collapsed_sim3_scale_does_not_agree(tmp_path: Path) -> None:
    good = tmp_path / "good.txt"
    bad = tmp_path / "bad.txt"
    write_tum(good)
    _write_degenerate_tum(bad, n=50)

    # Manually call evaluate_pair to confirm scale gate fires
    traj_good = pg.read_tum(good)
    traj_bad = pg.read_tum(bad)
    result = pg.evaluate_pair("good", traj_good, "bad", traj_bad, run_duration=4.0)
    assert result["agree"] is False, "collapsed Sim3 (scale≈0) must not agree"


def test_zip_is_rosbag2_returns_false_for_plain_zip(tmp_path: Path) -> None:
    import zipfile
    z = tmp_path / "not_a_bag.zip"
    with zipfile.ZipFile(z, "w") as zf:
        zf.writestr("scene/camera.hdf5", b"fake")
    assert pg._zip_is_rosbag2(z) is False


def test_zip_is_rosbag2_returns_true_for_rosbag2_zip(tmp_path: Path) -> None:
    import zipfile
    z = tmp_path / "bag.zip"
    with zipfile.ZipFile(z, "w") as zf:
        zf.writestr("metadata.yaml", "rosbag2_bagfile_information:\n")
        zf.writestr("bag_0.mcap", b"fake_mcap")
    assert pg._zip_is_rosbag2(z) is True


def test_hypersim_orbslam3_settings_use_more_features(tmp_path: Path) -> None:
    dataset = tmp_path / "dataset"
    _write_camera_info(dataset)
    _write_assoc_30hz(dataset)
    out = tmp_path / "out"
    out.mkdir()
    settings = pg.write_orbslam3_settings(dataset, out, profile={"name": "hypersim"}, with_imu=False)
    text = settings.read_text(encoding="utf-8")
    assert "ORBextractor.nFeatures: 2000" in text
    assert "ORBextractor.iniThFAST: 12" in text


def test_default_orbslam3_settings_unchanged_for_realsense(tmp_path: Path) -> None:
    dataset = tmp_path / "dataset"
    _write_camera_info(dataset)
    _write_assoc_30hz(dataset)
    out = tmp_path / "out"
    out.mkdir()
    settings = pg.write_orbslam3_settings(dataset, out, profile={"name": "realsense_d435i_ros1"}, with_imu=False)
    text = settings.read_text(encoding="utf-8")
    assert "ORBextractor.nFeatures: 1000" in text
    assert "ORBextractor.iniThFAST: 20" in text


def test_rtabmap_sync_dirs_created_by_normalization(tmp_path: Path) -> None:
    """normalize_tum_rgbd creates rgb_sync/ and depth_sync/ with timestamp-named symlinks."""
    cv2 = __import__("pytest").importorskip("cv2")

    root = tmp_path / "rgbd_dataset_freiburg3_long"
    (root / "rgb").mkdir(parents=True)
    (root / "depth").mkdir()
    with (root / "rgb.txt").open("w", encoding="utf-8") as f_rgb, (root / "depth.txt").open(
        "w", encoding="utf-8"
    ) as f_depth:
        for i in range(3):
            ts = 1305031452.791720 + i * 0.1
            rgb_rel = f"rgb/{i:06d}.png"
            depth_rel = f"depth/{i:06d}.png"
            cv2.imwrite(str(root / rgb_rel), np.full((48, 64, 3), 128, dtype=np.uint8))
            cv2.imwrite(str(root / depth_rel), np.full((48, 64), 2000, dtype=np.uint16))
            f_rgb.write(f"{ts:.6f} {rgb_rel}\n")
            f_depth.write(f"{ts:.6f} {depth_rel}\n")

    dataset = tmp_path / "normalized"
    pg.normalize_tum_rgbd(root, dataset, {}, target_fps=0, max_frames=0)
    assert (dataset / "rgb_sync").is_dir()
    assert (dataset / "depth_sync").is_dir()
    assert (dataset / "calibration.yaml").exists()
    sync_files = sorted((dataset / "rgb_sync").iterdir())
    assert len(sync_files) == 3
    # filenames must be parseable floats (timestamps)
    for f in sync_files:
        float(f.stem)
    # symlinks must resolve to existing images
    for f in sync_files:
        assert f.is_symlink() and f.resolve().exists()


def _write_short_tum(path: Path, n: int = 10) -> None:
    """Write a short trajectory (n frames at 0.1 s spacing, small displacement)."""
    with path.open("w", encoding="utf-8") as fh:
        for i in range(n):
            ts = i * 0.1
            x = i * 0.02
            fh.write(f"{ts:.9f} {x:.9f} 0.0 0.0 0 0 0 1\n")


def test_short_coverage_method_excluded_from_winner(tmp_path: Path) -> None:
    """A method that covers <25% of the max-duration method cannot win even if it
    agrees with another short-coverage method."""
    long_a = tmp_path / "long_a.txt"
    long_b = tmp_path / "long_b.txt"
    short_c = tmp_path / "short_c.txt"
    short_d = tmp_path / "short_d.txt"
    # long_a and long_b: 40 frames × 0.1 s = 4 s, offset timestamps
    write_tum(long_a, n=40)
    with long_b.open("w") as fh:
        for i in range(40):
            ts = i * 0.1 + 0.001  # slight offset so they associate
            x = i * 0.02 + 0.001
            fh.write(f"{ts:.9f} {x:.9f} 0.0 0.0 0 0 0 1\n")
    # short_c and short_d: 10 frames × 0.1 s = 1 s, disjoint timestamps (far future)
    with short_c.open("w") as fh:
        for i in range(10):
            ts = 1000.0 + i * 0.1
            fh.write(f"{ts:.9f} {float(i)*0.02:.9f} 0.0 0.0 0 0 0 1\n")
    with short_d.open("w") as fh:
        for i in range(10):
            ts = 1000.0 + i * 0.1 + 0.001
            fh.write(f"{ts:.9f} {float(i)*0.02+0.001:.9f} 0.0 0.0 0 0 0 1\n")

    results = [
        pg.CandidateResult("colmap_sfm", "ok", long_a, None, {}),
        pg.CandidateResult("orbslam3_rgbd", "ok", long_b, None, {}),
        pg.CandidateResult("rtabmap_rgbd", "ok", short_c, None, {}),
        pg.CandidateResult("rtabmap_rgbd_imu", "ok", short_d, None, {}),
    ]
    diag = tmp_path / "diag"
    agreement = pg.evaluate_agreement(results, diag, allow_unreliable=False)
    # short methods have duration ~1 s vs max ~4 s (25% boundary — just below)
    # winner must come from the long-coverage group
    assert agreement["winner"] in {"colmap_sfm", "orbslam3_rgbd", None}
