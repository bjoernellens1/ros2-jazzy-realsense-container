from pathlib import Path

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


def test_agreement_fails_without_two_methods(tmp_path: Path) -> None:
    a = tmp_path / "a.txt"
    write_tum(a)
    results = [pg.CandidateResult("rtabmap_rgbd", "ok", a, None, {})]
    agreement = pg.evaluate_agreement(results, tmp_path / "diag", allow_unreliable=False)
    assert agreement["status"] == "agreement_failed"
    assert agreement["winner"] is None


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
    info = __import__("json").loads((dataset / "camera_info.json").read_text(encoding="utf-8"))
    assert info["fx"] == 517.3
    assert info["depth_factor"] == 5000.0
