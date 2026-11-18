"""bin_bounds + geometry, no ROS: projection = OpenCV's, depth back-projection inverts it, and the UW lab's bin
rectangles from the committed workcell + camera calibration stay where they were checked by eye (2026-11-17,
runs/2026-11-17_bin_bounds/bounds_overlay.jpg). Run: pytest perception/pod_perception/test (needs numpy, cv2, yaml)."""
import os
import sys

import cv2
import numpy as np
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from pod_perception.bin_bounds import bin_bounds, front_corners, matrix, project  # noqa: E402
from pod_perception.geometry import backproject, transform_points  # noqa: E402

REPO = os.path.join(os.path.dirname(__file__), "..", "..", "..")
# The Azure Kinect's factory intrinsics at 3072P (camera_info, 2026-11-16), rational_polynomial.
K = np.array([[1940.13671875, 0.0, 2048.73974609375], [0.0, 1940.19580078125, 1551.388916015625], [0, 0, 1]])
D = np.array([0.48402512073516846, -3.203151226043701, -0.00024434912484139204, -0.000539066269993782,
              2.0108675956726074, 0.351034015417099, -2.9763967990875244, 1.9035305976867676])
SIZE = (4096, 3072)


def test_projection_matches_opencv():
    pts = np.random.default_rng(0).uniform([-0.6, -0.5, 0.8], [0.6, 0.5, 2.0], (300, 3))
    ref, _ = cv2.projectPoints(pts, np.zeros(3), np.zeros(3), K, D)
    assert np.abs(project(pts, K, D) - ref.reshape(-1, 2)).max() < 1e-6


def test_point_behind_the_camera_is_nan():
    assert np.isnan(project(np.array([[0.1, 0.1, -1.0]]), K, D)).all()


def test_backproject_inverts_projection():
    pts = np.random.default_rng(1).uniform([-0.5, -0.4, 0.9], [0.5, 0.4, 1.8], (50, 3))
    uv = np.round(project(pts, K, D)).astype(int)
    depth = np.zeros((SIZE[1], SIZE[0]), np.float32)
    depth[uv[:, 1], uv[:, 0]] = pts[:, 2] * 1000
    back = backproject(depth, depth > 0, K, D)
    # pixel rounding: <= 0.5 px at >= 0.9 m and f ~ 1940 px is < 0.3 mm
    for p in pts:
        assert np.linalg.norm(back - p, axis=1).min() < 5e-4


def test_backproject_crop_offset():
    depth = np.zeros((10, 10), np.float32)
    depth[3, 4] = 1500
    p_crop = backproject(depth, depth > 0, K, D, offset=(2000, 1500))
    full = np.zeros((SIZE[1], SIZE[0]), np.float32)
    full[1503, 2004] = 1500
    assert np.allclose(p_crop, backproject(full, full > 0, K, D))


def test_front_corners_margins():
    spec = {"origin": [0.0, 0.0, 0.3], "columns": 4, "bin_width": 0.25, "row_heights": [0.2, 0.3]}
    c = front_corners(spec, "1B")
    assert np.allclose(c[:, 0].min(), 0.03) and np.allclose(c[:, 0].max(), 0.25)
    assert np.allclose(c[:, 2].min(), 0.52) and np.allclose(c[:, 2].max(), 0.8)
    assert np.allclose(front_corners(spec, "4A")[:, 0].max(), 0.97)


def test_transform_points():
    T = matrix([1, 2, 3], rpy=[0, 0, np.pi / 2])
    assert np.allclose(transform_points(T, [[1, 0, 0]]), [[1, 3, 3]])


def test_uw_lab_bins_where_checked_by_eye():
    wc = yaml.safe_load(open(os.path.join(REPO, "sites/uw_lab/workcell.yaml")))
    cal = yaml.safe_load(open(os.path.join(REPO, "sites/uw_lab/calibration/camera_lower_right.yaml")))
    pod = wc["pods"]["pod_bins"]
    f = wc["frames"][pod["frame"]]
    b = bin_bounds(pod, matrix(f["xyz"], rpy=f["rpy"]), matrix(cal["xyz"], quat_xyzw=cal["quat_xyzw"]), K, D, SIZE)
    assert sorted(b) == sorted(f"{c}{r}" for c in range(1, 5) for r in "CDEFGHIJK")
    assert b["2H"] == [1095, 1447, 1750, 2140] and b["2G"] == [1475, 1646, 1746, 2138]
