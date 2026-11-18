"""Pixel rectangles of the pod's bins in the colour image, from the workcell's pod model and the camera calibration.

Replaces the hand-run ROS 1 script (aurmr_perception/scripts/bin_bound_calculation), which wrote a bin_bounds.yaml that
went stale whenever the camera or the pod moved. Same geometry as that script: the bin's front opening, 3 cm in from
the pod's outer uprights (first and last column) and 2 cm up from the bin floor (the flap), as [row_start, row_end,
col_start, col_end] in pixels. Unlike the script, points are projected through the camera's lens distortion.
No ROS: callers pass the transforms and intrinsics.
"""
import numpy as np

SIDE_MARGIN = 0.03     # m, pod's outer uprights (old script: 3 cm on columns 1 and 4)
FLOOR_MARGIN = 0.02    # m, the bin's front flap (old script: 2 cm)
ROWS = "abcdefghijklmnopqrstuvwxyz"


def bin_ids(spec):
    """All bins of a pod spec, e.g. ["1A", "2A", ..., "4L"]."""
    return [f"{c}{ROWS[r].upper()}" for r in range(len(spec["row_heights"])) for c in range(1, spec["columns"] + 1)]


def front_corners(spec, bin_id):
    """The four front corners of a bin's opening (with the margins) in the pod frame, 4 x 3."""
    col, row = int(bin_id[:-1]), ROWS.index(bin_id[-1].lower())
    if not 1 <= col <= spec["columns"] or row >= len(spec["row_heights"]):
        raise KeyError(f"no bin {bin_id}")
    x0, y0, z0 = spec["origin"]
    x1 = x0 + (col - 1) * spec["bin_width"]
    x2 = x1 + spec["bin_width"]
    if col == 1:
        x1 += SIDE_MARGIN
    if col == spec["columns"]:
        x2 -= SIDE_MARGIN
    z1 = z0 + sum(spec["row_heights"][:row]) + FLOOR_MARGIN
    z2 = z0 + sum(spec["row_heights"][:row + 1])
    return np.array([[x1, y0, z1], [x2, y0, z1], [x1, y0, z2], [x2, y0, z2]])


def project(points_cam, K, dist=None):
    """Camera-frame points (N x 3; z forward, x right, y down) to pixels (N x 2), with OpenCV's distortion model
    (k1 k2 p1 p2 k3 k4 k5 k6, ROS 'rational_polynomial' / 'plumb_bob' order). Points behind the camera -> NaN."""
    p = np.asarray(points_cam, float)
    d = np.zeros(8) if dist is None else np.r_[np.asarray(dist, float), np.zeros(8)][:8]
    x, y = p[:, 0] / p[:, 2], p[:, 1] / p[:, 2]
    r2 = x * x + y * y
    k1, k2, p1, p2, k3, k4, k5, k6 = d
    radial = (1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3) / (1 + k4 * r2 + k5 * r2 ** 2 + k6 * r2 ** 3)
    xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
    yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    uv = np.stack([K[0][0] * xd + K[0][2], K[1][1] * yd + K[1][2]], axis=1)
    uv[p[:, 2] <= 0] = np.nan
    return uv


def bin_bounds(spec, T_base_pod, T_base_cam, K, dist, image_size, bins=None):
    """{bin_id: [row_start, row_end, col_start, col_end]} for bins whose opening lies wholly in the image.
    T_base_pod: 4x4 base_link -> pod frame; T_base_cam: 4x4 base_link -> camera optical frame (rgb_camera_link);
    K: 3x3; dist: distortion coefficients; image_size: (width, height)."""
    w, h = image_size
    T_cam_pod = np.linalg.inv(T_base_cam) @ T_base_pod
    out = {}
    for b in bins or bin_ids(spec):
        c = front_corners(spec, b)
        pc = (T_cam_pod @ np.c_[c, np.ones(4)].T).T[:, :3]
        uv = project(pc, K, dist)
        if np.isnan(uv).any():
            continue
        u1, v1 = np.floor(uv.min(axis=0)).astype(int)
        u2, v2 = np.ceil(uv.max(axis=0)).astype(int)
        if u1 < 0 or v1 < 0 or u2 > w or v2 > h:
            continue
        out[b] = [int(v1), int(v2), int(u1), int(u2)]
    return out


def matrix(xyz, quat_xyzw=None, rpy=None):
    """4x4 from a translation and a quaternion (x, y, z, w) or roll-pitch-yaw."""
    T = np.eye(4)
    T[:3, 3] = xyz
    if quat_xyzw is not None:
        x, y, z, w = quat_xyzw
        T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    elif rpy is not None:
        r, p, yw = rpy
        cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(yw), np.sin(yw)
        T[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]]
    return T
