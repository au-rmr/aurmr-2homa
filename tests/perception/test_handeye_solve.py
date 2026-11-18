"""handeye_solve.py against synthetic data with a known answer: AprilTags rendered from a known camera pose and
known tag-on-link poses, written in the same files the real capture writes (<pose>_tf.json + <pose>.npz). The
solver runs unchanged, as a command, and must recover the camera and pick each tag's link out of several.

Runs in the perception image (OpenCV 4.6 + scipy; base:dev has no OpenCV):
  docker run --rm --network none -u $(id -u):$(id -g) -e HOME=/tmp -v $PWD:/repo:ro -w /tmp \
    aurmr-2homa/perception:draft python3 -m pytest -q -p no:cacheprovider /repo/tests/perception

Rendering is pinhole with zero distortion (a homography warp is exact only then), so this does not check the
solver's use of the rational-polynomial distortion model.
"""
import json
import os
import subprocess
import sys

import cv2
import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SOLVER = os.path.join(HERE, "handeye_solve.py")

W, H = 2048, 1536
K = np.array([[1100.0, 0, 1024.0], [0, 1100.0, 768.0], [0, 0, 1]])
D = np.zeros(8)
SIZE = 0.05          # black square edge, m
FAMILY = "36h11"
CAM = np.array([1.3, -0.9, 1.9])      # base_link
TARGET = np.array([0.5, 0.0, 1.2])    # where the arm holds the tags
# Limits, from 5 seeds of the two-tag scene (2026-11-17): camera off 1.1 to 4.6 mm and 0.04 to 0.19 deg, rms 0.33 to
# 0.42 px (0.53 to 0.59 with the oblique shoulder tag), tag on link within 0.4 mm. The rms is the detector's own: on these ~40 px tags its corners are off by
# 0.37 px rms with no mean bias, before any noise. A 10 % wrong --tag-size gives about 50 mm, 2 deg, 2.4 px.
POS_MM, ROT_DEG, RMS_PX = 8.0, 0.35, 0.8


def T(R, t):
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = R, t
    return m


def rot(axis, deg):
    return cv2.Rodrigues(np.asarray(axis, float) / np.linalg.norm(axis) * np.radians(deg))[0]


def look_at(eye, target):
    """Camera frame as handeye_solve reports it: z forward, x right, y down."""
    z = (target - eye) / np.linalg.norm(target - eye)
    x = np.cross([0, 0, -1.0], z)
    x /= np.linalg.norm(x)
    return T(np.column_stack([x, np.cross(z, x), z]), eye)


def facing(p, rng, tilt_deg=35):
    """A tag pose at p whose face (tag z) points at the camera, tilted up to tilt_deg, spun at random."""
    n = (CAM - p) / np.linalg.norm(CAM - p)
    a = np.cross(n, [0, 0, 1.0])
    a /= np.linalg.norm(a)
    a = rot(n, rng.uniform(0, 360)) @ a
    n = rot(a, rng.uniform(0, tilt_deg)) @ n
    x = np.cross([0, 0, 1.0], n)
    x /= np.linalg.norm(x)
    R = np.column_stack([x, np.cross(n, x), n])
    return T(R @ rot([0, 0, 1], rng.uniform(-180, 180)), p)


def quat_xyzw(R):
    q = np.empty(4)
    tr = np.trace(R)
    if tr > 0:
        s = 2 * np.sqrt(1 + tr)
        q = [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, s / 4]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2 * np.sqrt(1 + R[i, i] - R[j, j] - R[k, k])
        q = [0.0] * 4
        q[i], q[j], q[k] = s / 4, (R[j, i] + R[i, j]) / s, (R[k, i] + R[i, k]) / s
        q[3] = (R[k, j] - R[j, k]) / s
    return [float(v) for v in q]


def render(img, T_cam_tag, tag_id):
    """Draws the tag into img in place; returns its projected corners (OpenCV order)."""
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, f"DICT_APRILTAG_{FAMILY}"))
    side, margin = 8 * 24, 48
    marker = cv2.aruco.drawMarker(dictionary, tag_id, side)
    canvas = np.full((side + 2 * margin,) * 2, 255, np.uint8)
    canvas[margin:margin + side, margin:margin + side] = marker
    h = SIZE / 2
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]])
    pc = (T_cam_tag[:3, :3] @ obj.T).T + T_cam_tag[:3, 3]
    assert np.all(pc[:, 2] > 0.2)
    corners, _ = cv2.projectPoints(pc, np.zeros(3), np.zeros(3), K, D)
    corners = corners.reshape(4, 2)
    assert np.all((corners > 60) & (corners < [W - 60, H - 60])), "tag out of the image: change the scene"
    # The white quiet zone is drawn too: scale the black square's corners out by the margin.
    e0, e1 = margin - 0.5, margin + side - 0.5
    src = np.float32([[e0, e0], [e1, e0], [e1, e1], [e0, e1]])
    Hm = cv2.getPerspectiveTransform(src, corners.astype(np.float32))
    warped = cv2.warpPerspective(canvas, Hm, (W, H), flags=cv2.INTER_LINEAR).astype(float)
    mask = cv2.warpPerspective(np.ones_like(canvas, float), Hm, (W, H), flags=cv2.INTER_LINEAR)
    img[:] = mask * warped + (1 - mask) * img
    return corners


def make_scene(out, tags, n_poses, seed):
    """tags: {tag_id: (link, T_link_tag, pose_fn(rng, k) -> T_base_tag)}. Writes the files; links not carrying
    a tag ('forearm') move at random, as distractors for the solver's link choice."""
    rng = np.random.default_rng(seed)
    T_base_cam = look_at(CAM, TARGET)
    for k in range(n_poses):
        links = {"forearm": T(rot(rng.normal(size=3), rng.uniform(0, 180)), TARGET + rng.uniform(-0.2, 0.2, 3))}
        img = np.full((H, W), 110.0)
        boxes = []
        for tag_id, (link, T_link_tag, pose_fn) in tags.items():
            T_base_tag = pose_fn(rng, k)
            links[link] = T_base_tag @ np.linalg.inv(T_link_tag)
            c = render(img, np.linalg.inv(T_base_cam) @ T_base_tag, tag_id)
            boxes.append((c.min(0) - 40, c.max(0) + 40))
        for i in range(len(boxes)):
            for j in range(i):
                assert np.any(boxes[i][1] < boxes[j][0]) or np.any(boxes[j][1] < boxes[i][0]), "tags overlap"
        img += rng.normal(0, 1.5, img.shape)
        rgb = np.repeat(np.clip(img, 0, 255).astype(np.uint8)[..., None], 3, axis=2)
        np.savez(os.path.join(out, f"p{k:02d}.npz"), rgb=rgb, K=K, D=D)
        frames = {l: {"xyz": [float(v) for v in m[:3, 3]], "quat_xyzw": quat_xyzw(m[:3, :3]), "spread_mm": 0.05}
                  for l, m in links.items()}
        with open(os.path.join(out, f"p{k:02d}_tf.json"), "w") as f:
            json.dump({"base": "base_link", "frames": frames, "spread_mm": 0.05}, f)
    return T_base_cam


def solve(out, tag_ids):
    result = os.path.join(out, "solve.json")
    p = subprocess.run([sys.executable, SOLVER, out, "--tags", *map(str, tag_ids), "--tag-size", str(SIZE),
                        "--family", FAMILY, "--out", result], capture_output=True, text=True, timeout=600)
    assert p.returncode == 0, p.stdout + p.stderr
    with open(result) as f:
        return json.load(f), p.stdout


def quat_to_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def errors(res, T_true):
    pos = 1000 * np.linalg.norm(np.array(res["xyz"]) - T_true[:3, 3])
    dR = quat_to_R(res["quat_xyzw"]).T @ T_true[:3, :3]
    return pos, float(np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1))))


def tag_pose_error(res, tag_id, T_link_tag):
    t = res["tags"][str(tag_id)]
    return 1000 * np.linalg.norm(np.array(t["xyz"]) - T_link_tag[:3, 3])


T_WRIST_TAG = T(rot([1, 2, 0.5], 20), [0.02, -0.03, 0.06])
T_ELBOW_TAG = T(rot([0.3, -1, 1], 35), [-0.04, 0.01, 0.09])
T_SHOULDER_TAG = T(rot([0, 1, 0], 90) @ rot([0, 0, 1], 15), [0.0, -0.12, 0.05])


def free_tag(offset):
    return lambda rng, k: facing(TARGET + offset + rng.uniform(-0.05, 0.05, 3), rng)


def test_recovers_camera_and_links_two_tags(tmp_path):
    tags = {35: ("wrist_3_link", T_WRIST_TAG, free_tag(np.array([0.11, 0.10, 0.03]))),
            43: ("forearm_link", T_ELBOW_TAG, free_tag(np.array([-0.11, -0.10, -0.03])))}
    T_true = make_scene(str(tmp_path), tags, n_poses=10, seed=1)
    res, log = solve(str(tmp_path), [35, 43, 42])   # 42 is never seen: must be left out, not fail
    pos, ang = errors(res, T_true)
    assert pos < POS_MM and ang < ROT_DEG, f"camera off by {pos:.2f} mm, {ang:.3f} deg\n{log}"
    assert res["reprojection_rms_px"] < RMS_PX, log
    assert res["tags"]["35"]["link"] == "wrist_3_link", log
    assert res["tags"]["43"]["link"] == "forearm_link", log
    assert "42" not in res["tags"] and "tag 42: seen in 0 poses" in log
    for tag_id, (_, T_lt, _) in tags.items():
        assert tag_pose_error(res, tag_id, T_lt) < 2 * POS_MM, log


def test_tag_that_only_turns_about_one_axis_does_not_spoil_the_camera(tmp_path):
    """The case the solver's multi-start joint fit is for, laid out like the lab's (tags on wrist, forearm and the
    shoulder's base joint cap): the shoulder tag only ever turns about base z, so its own hand-eye solution leaves
    the camera undetermined. The joint fit must still match the fit without it.
    Measured 2026-11-17, 5 seeds: with it 1.7 to 4.1 mm, without 1.2 to 4.5 mm, the two within 1.2 mm. With only
    ONE other tag (not tested here) the shoulder tag made the camera 2 to 4 mm worse on every seed."""
    shoulder_origin = np.array([0.2, 0.35, 1.45])

    def on_shoulder(rng, k):
        return T(rot([0, 0, 1], -75 + 6 * k + rng.uniform(-2, 2)), shoulder_origin) @ T_SHOULDER_TAG

    tags = {35: ("wrist_3_link", T_WRIST_TAG, free_tag(np.array([0.11, 0.10, 0.03]))),
            43: ("forearm_link", T_ELBOW_TAG, free_tag(np.array([-0.11, -0.10, -0.03]))),
            39: ("shoulder_link", T_SHOULDER_TAG, on_shoulder)}
    T_true = make_scene(str(tmp_path), tags, n_poses=10, seed=2)
    res, log = solve(str(tmp_path), [35, 43, 39])
    pos, ang = errors(res, T_true)
    assert pos < POS_MM and ang < ROT_DEG, f"camera off by {pos:.2f} mm, {ang:.3f} deg\n{log}"
    assert res["reprojection_rms_px"] < RMS_PX, log
    assert res["tags"]["39"]["link"] == "shoulder_link", log
    assert res["tags"]["39"]["single_vs_joint_mm_deg"][0] > 100, "shoulder tag alone fixed the camera: not the case\n" + log
    without, _ = solve(str(tmp_path), [35, 43])
    moved = 1000 * np.linalg.norm(np.array(res["xyz"]) - np.array(without["xyz"]))
    assert moved < 3.0, f"the one-axis tag moved the camera {moved:.2f} mm\n{log}"


def test_too_few_views_is_refused(tmp_path):
    tags = {35: ("wrist_3_link", T_WRIST_TAG, free_tag(np.zeros(3)))}
    make_scene(str(tmp_path), tags, n_poses=3, seed=3)
    p = subprocess.run([sys.executable, SOLVER, str(tmp_path), "--tags", "35", "--tag-size", str(SIZE)],
                       capture_output=True, text=True, timeout=120)
    assert p.returncode != 0 and "no tag seen in enough poses" in p.stdout + p.stderr


@pytest.mark.parametrize("wrong", [0.9, 1.1])
def test_wrong_tag_size_shows_as_a_camera_error(tmp_path, wrong):
    """A 10 % wrong --tag-size scales every PnP distance. Proves the limits above can fail: the camera must come
    out well outside them, and the rms too (if not, the limits would be too loose to catch a real mistake)."""
    tags = {35: ("wrist_3_link", T_WRIST_TAG, free_tag(np.array([0.11, 0.10, 0.03]))),
            43: ("forearm_link", T_ELBOW_TAG, free_tag(np.array([-0.11, -0.10, -0.03])))}
    T_true = make_scene(str(tmp_path), tags, n_poses=10, seed=1)
    result = os.path.join(str(tmp_path), "solve.json")
    subprocess.run([sys.executable, SOLVER, str(tmp_path), "--tags", "35", "43", "--tag-size", str(SIZE * wrong),
                    "--out", result], capture_output=True, text=True, timeout=600, check=True)
    with open(result) as f:
        res = json.load(f)
    pos, ang = errors(res, T_true)
    assert pos > 3 * POS_MM and ang > 3 * ROT_DEG and res["reprojection_rms_px"] > 2 * RMS_PX, (pos, ang, res)


def test_closed_form_is_exact_on_exact_data():
    """cv2.calibrateHandEye in the eye-to-hand form, without images or noise: the camera must come back exactly.
    The image tests can't see a mistake here (the refinement recovers from a wrong start), so check it alone."""
    sys.path.insert(0, HERE)
    import handeye_solve
    rng = np.random.default_rng(4)
    T_base_cam = look_at(CAM, TARGET)
    samples = []
    for _ in range(8):
        T_base_link = T(rot(rng.normal(size=3), rng.uniform(10, 120)), TARGET + rng.uniform(-0.2, 0.2, 3))
        samples.append((T_base_link, np.linalg.inv(T_base_cam) @ T_base_link @ T_WRIST_TAG))
    T_fit, spread, T_link_tag, method = handeye_solve.closed_form(samples)
    assert np.allclose(T_fit, T_base_cam, atol=1e-6), method
    assert np.allclose(T_link_tag, T_WRIST_TAG, atol=1e-6) and spread < 1e-6
