"""FK of the new URDF against the URDF the ROS 1 robot ran. Needs NEW_URDF and REF_URDF."""
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import pytest

ARM = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
# gripper_equilibrium_grasp left out: the cup was re-measured 2026-11-10 (old model = older hardware).
FRAMES = ["arm_base_link", "arm_tool0", "coupling", "capacitive_cup",
          "camera_lower_left_mount", "camera_lower_right_mount"]
POS_TOL = 1e-4
ROT_TOL = 1e-3
# Newer ur_description lowers wrist effort limits from 56 to 54 Nm. Nothing plans on these.
WRIST_EFFORT = {"old": 56.0, "new": 54.0}


def origin(j):
    o = j.find("origin")
    xyz = np.fromstring(o.get("xyz", "0 0 0"), sep=" ") if o is not None else np.zeros(3)
    r, p, y = np.fromstring(o.get("rpy", "0 0 0"), sep=" ") if o is not None else np.zeros(3)
    T = np.eye(4)
    T[:3, :3] = rot([0, 0, 1], y) @ rot([0, 1, 0], p) @ rot([1, 0, 0], r)
    T[:3, 3] = xyz
    return T


def rot(axis, a):
    x, y, z = axis
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], float)
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


def load(path):
    root = ET.parse(path).getroot()
    return {j.find("child").get("link"): j for j in root.findall("joint")}


def fk(joints, link, q):
    T = np.eye(4)
    while link in joints:
        j = joints[link]
        M = origin(j)
        if j.get("type") == "revolute":
            R = np.eye(4)
            axis = np.fromstring(j.find("axis").get("xyz"), sep=" ")
            R[:3, :3] = rot(axis, q[j.get("name")])
            M = M @ R
        T = M @ T
        link = j.find("parent").get("link")
    return link, T


@pytest.fixture(scope="module")
def models():
    return load(os.environ["NEW_URDF"]), load(os.environ["REF_URDF"])


def poses():
    rng = np.random.default_rng(0)
    yield np.zeros(6)
    yield from rng.uniform(-2 * math.pi, 2 * math.pi, size=(200, 6))


def test_same_arm_joints_and_limits(models):
    new, ref = models
    for link in ("shoulder", "upper_arm", "forearm", "wrist_1", "wrist_2", "wrist_3"):
        a, b = new[f"arm_{link}_link"], ref[f"arm_{link}_link"]
        name = a.get("name")
        assert name == b.get("name")
        la, lb = a.find("limit"), b.find("limit")
        for k in ("lower", "upper", "velocity", "effort"):
            if k == "effort" and link.startswith("wrist"):
                assert (float(la.get(k)), float(lb.get(k))) == (WRIST_EFFORT["new"], WRIST_EFFORT["old"])
                continue
            assert float(la.get(k)) == pytest.approx(float(lb.get(k))), (name, k)


@pytest.mark.parametrize("frame", FRAMES)
def test_fk_matches_reference(models, frame):
    new, ref = models
    for q in poses():
        q = {f"arm_{n}_joint": v for n, v in zip(ARM, q)}
        root_new, A = fk(new, frame, q)
        root_ref, B = fk(ref, frame, q)
        assert root_new == root_ref == "base_link"
        assert np.linalg.norm(A[:3, 3] - B[:3, 3]) < POS_TOL
        c = (np.trace(A[:3, :3].T @ B[:3, :3]) - 1) / 2
        assert math.acos(min(1, max(-1, c))) < ROT_TOL


def test_table_not_in_robot_model(models):
    assert "table" not in models[0]
