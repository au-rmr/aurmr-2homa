"""Named poses and scenes, using MoveIt's own collision checking. No ROS graph needed."""
import os
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from geometry_msgs.msg import Pose
from moveit.core.planning_scene import PlanningScene
from moveit.core.robot_model import RobotModel
from moveit.core.robot_state import RobotState
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from tahoma_moveit_config import workcell as W

GROUP = "manipulator"
ARM = [f"arm_{n}_joint" for n in
       ("shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3")]
# As used by the old demos: table in the down position with each pod scene.
SCENES = [[], ["table_down"], ["limits_full"], ["limits_drop_hide"], ["pod_partial"],
          ["pod_bins"], ["table_down", "limits_full"], ["table_down", "limits_drop_hide"]]
TOOL_LINKS = {"coupling", "capacitive_cup"}
SEARCH = 3000


@pytest.fixture(scope="module")
def wc():
    return W.load(os.environ["WORKCELL"])


@pytest.fixture(scope="module")
def model():
    return RobotModel(urdf_xml_path=os.environ["NEW_URDF"], srdf_xml_path=os.environ["NEW_SRDF"])


def scene(model, wc, names):
    """moveit_py gives every PlanningScene the same world, so each call clears it first.
    Only the most recently built scene is valid: never hold two at once."""
    ps = PlanningScene(model)
    ps.remove_all_collision_objects()
    for name in W.object_names(wc, names):
        ps.apply_collision_object(W.collision_object(wc, name))
    return ps


def state(model, q):
    st = RobotState(model)
    st.set_joint_group_positions(GROUP, list(q))
    st.update()
    return st


def limits():
    root = ET.parse(os.environ["NEW_URDF"]).getroot()
    by_name = {j.get("name"): j.find("limit") for j in root.findall("joint")}
    return [(float(by_name[n].get("lower")), float(by_name[n].get("upper"))) for n in ARM]


def box(name, x, y, z):
    co = CollisionObject()
    co.id = name
    co.header.frame_id = "base_link"
    co.operation = CollisionObject.ADD
    co.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.06, 0.06, 0.06]))
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = float(x), float(y), float(z)
    pose.orientation.w = 1.0
    co.primitive_poses.append(pose)
    return co


def test_named_poses_within_joint_limits(wc):
    bounds = limits()
    for name, q in wc["named_poses"].items():
        for value, (low, high) in zip(q, bounds, strict=True):
            assert low <= value <= high, name


@pytest.mark.parametrize("names", SCENES, ids=lambda n: "+".join(n) or "empty")
def test_named_poses_collision_free(model, wc, names):
    ps = scene(model, wc, names)
    hits = [n for n, q in wc["named_poses"].items()
            if ps.is_state_colliding(state(model, q), GROUP)]
    assert hits == []


def collisions(ps, model, seed=0):
    bounds = np.array(limits())
    rng = np.random.default_rng(seed)
    qs = rng.uniform(bounds[:, 0], bounds[:, 1], size=(SEARCH, 6))
    return [q for q in qs if ps.is_state_colliding(state(model, q), GROUP)]


@pytest.mark.parametrize("names", [["table_down"], ["limits_full"], ["pod_partial"], ["pod_bins"]],
                         ids=lambda n: n[0])
def test_scene_object_blocks_poses(model, wc, names):
    """Control: without this, an all-clear above could mean nothing was checked."""
    empty = {tuple(q) for q in collisions(scene(model, wc, []), model)}
    with_scene = {tuple(q) for q in collisions(scene(model, wc, names), model)}
    assert len(with_scene - empty) > 0
    assert empty <= with_scene


def test_scenes_do_not_leak(model, wc):
    ps = scene(model, wc, ["table_down"])
    ps = scene(model, wc, ["limits_full"])
    ids = {o.id for o in ps.planning_scene_message.world.collision_objects}
    assert ids == set(wc["scenes"]["limits_full"])


@pytest.mark.parametrize("limit", ["limits_full", "limits_drop_hide"])
def test_limit_walls_are_vertical_on_the_limits(wc, limit):
    bounds = wc["limits"][limit]
    for name in wc["scenes"][limit]:
        co = W.collision_object(wc, name)
        pose, size = co.primitive_poses[0], co.primitives[0].dimensions
        assert co.header.frame_id == "base_link"
        assert (pose.orientation.x, pose.orientation.y, pose.orientation.z) == pytest.approx((0, 0, 0))
        faces = {"x_max": pose.position.x - size[0] / 2, "y_min": pose.position.y + size[1] / 2,
                 "y_max": pose.position.y - size[1] / 2}
        side = name.removeprefix(limit + "_")
        assert faces[side] == pytest.approx(bounds[side])


def test_arm_self_collision_detected(model):
    assert len(collisions(scene(model, wc, []), model)) > 0


@pytest.mark.parametrize("link", ["arm_forearm_link", "coupling", "capacitive_cup"])
def test_object_on_link_collides(model, wc, link):
    """A clear pose stops being clear when a box sits on the link, cup included."""
    st = state(model, wc["named_poses"]["home"])
    assert not scene(model, wc, []).is_state_colliding(st, GROUP)
    x, y, z = st.get_global_link_transform(link)[:3, 3]
    ps = scene(model, wc, [])
    ps.apply_collision_object(box("probe", x, y, z))
    assert ps.is_state_colliding(st, GROUP)


def test_far_object_does_not_collide(model, wc):
    st = state(model, wc["named_poses"]["home"])
    ps = scene(model, wc, [])
    ps.apply_collision_object(box("probe", 5.0, 5.0, 5.0))
    assert not ps.is_state_colliding(st, GROUP)


def test_tool_only_adjacent_exceptions():
    root = ET.parse(os.environ["NEW_SRDF"]).getroot()
    for e in root.iter("disable_collisions"):
        if {e.get("link1"), e.get("link2")} & TOOL_LINKS:
            assert e.get("reason") == "Adjacent", (e.get("link1"), e.get("link2"))


def _extent(spec):
    c, h = np.array(spec["xyz"]), np.array(spec["size"]) / 2
    return c - h, c + h


def test_pod_bins_leave_each_bin_open(wc):
    """Every bin's inside, less half a modelled wall on each side, is free of the bin boxes:
    bin_box and the generated walls agree. The front lip sits inside that margin."""
    spec = wc["pods"]["pod_bins"]
    boxes = [_extent(wc["objects"][n]) for n in wc["scenes"]["pod_bins"]]
    margin = spec["min_wall"] / 2 + 1e-9
    rows = "abcdefghijklmnopqrstuvwxyz"[:len(spec["row_heights"])]
    for col in range(1, spec["columns"] + 1):
        for row in rows:
            lo, hi = (np.array(v) for v in W.bin_box(wc, "pod_bins", f"{col}{row}"))
            lo, hi = lo + margin, hi - margin
            hits = [n for n, (blo, bhi) in zip(wc["scenes"]["pod_bins"], boxes)
                    if np.all(blo < hi) and np.all(bhi > lo)]
            assert hits == [], f"bin {col}{row} blocked by {hits}"


def test_bin_2h_where_the_demo_works(wc):
    """The demo's bin, in base_link: about 0.23 m wide, its face near the measured pod face."""
    lo, hi = W.bin_box(wc, "pod_bins", "2h")
    corners = [(lo[0], lo[1], lo[2]), (hi[0], lo[1], lo[2]), (lo[0], lo[1], hi[2])]
    T = W._matrix(wc["frames"]["pod_base_link"]["xyz"], wc["frames"]["pod_base_link"]["rpy"])
    base = [(T @ np.r_[c, 1])[:3] for c in corners]
    face_x = wc["limits"]["limits_full"]["x_max"]
    assert all(abs(p[0] - face_x) < 0.04 for p in base)
    assert np.linalg.norm(base[0] - base[1]) == pytest.approx(0.234, abs=1e-6)
    assert 1.4 < base[0][2] < 1.6
