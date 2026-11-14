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
    assert np.linalg.norm(base[0] - base[1]) == pytest.approx(wc["pods"]["pod_bins"]["bin_width"], abs=1e-6)
    assert 1.4 < base[0][2] < 1.6


def test_soft_bin_gives_the_canvas_allowance(wc):
    """pod_bins_soft_<bin>: the bin's inside plus canvas_allowance on every side (front open, no
    lip) is free; one more wall's thickness out is blocked on the floor, sides and back."""
    spec = wc["pods"]["pod_bins"]
    a, t = spec["canvas_allowance"], spec["min_wall"]
    for b in ("2h", "1a", "4l"):
        names = wc["scenes"][f"pod_bins_soft_{b}"]
        boxes = [_extent(wc["objects"][n]) for n in names]
        lo, hi = (np.array(v) for v in W.bin_box(wc, "pod_bins", b))
        free_lo, free_hi = lo - [a, 0, a] + 1e-9, hi + a - 1e-9
        assert not [n for n, (blo, bhi) in zip(names, boxes) if np.all(blo < free_hi) and np.all(bhi > free_lo)]
        for probe in ([lo[0] - a - t / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2],
                      [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2] - a - t / 2],
                      [(lo[0] + hi[0]) / 2, hi[1] + a + t / 2, (lo[2] + hi[2]) / 2]):
            assert any(np.all(blo <= probe) and np.all(bhi >= probe) for blo, bhi in boxes), (b, probe)




def test_drop_hide_clear_with_a_held_bin_sized_item(model, wc):
    """cap_cup attaches a box as large as the bin it picked from to the cup for the planned move
    to hide (2026-11-13). drop_hide itself must be clear with it, or every hide after a pick fails.
    Control: a small object 10 cm past the cup tip collides only while the box is attached."""
    import yaml
    from moveit_msgs.msg import AttachedCollisionObject
    site = os.path.dirname(os.environ["WORKCELL"])
    with open(os.path.join(site, "demos", "cap_cup.yaml")) as f:
        cfg = yaml.safe_load(f)
    lo, hi = W.bin_box(wc, cfg["pod"], cfg["pick_bin"])
    side, depth = max(hi[0] - lo[0], hi[2] - lo[2]), hi[1] - lo[1]
    tcp = "gripper_equilibrium_grasp"
    st = state(model, wc["named_poses"]["drop_hide"])
    T = st.get_global_link_transform(tcp)
    probe = T[:3, 3] + 0.10 * T[:3, 2]
    ps = scene(model, wc, cfg["scene_hide"])
    ps.current_state = st
    ps.apply_collision_object(box("probe", *probe))
    assert not ps.is_state_colliding(GROUP)
    co = CollisionObject(id="held_item", operation=CollisionObject.ADD)
    co.header.frame_id = tcp
    co.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[side, side, depth]))
    pose = Pose()
    pose.position.z, pose.orientation.w = -0.01 + depth / 2, 1.0
    co.primitive_poses.append(pose)
    ps.process_attached_collision_object(AttachedCollisionObject(
        link_name=tcp, object=co,
        touch_links=["coupling", "capacitive_cup", "gripper_equilibrium_grasp", "cup_aluminium_end"]))
    assert ps.is_state_colliding(GROUP)
    ps.apply_collision_object(CollisionObject(id="probe", operation=CollisionObject.REMOVE))
    assert not ps.is_state_colliding(GROUP)


def test_cap_cup_approach_joints_reach_the_approach_pose(model, wc):
    """approach_joints (a fixed IK branch) must put approach_link at approach_xyz/quat."""
    import yaml
    site = os.path.dirname(os.environ["WORKCELL"])
    with open(os.path.join(site, "demos", "cap_cup.yaml")) as f:
        cfg = yaml.safe_load(f)
    T = state(model, cfg["approach_joints"]).get_global_link_transform(cfg["approach_link"])
    assert np.linalg.norm(T[:3, 3] - np.array(cfg["approach_xyz"])) < 0.002
    x, y, z, w = cfg["approach_quat"]
    Rq = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                   [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                   [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    angle = np.degrees(np.arccos(np.clip((np.trace(Rq.T @ T[:3, :3]) - 1) / 2, -1, 1)))
    assert angle < 0.5
    ps = scene(model, wc, cfg["scene_work"])
    assert not ps.is_state_colliding(state(model, cfg["approach_joints"]), GROUP)


def test_cap_cup_pre_approach_is_the_approach_pulled_back(model, wc):
    """pre_approach_joints: approach_link 0.154 m back along the approach direction (-x here), same
    orientation, collision-free in the work and hide scenes (the recorded path ends there)."""
    import yaml
    site = os.path.dirname(os.environ["WORKCELL"])
    with open(os.path.join(site, "demos", "cap_cup.yaml")) as f:
        cfg = yaml.safe_load(f)
    A = state(model, cfg["approach_joints"]).get_global_link_transform(cfg["approach_link"])
    P = state(model, cfg["pre_approach_joints"]).get_global_link_transform(cfg["approach_link"])
    assert np.linalg.norm(P[:3, 3] - (A[:3, 3] - np.array([0.154, 0, 0]))) < 0.002
    assert np.degrees(np.arccos(np.clip((np.trace(A[:3, :3].T @ P[:3, :3]) - 1) / 2, -1, 1))) < 0.5
    for names in (cfg["scene_work"], cfg["scene_hide"]):
        ps = scene(model, wc, names)
        assert not ps.is_state_colliding(state(model, cfg["pre_approach_joints"]), GROUP)
