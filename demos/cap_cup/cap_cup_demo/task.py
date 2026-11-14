"""One pick with the capacitive cup, as the old hard_code_pose state machine did it:

1. Hide (drop_hide), then plan to the approach pose in front of the bin, with every keep-out
   wall on. At the approach the scene swaps the wall at the pod face for the pod's bins
   (scene_bin): the side walls stay, and the bin floors, dividers and lips are modelled.
2. Vacuum on, then move in along the tool axis until the cup feels contact.
3. Haptic search: until the vacuum seals, or the time runs out: back off a little, make the
   cup's proposed pose change (tilt or sideways) in free air, then go in along the cup's axis
   only, stopping if the vacuum seals. Steps alternate on purpose:
   a turn (tilt to level the contact), then a move (sideways towards the load), and so on.
4. Lift, back straight out until a held item as deep as the bin would be clear of the pod face.
   If the vacuum holds, MoveIt gets a bin-sized box attached to the cup (the worst-case item, user
   2026-11-13), so the planned move to hide keeps the item clear of the pod, the table and
   everything else, whichever way the cup points. Blow off at hide, then release the box.

Changes from the old code, on purpose:
- The search ends only on a vacuum seal (or the time limit), not when the cap reads aligned
  (user 2026-11-12): an even press doesn't mean the vacuum caught. Judging seal affordance from
  the capacitance is for a learned model later.
- A proposal is used only if it was made after the last move. The old GetHapticSearchPose
  returned an empty pose at once, before any proposal came.
- No contact on the way in ends the pick ("no_contact") instead of searching in free air.
- The result says whether the vacuum held the item at the end ("picked" or "missed").
- On an error the vacuum is left as it is: turning it off would drop a held item.
- The retreat backs out along the approach direction. The old CLEAR_WALL used the tool axis,
  which the search may have turned, so it could drag the cup sideways inside the bin. It backs
  out only until clear of the pod face: the old fixed 20 cm folded the arm into itself from some
  search end points (T12 item 4), and a planned move does the rest.
- Turn and move alternate on purpose. The old node published both on one topic every 20 ms and
  the old demo took whichever came next, so many picks sealed without ever turning.
"""
import os
import time
from dataclasses import dataclass

import yaml

from tahoma_moveit_config import workcell as W
from tahoma_robot import Stop
from tahoma_robot.errors import PlanningFailed
from tahoma_robot.robot import _rotate, path_from_dict


@dataclass
class Config:
    scene_hide: list
    scene_work: list
    scene_bin: list     # inside the bin: side walls and the pod's bins, no pod-face wall
    pod_face_limit: str  # the limits whose x_max wall is the pod face
    pod: str             # the pod (workcell pods) and bin picked from; the held item is
    pick_bin: str        # modelled as large as that bin
    approach_xyz: list
    approach_quat: list
    approach_link: str
    approach_joints: list = None   # if set, the approach is this joint target (fixed IK branch)
    planner: str = "ompl"          # planned moves to the approach and to hide: ompl or ptp
    pre_approach_joints: list = None  # the approach pose pulled back to where the retreat ends
    path_file: str = ""            # recorded path drop_hide -> pre_approach (run as is, and reversed)
    site_dir: str = ""             # set by the runner; path_file is relative to <site_dir>/demos
    approach_m: float = 0.05
    touch_force_n: float = 15.0
    search_timeout_s: float = 20.0
    clearance_m: float = 0.005
    lift_m: float = 0.03
    retreat_clear_m: float = 0.05   # the held item's far end stops this far in front of the pod face
    held_start_m: float = -0.01     # the held box starts this far along the tool axis from the tip
                                    # (a sealed suction cup pulls the item back ~8 mm)
    straight_speed: float = 0.01
    approach_speed: float = 0.01    # the contact-stopped move in; the search stays at straight_speed
    settle_s: float = 0.0           # pause at the approach pose, vacuum on, before the move in
    planned_speed: float = 0.2


def pick(robot, cup, cfg, log=print):
    """Returns (outcome, details). outcome: picked, missed or no_contact."""
    details = {"search_moves": 0}
    robot.scene.release()   # a held item left by an earlier run that stopped part way
    robot.scene.use(*cfg.scene_hide)
    go(robot, cfg, robot.workcell["named_poses"]["drop_hide"], log)
    robot.scene.use(*cfg.scene_work)
    path = recorded_path(cfg)
    if path and cfg.pre_approach_joints:
        run_path_or_plan(robot, cfg, path_from_dict(path), cfg.pre_approach_joints, log)
        go(robot, cfg, cfg.approach_joints, log)   # 15 cm straight in: PTP is a short straight line here
    elif cfg.approach_joints:
        go(robot, cfg, cfg.approach_joints, log)
    else:
        robot.plan_to(cfg.approach_xyz, cfg.approach_quat, speed=cfg.planned_speed,
                      link=cfg.approach_link)
    log("at the approach pose")
    robot.scene.use(*cfg.scene_bin)

    _, quat = robot.tcp_pose()
    way_in = _rotate(quat, (0.0, 0.0, 1.0))   # the tool axis at the approach, in base_link
    cup.wait_ready()
    cup.suck()   # the old demo kept the vacuum on through the approach and the search
    # Let the hose settle before the move in: each move's force stop is measured from the force
    # at its start (motion_server, 50 ms average), and after the long planned move the hose's pull
    # on the wrist relaxes by ~15 N over ~1 s, which read as a 15 N touch (L5, 2026-11-13).
    time.sleep(cfg.settle_s)
    # New in-air zero for the search here, in front of the bin with the cup pointing the way it will
    # touch: the readings drift and change with the cup's orientation (user 2026-11-13; the zero
    # had been the first sample after the cap drivers started).
    cup.rezero()
    log("search re-zeroed in front of the bin")
    r = robot.move_straight((0, 0, cfg.approach_m), stop=Stop(contact=True, force_n=cfg.touch_force_n),
                            speed=cfg.approach_speed)
    log(f"approach ended: {r.reason}")
    if r.reason != "contact" and r.reason != "force":
        robot.move_straight((0, 0, -cfg.approach_m), speed=cfg.straight_speed)
        cup.off()
        hide(robot, cfg)
        return "no_contact", details

    deadline = time.monotonic() + cfg.search_timeout_s
    ended = "timeout"
    kind = "turn"
    details["turns"] = details["moves"] = 0
    while time.monotonic() < deadline:
        # The last move has ended: wait for a cap sample taken after it, so alignment and the
        # next proposal come from where the cup is now, not from a sample taken while it moved
        # (the board sends every 0.171 s; user 2026-11-12).
        asked = cup.now()
        cup.wait_for_sample(after=asked)
        if cup.sealed():
            ended = "sealed"
            break
        proposal = cup.proposal(kind, newer_than=asked, wait_s=0.5)
        if proposal is None and kind == "turn":
            kind = "move"   # no turn is proposed while the contact is already level
            proposal = cup.proposal(kind, newer_than=asked, wait_s=0.5)
        if proposal is None:
            ended = "no_proposal"
            break
        xyz, quat = proposal
        log(describe(kind, xyz, quat, robot.tcp_pose()[1]))
        # Every pose change happens backed off, then the cup goes in along its own axis only
        # (user 2026-11-13): a combined sideways + in move dragged the touching cup sideways, and
        # a sheared rim reads like load on the opposite side, which sent the search the wrong way.
        robot.move_straight((0, 0, -cfg.clearance_m), speed=cfg.straight_speed)
        robot.move_straight((xyz[0], xyz[1], 0.0), rotation=quat, speed=cfg.straight_speed,
                            stop=Stop(force_n=cfg.touch_force_n))
        press = cfg.clearance_m + xyz[2]   # back to the depth, plus the search's along-axis step
        if press <= 0:
            details["search_moves"] += 1
            details[kind + "s"] += 1
            kind = "move" if kind == "turn" else "turn"
            continue
        r = robot.move_straight((0, 0, press), speed=cfg.straight_speed,
                                stop=Stop(seal=True, force_n=cfg.touch_force_n))
        details["search_moves"] += 1
        details[kind + "s"] += 1
        kind = "move" if kind == "turn" else "turn"
        if r.reason == "seal":
            ended = "sealed"
            break
    details["search_ended"] = ended
    log(f"search ended: {ended} after {details['search_moves']} moves "
        f"({details['turns']} turns, {details['moves']} moves)")

    robot.move_straight((0, 0, cfg.lift_m), frame="base", speed=cfg.straight_speed)
    held = cup.sealed()
    item = held_item_size(robot, cfg)
    retreat(robot, cfg, way_in, item)
    face_x = robot.workcell["limits"][cfg.pod_face_limit]["x_max"]
    log(f"retreat ended: tip x {robot.tcp_pose()[0][0]:.3f}, pod face {face_x:.3f}, "
        f"held-item depth {item[2]:.3f}, farthest box corner x {item_reach_x(robot, cfg, item):.3f}")
    if held:
        robot.scene.hold(item, start=cfg.held_start_m)
        log(f"holding a {item[0]:.3f} x {item[1]:.3f} x {item[2]:.3f} m box at the cup (bin {cfg.pick_bin})")
    hide(robot, cfg, log)
    cup.blow()
    time.sleep(0.5)
    cup.off()
    robot.scene.release()
    return ("picked" if held else "missed"), details


def held_item_size(robot, cfg):
    """The worst-case item from the bin, in the TCP frame: the bin's depth along the tool axis,
    its larger opening side across in both directions (whatever the cup's roll), centred on the
    cup. An item gripped off-centre can reach further; not modelled."""
    lo, hi = W.bin_box(robot.workcell, cfg.pod, cfg.pick_bin)
    width, depth, height = (hi[i] - lo[i] for i in range(3))
    side = max(width, height)
    return (side, side, depth)


def retreat(robot, cfg, way_in, item):
    """Backs straight out along the approach direction until a held item (item: the bin-sized box,
    x, y, z in the TCP frame, from cfg.held_start_m along the tool axis) is retreat_clear_m in front
    of the pod face: first by its depth, then further if a corner of the box, tilted by the search,
    still reaches closer (L5 2026-11-13: a corner at x ~0.95 put the held box in the pod-face wall,
    so the plan to hide refused to start). Then puts the wall back (scene_work)."""
    wall = robot.workcell["limits"][cfg.pod_face_limit]["x_max"] - cfg.retreat_clear_m
    if way_in[0] < 0.5:
        raise ValueError(f"the approach doesn't point into the pod (+x): {way_in}")
    xyz, _ = robot.tcp_pose()
    out = max(0.0, xyz[0] - (wall - item[2])) / way_in[0]
    robot.move_straight(tuple(-out * v for v in way_in), frame="base",
                        speed=cfg.straight_speed * 3)
    extra = item_reach_x(robot, cfg, item) - wall
    if extra > 0:
        robot.move_straight(tuple(-(extra / way_in[0]) * v for v in way_in), frame="base",
                            speed=cfg.straight_speed * 3)
    robot.scene.use(*cfg.scene_work)


def item_reach_x(robot, cfg, item):
    """How far into the pod (base x) the held box's farthest corner reaches at the cup's pose now."""
    xyz, quat = robot.tcp_pose()
    reach = -1e9
    for sx in (-0.5, 0.5):
        for sy in (-0.5, 0.5):
            for z in (cfg.held_start_m, cfg.held_start_m + item[2]):
                corner = _rotate(quat, (sx * item[0], sy * item[1], z))
                reach = max(reach, xyz[0] + corner[0])
    return reach


def hide(robot, cfg, log=print):
    """Planned move to drop_hide. A held item, if any, is attached to the cup (scene.hold). With a
    recorded path: back to the pre-approach point first (undoes the search's turns), then the
    recorded path reversed."""
    robot.scene.use(*cfg.scene_hide)
    path = recorded_path(cfg)
    hide_joints = robot.workcell["named_poses"]["drop_hide"]
    if path and cfg.pre_approach_joints:
        go(robot, cfg, cfg.pre_approach_joints, log)
        run_path_or_plan(robot, cfg, path_from_dict(path, reverse=True), hide_joints, log)
    else:
        go(robot, cfg, hide_joints, log)


def recorded_path(cfg):
    if not cfg.path_file:
        return None
    full = os.path.join(cfg.site_dir, "demos", cfg.path_file)
    if not os.path.exists(full):
        return None
    with open(full) as f:
        return yaml.safe_load(f)


def run_path_or_plan(robot, cfg, trajectory, joints, log=print):
    """The recorded path (user 2026-11-13: the same path every pick, so the hose is dragged the same
    way). If the arm isn't at its start, or a point would collide (e.g. with a held item), log it
    and plan instead."""
    try:
        return robot.run_path(trajectory)
    except PlanningFailed as e:
        log(f"RECORDED PATH NOT USED ({e}); planning instead")
    return go(robot, cfg, joints, log)


def go(robot, cfg, joints, log=print):
    """Planned move to joints with cfg.planner. With ptp (the same straight joint-space path every
    pick, user 2026-11-13: a sampled path drags the hose differently each time), a path that
    would collide falls back to ompl, logged, so the arm isn't left part way."""
    if cfg.planner == "ptp":
        try:
            return robot.go_joints(joints, speed=cfg.planned_speed, planner="ptp")
        except PlanningFailed as e:
            log(f"PTP FAILED ({e}); falling back to a sampled (ompl) path")
    return robot.go_joints(joints, speed=cfg.planned_speed)


def describe(kind, xyz, quat, tool_quat):
    """A search step in room words, to compare with what the person at the arm sees (user
    2026-11-13: sideways moves looked the wrong way). Base frame: +x into the pod, +y left, +z up."""
    import math
    d = _rotate(tool_quat, xyz)
    words = []
    for value, plus, minus in ((d[1], "left", "right"), (d[2], "up", "down"), (d[0], "in", "out")):
        if abs(value) >= 0.0005:
            words.append(f"{abs(value) * 1000:.1f} mm {plus if value > 0 else minus}")
    angle = 2 * math.degrees(math.acos(max(-1.0, min(1.0, abs(quat[3])))))
    if angle >= 0.1:
        axis = _rotate(tool_quat, quat[:3])
        n = math.sqrt(sum(v * v for v in axis)) or 1.0
        words.append(f"tilt {angle:.1f} deg about ({axis[0] / n:+.2f}, {axis[1] / n:+.2f}, {axis[2] / n:+.2f})")
    return f"{kind}: " + (", ".join(words) or "no change")
