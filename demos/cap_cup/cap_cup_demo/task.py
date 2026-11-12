"""One pick with the capacitive cup, as the old hard_code_pose state machine did it:

1. Hide (drop_hide), then plan to the approach pose in front of the bin, with every keep-out
   wall on. At the approach the scene swaps the wall at the pod face for the pod's bins
   (scene_bin): the side walls stay, and the bin floors, dividers and lips are modelled.
2. Vacuum on, then move in along the tool axis until the cup feels contact.
3. Haptic search: until aligned or sealed, or the time runs out: back off a little, then move
   by the cup's proposed pose change, stopping if the vacuum seals. Steps alternate on purpose:
   a turn (tilt to level the contact), then a move (sideways towards the load), and so on.
4. Lift, back straight out until the tool is clear of the pod-face wall, put the wall back,
   then a planned move to hide, and blow off.

Changes from the old code, on purpose:
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
import time
from dataclasses import dataclass

from tahoma_robot import Stop
from tahoma_robot.robot import _rotate


@dataclass
class Config:
    scene_hide: list
    scene_work: list
    scene_bin: list     # inside the bin: side walls and the pod's bins, no pod-face wall
    pod_face_limit: str  # the limits whose x_max wall is the pod face
    approach_xyz: list
    approach_quat: list
    approach_link: str
    approach_m: float = 0.05
    touch_force_n: float = 15.0
    search_timeout_s: float = 20.0
    clearance_m: float = 0.005
    lift_m: float = 0.03
    retreat_clear_m: float = 0.05   # the tool tip ends this far in front of the pod-face wall
    straight_speed: float = 0.01
    planned_speed: float = 0.2


def pick(robot, cup, cfg, log=print):
    """Returns (outcome, details). outcome: picked, missed or no_contact."""
    details = {"search_moves": 0}
    robot.scene.use(*cfg.scene_hide)
    robot.go_named("drop_hide", speed=cfg.planned_speed)
    robot.scene.use(*cfg.scene_work)
    robot.plan_to(cfg.approach_xyz, cfg.approach_quat, speed=cfg.planned_speed,
                  link=cfg.approach_link)
    log("at the approach pose")
    robot.scene.use(*cfg.scene_bin)

    _, quat = robot.tcp_pose()
    way_in = _rotate(quat, (0.0, 0.0, 1.0))   # the tool axis at the approach, in base_link
    cup.wait_ready()
    cup.suck()   # the old demo kept the vacuum on through the approach and the search
    r = robot.move_straight((0, 0, cfg.approach_m), stop=Stop(contact=True, force_n=cfg.touch_force_n),
                            speed=cfg.straight_speed)
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
        if cup.aligned():
            ended = "aligned"
            break
        proposal = cup.proposal(kind, newer_than=asked, wait_s=0.5)
        if proposal is None and kind == "turn":
            kind = "move"   # no turn is proposed while the contact is already level
            proposal = cup.proposal(kind, newer_than=asked, wait_s=0.5)
        if proposal is None:
            ended = "no_proposal"
            break
        xyz, quat = proposal
        robot.move_straight((0, 0, -cfg.clearance_m), speed=cfg.straight_speed)
        offset = (xyz[0], xyz[1], xyz[2] + cfg.clearance_m)
        r = robot.move_straight(offset, rotation=quat, speed=cfg.straight_speed,
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
    retreat(robot, cfg, way_in)
    hide(robot, cfg)
    cup.blow()
    time.sleep(0.5)
    cup.off()
    return ("picked" if held else "missed"), details


def retreat(robot, cfg, way_in):
    """Backs straight out along the approach direction until the tool tip is retreat_clear_m in
    front of the pod-face wall, then puts the wall back (scene_work)."""
    face_x = robot.workcell["limits"][cfg.pod_face_limit]["x_max"]
    if way_in[0] < 0.5:
        raise ValueError(f"the approach doesn't point into the pod (+x): {way_in}")
    xyz, _ = robot.tcp_pose()
    out = max(0.0, xyz[0] - (face_x - cfg.retreat_clear_m)) / way_in[0]
    robot.move_straight(tuple(-out * v for v in way_in), frame="base",
                        speed=cfg.straight_speed * 3)
    robot.scene.use(*cfg.scene_work)


def hide(robot, cfg):
    robot.scene.use(*cfg.scene_hide)
    robot.go_named("drop_hide", speed=cfg.planned_speed)
