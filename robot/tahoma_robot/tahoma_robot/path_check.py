"""Joint jump check for straight moves, done on the trajectory MoveIt returns.

MoveIt 2.12.4's compute_cartesian_path ignores revolute_jump_threshold and jump_threshold, and its
points are re-timed samples, not the 5 mm IK steps. So the limit is applied here against progress
along the commanded line: no joint may travel more than MAX_JOINT_STEP_RAD per STEP_M of progress
(plus one step of slack), and the tip may not leave the line by more than MAX_OFF_LINE_M.
An IK branch switch shows up as large joint travel with little progress.
"""
import math

MAX_JOINT_STEP_RAD = 0.3   # user, 2026-11-09
STEP_M = 0.005             # the Cartesian step asked of MoveIt
MAX_OFF_LINE_M = 0.005     # good lines stay within about 0.2 mm on mock
LEVER_M = 0.1              # turns count as tip travel at this radius


def _conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _turn_about(q_from, q_to, axis):
    """Angle of the turn q_from -> q_to (tool frame) about a unit axis."""
    r = _qmul(_conj(q_from), q_to)
    if r[3] < 0:
        r = tuple(-v for v in r)
    return 2 * math.atan2(sum(a * b for a, b in zip(r[:3], axis)), r[3])


def find_jump(positions, tips, start, end):
    """None if the path is fine, else a reason.

    positions: joint positions per trajectory point. tips: (xyz, quat) per point from FK.
    start, end: (xyz, quat) of the commanded line.
    """
    (p0, q0), (p1, q1) = start, end
    line = [b - a for a, b in zip(p0, p1)]
    length = math.sqrt(sum(v * v for v in line))
    direction = [v / length for v in line] if length > 1e-6 else None
    r = _qmul(_conj(q0), q1)
    if r[3] < 0:
        r = tuple(-v for v in r)
    turn = 2 * math.acos(min(1.0, r[3]))
    axis = [v / math.sin(turn / 2) for v in r[:3]] if turn > 1e-6 else None

    progress = []
    for xyz, quat in tips:
        rel = [b - a for a, b in zip(p0, xyz)]
        along = sum(a * b for a, b in zip(rel, direction)) if direction else 0.0
        closest = [p0[i] + (direction[i] * min(max(along, 0.0), length) if direction else 0.0)
                   for i in range(3)]
        off = math.dist(xyz, closest)
        if off > MAX_OFF_LINE_M:
            return f"the tip leaves the line by {off * 1000:.0f} mm"
        turned = _turn_about(q0, quat, axis) if axis else 0.0
        progress.append(along + LEVER_M * turned)

    per_m = MAX_JOINT_STEP_RAD / STEP_M
    for j in range(len(positions[0])):
        travel, lowest = 0.0, 0.0
        for i in range(1, len(positions)):
            travel += abs(positions[i][j] - positions[i - 1][j])
            excess = travel - per_m * (progress[i] - progress[0])   # signed: backwards is no progress
            # joint travel over any stretch, beyond what that stretch's progress allows
            if excess - lowest > MAX_JOINT_STEP_RAD:
                return (f"joint {j} moves more than {MAX_JOINT_STEP_RAD} rad per "
                        f"{STEP_M * 1000:.0f} mm along the line")
            lowest = min(lowest, excess)
    return None
