"""Cup press on the table: raw cap counts against press depth and force. First step towards a
CapCup calibration (S6, user 2026-11-12). One motion per call; a person at the e-stop.

    python3 tests/arm/cap_press.py start [--xy 0.45 0.0] [--above 0.04] [--square] [--roll 0]  # planned
    python3 tests/arm/cap_press.py press [--depth 0.005] [--stop-n 8] [--back]   # along the cup, logged
    python3 tests/arm/cap_press.py up                                       # straight back to --above

The table is the standing desk RAISED (table_up in the site's workcell.yaml, touched by the arm,
tilted 1.17 deg; the press uses its top's height at --xy):
lowered (table_down, 0.725) it is out of reach, the arm's shoulder is at z 1.84 and the tip gets
down to ~0.74 m fully stretched. press and up leave the table out of the scene (the press goes
into it on purpose) and are capped by distance instead: the tip (relaxed suction cup) ends
--depth below the table top, at most 6 mm (the aluminium end is 8 mm behind the tip); the 15 N
stop ends a press that meets the table early. press logs while it moves and dwells --dwell s at
the bottom, then writes
a CSV (default /tmp/cap_press_<time>.csv in the container: /repo is read-only):
t, tcp z, force change from the start (N, magnitude and z), contact, 8 raw channels, tool
orientation (x, y, z, w in base_link; channel k sits at 22.5 + 45 (k - 1) deg from the tool's x).
Speed: every move's speed is set here to be safe at slider 100 % (user 2026-11-12); the slider
only slows it down, and each NEXT line prints the real speed at the current slider.
"""
import argparse
import csv
import math
import os
import threading
import time

import numpy as np
import rclpy
from capcup_interfaces.msg import CapData
from geometry_msgs.msg import WrenchStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from tahoma_moveit_config import workcell as W
from tahoma_robot import Robot, Stop
from tahoma_robot.robot import DEFAULT_PLANNED_TIP_SPEED

import slider

WORKCELL = os.environ.get("WORKCELL", "/repo/sites/uw_lab/workcell.yaml")
WRENCH = os.environ.get("WRENCH_TOPIC", "/force_torque_sensor_broadcaster/wrench")
LIMIT_N = 50.0
STOP_N = 8.0            # press 1 (2026-11-12): 8 N came ~7.5 mm into the suction cup, 15 N at ~8.3 mm
TABLE = "table_up"
MAX_DEPTH = 0.006        # press 1 (2026-11-12): 15 N after ~7 mm, the suction cup's travel nearly used up
SCENE_PLAN = ("limits_full", TABLE, "pod_partial")
SCENE_PRESS = ("limits_full", "pod_partial")   # no table: capped by distance


def rot_of(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def align(R, axis):
    """R turned by the smallest rotation that puts its z axis on axis."""
    a, b = R[:, 2], axis / np.linalg.norm(axis)
    v, c = np.cross(a, b), float(a @ b)
    if c < -0.999:
        raise SystemExit("cup points straight up: move it roughly downwards first")
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return (np.eye(3) + K + K @ K / (1 + c)) @ R


def rolled(axis, roll_deg):
    """Rotation with z on axis and x at roll_deg from base x, seen from above."""
    z = axis / np.linalg.norm(axis)
    r = math.radians(roll_deg)
    x = np.array([math.cos(r), math.sin(r), 0.0])
    x = x - (x @ z) * z
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


class Recorder:
    """Raw cap samples with the latest contact, wrench and TCP z at each one."""

    def __init__(self, robot):
        self.robot, self.rows, self.on = robot, [], False
        self.contact, self.wrench, self.base, self.phase = None, None, None, "in"
        self.node = rclpy.create_node("cap_press_recorder")
        self.node.create_subscription(CapData, "/cap_data", self.on_cap, qos_profile_sensor_data)
        self.node.create_subscription(Bool, "/cap_contact", lambda m: setattr(self, "contact", m.data),
                                      qos_profile_sensor_data)
        self.node.create_subscription(WrenchStamped, WRENCH, self.on_wrench, qos_profile_sensor_data)
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(self.node)
        threading.Thread(target=self.ex.spin, daemon=True).start()

    def on_wrench(self, m):
        f = m.wrench.force
        self.wrench = np.array([f.x, f.y, f.z])

    def on_cap(self, m):
        if not self.on or self.wrench is None:
            return
        if self.base is None:
            self.base = self.wrench.copy()
        d = self.wrench - self.base
        p, q = self.robot.tcp_pose()
        self.rows.append([round(time.monotonic() - self.t0, 3), round(p[2], 5),
                          round(float(np.linalg.norm(d)), 2), round(float(d[2]), 2),
                          self.contact, *m.values, *(round(v, 5) for v in q), self.phase])

    def mark(self, phase):
        self.phase = phase

    def start(self):
        self.t0, self.on = time.monotonic(), True

    def save(self, path):
        self.on = False
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "tcp_z", "df_n", "dfz_n", "contact"] + [f"ch{i}" for i in range(1, 9)]
                       + ["qx", "qy", "qz", "qw", "phase"])
            w.writerows(self.rows)
        return path


def press_scene(robot):
    """Scene.use only removes what this process placed, and start ran in another process: place
    the plan scene first so the table is known here, then swap to the press scene."""
    robot.scene.use(*SCENE_PLAN)
    robot.scene.use(*SCENE_PRESS)


def to_table(robot, xy):
    """Distance from the tip to the table top along the cup (m, + = table ahead), after checking
    that the cup points down (within 5.7 deg) over the press point."""
    p, q = robot.tcp_pose()
    axis = rot_of(q)[:, 2]
    cos = -float(axis[2])
    if cos < 0.995:
        raise SystemExit(f"cup is {math.degrees(math.acos(min(1, cos))):.1f} deg off vertical: run start first")
    if math.hypot(p[0] - xy[0], p[1] - xy[1]) > 0.02:
        raise SystemExit(f"tip at x {p[0]:.3f} y {p[1]:.3f}, not over the press point: run start first")
    on_top = np.array([xy[0], xy[1], TABLE_Z])
    return float(NORMAL @ (on_top - np.array(p)) / (NORMAL @ axis))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("step", choices=["start", "press", "up"])
    ap.add_argument("--xy", type=float, nargs=2, default=[0.45, 0.0])
    ap.add_argument("--above", type=float, default=0.04)
    ap.add_argument("--square", action="store_true", help="start: cup square to the table, not straight down")
    ap.add_argument("--depth", type=float, default=0.005)
    ap.add_argument("--speed", type=float, default=0.003)
    ap.add_argument("--dwell", type=float, default=3.0)
    ap.add_argument("--stop-n", type=float, default=STOP_N, help="press: force stop (N), up to 15")
    ap.add_argument("--back", action="store_true", help="press: after the dwell, straight back to --above, logged")
    ap.add_argument("--roll", type=float, default=None,
                    help="start: tool x at this angle (deg) from base x seen from above; default keeps the current roll")
    ap.add_argument("--tip-speed", type=float, default=DEFAULT_PLANNED_TIP_SPEED,
                    help="start: planned move's tool tip cap (m/s)")
    ap.add_argument("--out", default=f"/tmp/cap_press_{time.strftime('%H%M%S')}.csv")
    a = ap.parse_args()
    if not 0.02 <= a.above <= 0.1:
        raise SystemExit("--above is 0.02 to 0.1 m")
    if not 0 <= a.depth <= MAX_DEPTH:
        raise SystemExit(f"--depth is 0 to {MAX_DEPTH} m (aluminium end 8 mm behind the tip)")
    if not 0 < a.stop_n <= 15:
        raise SystemExit("--stop-n is up to 15 N")
    if not 0 < a.speed <= 0.01:
        raise SystemExit("--speed is up to 0.01 m/s")
    table = W.load(WORKCELL)["objects"].get(TABLE)
    if table is None or table.get("frame") != "base_link":
        raise SystemExit(f"{TABLE} (frame base_link) missing in {WORKCELL}")
    global TABLE_Z, NORMAL
    T = W._matrix(table["xyz"], table.get("rpy", [0, 0, 0]))
    local = np.linalg.inv(T) @ np.r_[a.xy[0], a.xy[1], 0.0, 1.0]
    (sx, sy, sz) = table["size"]
    if abs(local[0]) > sx / 2 - 0.05 or abs(local[1]) > sy / 2 - 0.05:
        raise SystemExit(f"--xy is not at least 5 cm inside {TABLE}")
    top, n = (T @ np.r_[0, 0, sz / 2, 1])[:3], T[:3, 2]   # the top face: a point and its normal
    TABLE_Z = top[2] - (n[0] * (a.xy[0] - top[0]) + n[1] * (a.xy[1] - top[1])) / n[2]
    NORMAL = n / np.linalg.norm(n)
    tilt = math.degrees(math.acos(min(1.0, NORMAL[2])))
    print(f"{TABLE} top z {TABLE_Z:.4f} at x {a.xy[0]:.3f} y {a.xy[1]:.3f}, tilted {tilt:.2f} deg", flush=True)
    rclpy.init()
    with Robot.connect("capcup", LIMIT_N, WORKCELL) as robot:
        p, q = robot.tcp_pose()
        print(f"tip now x {p[0]:.3f} y {p[1]:.3f} z {p[2]:.3f}", flush=True)
        if a.step == "start":
            robot.scene.use(*SCENE_PLAN)
            axis = -NORMAL if a.square else np.array([0.0, 0.0, -1.0])
            R = align(rot_of(q), axis) if a.roll is None else rolled(axis, a.roll)
            quat = W._quaternion(R)
            target = tuple(np.array([a.xy[0], a.xy[1], TABLE_Z]) - a.above * axis)
            how = "square to the table" if a.square else "straight down"
            print(f"\nNEXT: planned move, tip to x {target[0]:.3f} y {target[1]:.3f} z {target[2]:.3f} "
                  f"({1000 * a.above:.0f} mm from the table along the cup), cup {how}, tip up to "
                  f"{a.tip_speed:.2f} m/s" + slider.note(robot, a.tip_speed, "m/s", 1.0), flush=True)
            # Tight tolerances: the default angle tolerance (0.02 rad) is as big as the table's tilt.
            r = robot.plan_to(target, quat, position_tolerance=0.001, angle_tolerance=0.002,
                              tip_speed=a.tip_speed)
            print(f"RESULT {r.reason}")
            _, q2 = robot.tcp_pose()
            off = math.degrees(math.acos(min(1.0, float(rot_of(q2)[:, 2] @ axis))))
            x2 = rot_of(q2)[:, 0]
            print(f"cup axis {off:.2f} deg off the target; tool x at {math.degrees(math.atan2(x2[1], x2[0])):.1f} deg "
                  f"from base x (seen from above)")
        elif a.step == "press":
            d = to_table(robot, a.xy) + a.depth
            if not 0 < d <= 0.1:
                raise SystemExit(f"would move {1000 * d:.0f} mm; press goes 0 to 100 mm")
            press_scene(robot)
            rec = Recorder(robot)
            time.sleep(0.5)
            print(f"\nNEXT: straight {1000 * d:.0f} mm along the cup, to {1000 * a.depth:.0f} mm into the "
                  f"table top, {1000 * a.speed:.0f} mm/s" + slider.note(robot, a.speed) + ", "
                  f"{a.stop_n:.0f} N stop, then {a.dwell:.0f} s still"
                  + (f", then straight back {1000 * (a.above + a.depth):.0f} mm, logged" if a.back else ""),
                  flush=True)
            rec.start()
            r = robot.move_straight((0, 0, d), speed=a.speed, stop=Stop(force_n=a.stop_n))
            print(f"RESULT {r.reason} peak {r.peak_force_n:.1f} N", flush=True)
            rec.mark("hold")
            time.sleep(a.dwell)
            if a.back:
                rec.mark("out")
                back = a.above - to_table(robot, a.xy)
                print(f"RESULT back {robot.move_straight((0, 0, -back), speed=a.speed).reason}", flush=True)
                time.sleep(1.0)
            print(f"CSV {rec.save(a.out)} ({len(rec.rows)} samples)")
        else:
            d = a.above - to_table(robot, a.xy)
            if not 0 < d <= 0.1:
                raise SystemExit(f"would move {1000 * d:.0f} mm; up goes 0 to 100 mm")
            press_scene(robot)
            print(f"\nNEXT: straight back {1000 * d:.0f} mm along the cup, 10 mm/s"
                  + slider.note(robot, 0.01), flush=True)
            print(f"RESULT {robot.move_straight((0, 0, -d), speed=0.01).reason}")
        p, _ = robot.tcp_pose()
        print(f"tip x {p[0]:.3f} y {p[1]:.3f} z {p[2]:.3f}")


if __name__ == "__main__":
    main()
