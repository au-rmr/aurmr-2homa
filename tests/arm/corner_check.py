"""Pod calibration check by eye: the tool stops square to the face, in front of a corner where four
bins meet (docs/procedures/pod_calibration.md). One motion per call; a person at the e-stop.

    python3 tests/arm/corner_check.py start          # planned move to pre_bin_3g
    python3 tests/arm/corner_check.py corner [--bin 2g] [--gap 0.05]
    python3 tests/arm/corner_check.py in [--final 0.01]   # straight along the cup, 10 N stop
    python3 tests/arm/corner_check.py out                 # straight back to --gap
    python3 tests/arm/corner_check.py back [--pose drop_hide]   # planned move to pre_bin_3g (or --pose)
Planned moves stay at --gap (5 cm); only in/out go closer, straight at 1 cm/s with a force stop.

corner: the TCP goes to the top right corner of --bin (2g: where 2g, 3g, 2h, 3h meet), --gap in
front of the pod face, cup axis square to the face, roll kept from the current pose. Prints the
target and the reached pose in the pod frame.
"""
import argparse
import os

import numpy as np
from tahoma_moveit_config import workcell as W
from tahoma_robot import Robot, Stop

import slider

WORKCELL = os.environ.get("WORKCELL", "/repo/sites/uw_lab/workcell.yaml")
LIMIT_N = 50.0
START = "pre_bin_3g"


def rot_of(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def align(R, axis):
    """R turned by the smallest rotation that puts its z axis on axis."""
    a, b = R[:, 2], axis / np.linalg.norm(axis)
    v, c = np.cross(a, b), float(a @ b)
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return (np.eye(3) + K + K @ K / (1 + c)) @ R


def main():
    p = argparse.ArgumentParser()
    p.add_argument("step", choices=["start", "corner", "in", "out", "back"])
    p.add_argument("--bin", default="2g")
    p.add_argument("--gap", type=float, default=0.05)
    p.add_argument("--final", type=float, default=0.01)
    p.add_argument("--pose", default=START, help="back: named pose to go to")
    p.add_argument("--tip-speed", type=float, default=0.1)   # planned moves end 5 cm from the pod: tip cap 0.1 m/s, safe at slider 100 % (user 2026-11-12)
    args = p.parse_args()
    if not 0.02 <= args.gap <= 0.3:
        raise SystemExit("--gap is 0.02 to 0.3 m")
    if not 0.005 <= args.final < args.gap:
        raise SystemExit("--final is 0.005 m up to --gap")
    wc = W.load(WORKCELL)
    pod = wc["frames"]["pod_base_link"]
    T = W._matrix(pod["xyz"], pod["rpy"])
    with Robot.connect("capcup", LIMIT_N, WORKCELL) as robot:
        robot.scene.use("limits_full", "table_down", "pod_bins")
        if args.step in ("start", "back"):
            pose = args.pose if args.step == "back" else START
            if pose != START:   # the cap_cup demo's hide scene (sites/uw_lab/demos/cap_cup.yaml) + the bins
                robot.scene.use("limits_drop_hide", "table_down", "pod_partial", "pod_bins")
            print(f"\nNEXT: planned move to {pose}, tip up to {args.tip_speed:.2f} m/s"
                  + slider.note(robot, args.tip_speed, "m/s", 1.0), flush=True)
            print(f"RESULT {robot.go_named(pose, tip_speed=args.tip_speed).reason}")
        elif args.step in ("in", "out"):
            p0, q0 = robot.tcp_pose()
            gap_now = -(np.linalg.inv(T) @ np.r_[p0, 1])[1]
            cos = float(T[:3, 1] @ rot_of(q0)[:, 2])
            if cos < 0.995:
                raise SystemExit(f"cup is {np.degrees(np.arccos(cos)):.1f} deg off square: run corner first")
            want = args.final if args.step == "in" else args.gap
            d = (gap_now - want) / cos
            if abs(d) > 0.06:
                raise SystemExit(f"would move {1000 * d:.0f} mm; in/out only go up to 60 mm (gap now {1000 * gap_now:.0f} mm)")
            print(f"\nNEXT: straight {1000 * d:+.0f} mm along the cup (gap {1000 * gap_now:.0f} -> "
                  f"{1000 * want:.0f} mm), 10 mm/s" + slider.note(robot, 0.01) + ", 10 N stop", flush=True)
            print(f"RESULT {robot.move_straight((0, 0, d), speed=0.01, stop=Stop(force_n=10)).reason}")
        else:
            lo, hi = W.bin_box(wc, "pod_bins", args.bin)
            corner_pod = np.array([hi[0], -args.gap, hi[2]])
            xyz = (T @ np.r_[corner_pod, 1])[:3]
            _, q = robot.tcp_pose()
            R = align(rot_of(q), T[:3, 1])
            quat = W._quaternion(R)
            print(f"\nNEXT: planned move, TCP {1000 * args.gap:.0f} mm in front of the top right "
                  f"corner of {args.bin} (pod x {hi[0]:.3f}, z {hi[2]:.3f}; base_link "
                  f"{xyz[0]:.3f} {xyz[1]:.3f} {xyz[2]:.3f}), tip up to {args.tip_speed:.2f} m/s"
                  + slider.note(robot, args.tip_speed, "m/s", 1.0), flush=True)
            print(f"RESULT {robot.plan_to(xyz, quat, tip_speed=args.tip_speed).reason}")
        p_, q_ = robot.tcp_pose()
        local = np.linalg.inv(T) @ np.r_[p_, 1]
        axis = T[:3, :3].T @ rot_of(q_)[:, 2]
        print(f"REACHED pod x {local[0]:.3f} y {local[1]:+.3f} z {local[2]:.3f}, cup axis in pod "
              f"{axis[0]:+.3f} {axis[1]:+.3f} {axis[2]:+.3f}")


if __name__ == "__main__":
    main()
