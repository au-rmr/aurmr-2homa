"""Fits pod_base_link from touched points (tests/arm/touch_point.py). Plain numpy, no ROS.

    python3 tests/arm/fit_pod.py [--floor] points.csv [workcell.yaml]

--floor: no base_top touch; the pod stands on the floor, which is z = 0 in base_link (user
2026-11-10: the arm can't reach the base top). Pins the pod frame's origin to z = 0.

pod_base_link: origin at the pod's front bottom left corner seen from the arm, x across the face
towards column 4, y into the pod, z up. Each feature is a flat rigid surface whose position in the
pod frame is known; with the cup's end flush on it, the tip lies on that surface.
The face points fix the face plane (distance, yaw, lean). x is assumed horizontal (no tilt across
the face). Side and top points then fix the origin across and up.
"""
import csv
import math
import sys

import numpy as np

# feature prefix: (pod axis 0/1/2, value in the pod frame (m), expected tool axis in the pod frame)
# Measured on the lab's pod 2026-11-10 (user, tape): 930 mm outer width, 30 mm L-channel uprights
# (left centred on x = 0, right on 0.900). base_top 0.255 is unreachable by the arm (use --floor).
FEATURES = {
    "face": (1, 0.0, (0, 1, 0)),             # front face of an upright (left or right)
    "left_side": (0, -0.015, (1, 0, 0)),    # outer side of the left upright, cup pointing right
    "right_side": (0, 0.915, (-1, 0, 0)),   # outer side of the right upright, cup pointing left
    #   (tape 2026-11-10: 930 mm outer width, uprights 30 mm, centres 0.900 apart; old boxes said 0.9563)
    "base_top": (2, 0.255, (0, 0, -1)),      # top of the pod base, cup pointing down (tape 2026-11-10; pod.xacro 0.265)
    # Cup pointing into the pod with the SIDE of its aluminium end (38.1 mm diameter, user 2026-11-10)
    # flush on an upright's outer side: the tip frame is on the cup's axis, one radius outside it.
    "left_side_cyl": (0, -0.015 - 0.01905, (0, 1, 0)),
    "right_side_cyl": (0, 0.915 + 0.01905, (0, 1, 0)),
}


def rpy_of(R):
    """Inverse of workcell._matrix: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    return (math.atan2(R[2, 1], R[2, 2]), math.asin(-R[2, 0]), math.atan2(R[1, 0], R[0, 0]))


def matrix_of(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def read(path):
    out = []
    for row in csv.reader(open(path)):
        if not row or row[0].startswith("#"):
            continue
        kind = max((k for k in FEATURES if row[0] == k or row[0].startswith(k + "_")), key=len, default=None)
        if kind is None:
            sys.exit(f"unknown feature {row[0]}: names start with one of {list(FEATURES)}")
        out.append((row[0], kind, np.array([float(v) for v in row[1:4]]),
                    np.array([float(v) for v in row[4:7]])))
    return out


def fit(points, floor=False):
    face = np.array([p for _, k, p, _ in points if k == "face"])
    if len(face) < 3:
        sys.exit("need at least 3 face points (best: low and high on both uprights)")
    centre = face.mean(axis=0)
    normal = np.linalg.svd(face - centre)[2][2]
    into = np.mean([a for _, k, _, a in points if k == "face"], axis=0)
    y = normal if normal @ into > 0 else -normal          # into the pod
    x = np.cross(y, [0, 0, 1.0])
    x /= np.linalg.norm(x)                                 # horizontal, across the face
    z = np.cross(x, y)
    R = np.column_stack([x, y, z])
    rows, rhs = [R[:, 1]], [R[:, 1] @ centre]              # face plane: y_pod(p - t) = 0
    for _, kind, p, _ in points:
        if kind != "face":
            i, value, _ = FEATURES[kind]
            rows.append(R[:, i])
            rhs.append(R[:, i] @ p - value)
    if floor:
        rows.append(np.array([0, 0, 1.0]))
        rhs.append(0.0)
    if not any("side" in k for _, k, _, _ in points) or \
            not (floor or any(k == "base_top" for _, k, _, _ in points)):
        sys.exit("need at least one side point, and a base_top point or --floor, to place the origin")
    t = np.linalg.lstsq(np.array(rows), np.array(rhs), rcond=None)[0]
    return R, t


def main():
    floor = "--floor" in sys.argv
    args = [a for a in sys.argv[1:] if a != "--floor"]
    points = read(args[0])
    R, t = fit(points, floor)
    roll, pitch, yaw = rpy_of(R)
    print(f"FIT pod_base_link: {{parent: base_link, xyz: [{t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}], "
          f"rpy: [{roll:.4f}, {pitch:.4f}, {yaw:.4f}]}}")
    print(f"    lean of the face {math.degrees(roll):.2f} deg (2026-11-10: 0.74, top towards the arm; bubble level agrees)")
    for name, kind, p, a in points:
        i, value, want = FEATURES[kind]
        local = R.T @ (p - t)
        tilt = math.degrees(math.acos(min(1.0, float((R.T @ a) @ np.array(want)))))
        print(f"  {name:16s} off its surface {1000 * (local[i] - value):+6.1f} mm, cup {tilt:4.1f} deg "
              f"from flush, at pod x {local[0]:.3f} y {local[1]:.3f} z {local[2]:.3f}")
    if len(args) > 1:
        import yaml
        old = yaml.safe_load(open(args[1]))["frames"]["pod_base_link"]
        R0, t0 = matrix_of(old["rpy"]), np.array(old["xyz"])
        print(f"OLD pod_base_link: xyz {old['xyz']}, rpy {old['rpy']}")
        for name, corner in (("bottom front left", (0, 0, 0.35)), ("top front left", (0, 0, 2.3)),
                             ("bottom front right", (0.94, 0, 0.35)), ("top front right", (0.94, 0, 2.3))):
            c = np.array(corner)
            moved = (R @ c + t) - (R0 @ c + t0)
            print(f"  {name:20s} moves {1000 * np.linalg.norm(moved):5.0f} mm "
                  f"(x {1000 * moved[0]:+.0f}, y {1000 * moved[1]:+.0f}, z {1000 * moved[2]:+.0f})")


if __name__ == "__main__":
    main()
