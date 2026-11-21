"""Where each calibrated camera puts one AprilTag, in base_link: the camera half of the touch check.

    python3 tests/perception/tag_touch_check.py <tag_id> <camera1.json> [<camera2.json> ...] [--seconds 2]

Each json = tests/perception/handeye_solve.py --out (parent base_link, child <camera>_color_optical_frame). Reads the
tag's TF from apriltag_ros (<camera>_color_optical_frame -> tag<family>:<id>, published per detection) on the camera
domain, averages it, and prints the tag centre in base_link per camera and their spread. Then touch the tag's centre
with the flange (tests/arm/touch_point.py with TOUCH_FRAME=arm_tool0) and compare. Read-only.
"""
import argparse
import json
import time

import numpy as np
import rclpy
from tf2_ros import Buffer, TransformListener


def quat_to_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def main():
    p = argparse.ArgumentParser()
    p.add_argument("tag", type=int)
    p.add_argument("cals", nargs="+")
    p.add_argument("--family", default="tagStandard41h12")
    p.add_argument("--seconds", type=float, default=2.0)
    a = p.parse_args()
    cals = [json.load(open(c)) for c in a.cals]
    rclpy.init()
    node = rclpy.create_node("tag_touch_check")
    tf = Buffer()
    TransformListener(tf, node)
    child = f"{a.family}:{a.tag}"
    seen = {c["child"]: [] for c in cals}
    last = {}
    end = time.monotonic() + a.seconds + 2.0   # discovery + TF buffer fill
    start = time.monotonic() + 2.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
        if time.monotonic() < start:
            continue
        for frame in seen:
            try:
                t = tf.lookup_transform(frame, child, rclpy.time.Time())
            except Exception:
                continue
            stamp = (t.header.stamp.sec, t.header.stamp.nanosec)
            if last.get(frame) != stamp and time.time() - (t.header.stamp.sec + 1e-9 * t.header.stamp.nanosec) < 0.5:
                last[frame] = stamp
                v = t.transform.translation
                seen[frame].append([v.x, v.y, v.z])
    points = {}
    for c in cals:
        obs = np.array(seen[c["child"]])
        if len(obs) == 0:
            print(f"{c['child']}: tag {a.tag} not seen")
            continue
        cam = np.mean(obs, axis=0)
        base = quat_to_R(c["quat_xyzw"]) @ cam + np.array(c["xyz"])
        points[c["child"]] = base
        print(f"{c['child']}: tag {a.tag} at {np.round(base, 4)} m in base_link "
              f"({len(obs)} detections, spread {1000 * np.max(np.linalg.norm(obs - cam, axis=1)):.1f} mm, "
              f"range {np.linalg.norm(cam):.2f} m)")
    if len(points) > 1:
        v = list(points.values())
        print(f"cameras disagree by {1000 * np.linalg.norm(v[0] - v[1]):.1f} mm "
              f"(per axis {np.round(1000 * (v[0] - v[1]), 1)} mm)")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
