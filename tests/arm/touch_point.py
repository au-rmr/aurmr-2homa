"""Records where the tool tip is, for the pod calibration (docs/procedures/pod_calibration.md).

Read-only: listens to TF, sends nothing. Put the tool (in freedrive, program not playing, suction cup removed) with the
cup's aluminium end pressed flush on a surface, then run:
    python3 tests/arm/touch_point.py <feature> [points.csv]
It averages the tip pose over 1 s and appends: feature, x, y, z (base_link, m), tool axis x, y, z,
spread (mm). Features: see tests/arm/fit_pod.py FEATURES.
"""
import math
import os
import sys
import time

import rclpy
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener

# Probe point: the cup body's rigid aluminium end (suction cup removed). TOUCH_FRAME overrides.
BASE, TIP = "base_link", os.environ.get("TOUCH_FRAME", "cup_aluminium_end")


def main():
    feature = sys.argv[1]
    path = sys.argv[2] if len(sys.argv) > 2 else "points.csv"
    rclpy.init()
    node = rclpy.create_node("touch_point")
    tf = Buffer()
    TransformListener(tf, node)
    seen, end = [], time.monotonic() + 3.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
        if tf.can_transform(BASE, TIP, rclpy.time.Time(), Duration(seconds=0)) and \
                time.monotonic() > end - 1.0:
            t = tf.lookup_transform(BASE, TIP, rclpy.time.Time()).transform
            q = t.rotation
            axis = (2 * (q.x * q.z + q.w * q.y), 2 * (q.y * q.z - q.w * q.x),
                    1 - 2 * (q.x * q.x + q.y * q.y))   # the tool's z axis (out of the cup)
            seen.append(((t.translation.x, t.translation.y, t.translation.z), axis))
    node.destroy_node()
    rclpy.shutdown()
    if len(seen) < 10:
        sys.exit(f"only {len(seen)} tip poses in 1 s: is the robot stack running?")
    mean = [sum(p[i] for p, _ in seen) / len(seen) for i in range(3)]
    axis = [sum(a[i] for _, a in seen) / len(seen) for i in range(3)]
    spread = max(math.dist(p, mean) for p, _ in seen) * 1000
    row = [feature] + [f"{v:.5f}" for v in mean + axis] + [f"{spread:.2f}"]
    with open(path, "a") as f:
        f.write(",".join(row) + "\n")
    print(f"RESULT {feature}: tip {mean[0]:.4f} {mean[1]:.4f} {mean[2]:.4f} m, tool axis "
          f"{axis[0]:.3f} {axis[1]:.3f} {axis[2]:.3f}, spread {spread:.2f} mm -> {path}")


if __name__ == "__main__":
    main()
