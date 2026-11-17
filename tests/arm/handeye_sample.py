"""Records the flange pose for one camera-calibration sample (eye-to-hand, tag on the tool).

Read-only: listens to TF, sends nothing. Freedrive the arm (program not playing) so the tag on the tool faces the
camera, hold still, then run:
    python3 tests/arm/handeye_sample.py <out_dir> <pose_name>
It averages base_link -> each arm link (FRAMES) over 1 s and writes <out_dir>/<pose_name>_tf.json (per link:
translation m, quaternion xyzw, spread mm). Grab the camera frame for the same pose right after (tests/perception/capture_frames.py
<out_dir>/<pose_name>.npz, on the camera's domain). Solve with tests/perception/handeye_solve.py.
"""
import json
import math
import os
import sys
import time

import rclpy
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

BASE = "base_link"
# Every link a calibration tag could sit on; the solver picks, per tag, the link it is rigid with.
FRAMES = os.environ.get("HANDEYE_FRAMES", "arm_tool0 arm_wrist_3_link arm_wrist_2_link arm_wrist_1_link arm_forearm_link "
                        "arm_upper_arm_link arm_shoulder_link").split()


def main():
    out_dir, name = sys.argv[1], sys.argv[2]
    rclpy.init()
    node = rclpy.create_node("handeye_sample")
    tf = Buffer()
    TransformListener(tf, node)
    joints = {}
    node.create_subscription(JointState, "/joint_states", lambda m: joints.update(zip(m.name, m.position)), 10)
    seen, end = {f: [] for f in FRAMES}, time.monotonic() + 3.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
        if time.monotonic() < end - 1.0:
            continue
        for f in FRAMES:
            if tf.can_transform(BASE, f, rclpy.time.Time(), Duration(seconds=0)):
                t = tf.lookup_transform(BASE, f, rclpy.time.Time()).transform
                seen[f].append(((t.translation.x, t.translation.y, t.translation.z),
                                (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)))
    node.destroy_node()
    rclpy.shutdown()
    if len(seen[FRAMES[0]]) < 5:
        sys.exit(f"no TF {BASE} -> {FRAMES[0]}: is the robot stack up on this domain?")
    out = {"base": BASE, "frames": {}, "joints": dict(sorted(joints.items()))}  # replayable pose list
    for f, s in seen.items():
        if len(s) < 5:
            continue
        xyz = [sum(p[0][i] for p in s) / len(s) for i in range(3)]
        q0 = s[-1][1]
        q = [sum(p[1][i] * (1 if sum(a * b for a, b in zip(p[1], q0)) >= 0 else -1) for p in s) for i in range(4)]
        n = math.sqrt(sum(c * c for c in q))
        out["frames"][f] = {"xyz": xyz, "quat_xyzw": [c / n for c in q],
                            "spread_mm": 1000 * max(math.dist(p[0], xyz) for p in s), "samples": len(s)}
    spread = max(v["spread_mm"] for v in out["frames"].values())
    out["spread_mm"] = spread
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, f"{name}_tf.json"), "w") as fh:
        json.dump(out, fh, indent=1)
    flag = "  MOVED: hold still and redo" if spread > 0.5 else ""
    tool = out["frames"][FRAMES[0]]["xyz"]
    print(f"{name}: {FRAMES[0]} at {[round(v, 4) for v in tool]} m, {len(out['frames'])} links, "
          f"max spread {spread:.2f} mm{flag}")


if __name__ == "__main__":
    main()
