"""Fake cell for the planner node: a kinematic arm + still cubes from a scenario YAML, over ROS 2. No robot, no cameras.

    python fake_cell.py --scenario <scenario.yaml> [--exit-after N] [--timeout S] [--report out.json]

Publishes /joint_states (arm_* names, 100 Hz; q integrates the latest /mpc/joint_velocity_cmd, zero if none for
0.5 s) and /mpc/object_poses (10 Hz, frame mpc_world: bin frame + (x0, y0, 1.225), then the back tracker at the
position the planner's conversion expects and a synthetic side tracker). --exit-after N: after N commands the
target is moved out past the exit line, so the node should finish ("done"). Prints the node's status changes and
exits 0 when the node reports done (or failed with --expect-failed), 1 otherwise.
Runs on any Jazzy rclpy: the planner image's bundled one or the base image.
"""
import argparse
import json
import math
import sys
import threading
import time

import yaml

ap = argparse.ArgumentParser()
ap.add_argument("--scenario", required=True)
ap.add_argument("--exit-after", type=int, default=0)
ap.add_argument("--timeout", type=float, default=900.0)
ap.add_argument("--report", default=None)
ap.add_argument("--expect-failed", action="store_true")
ap.add_argument("--nan-object", type=int, default=-1, help="publish this object index as NaN (missing)")
args, ros_args = ap.parse_known_args()

import rclpy  # noqa: E402
from geometry_msgs.msg import Pose, PoseArray  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import String  # noqa: E402

JOINTS = [f"arm_{n}_joint" for n in ("shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3")]
Q0 = [1.489, -1.792, 2.0798, -1.669, 1.474, 0.0892]   # the planner's config.yaml robot_init_joints

sc = yaml.safe_load(open(args.scenario))
bs = sc["bin_size"]
x0, y0 = sc["bin_center"][0] - bs / 2, sc["bin_center"][1] - bs / 2
ist = sc["initial_state"]
objs = [(ist["target_pos"], ist["target_quat"])] + [(o["pos"], o["quat"]) for o in ist["obstacles"]]
world = [[p[0] + x0, p[1] + y0, p[2] + 1.225] + list(q) for p, q in objs]
world.append([x0 + bs, y0 + bs / 2, 1.26, 1.0, 0.0, 0.0, 0.0])   # back tracker: scene_from_tracked_poses inverse
world.append([x0 + bs / 2, y0 - 0.03, 1.26, 1.0, 0.0, 0.0, 0.0])  # side tracker (synthetic)

rclpy.init(args=["fake_cell", *ros_args])
node = rclpy.create_node("bin_clearing_fake_cell")
js_pub = node.create_publisher(JointState, "/joint_states", 10)
pose_pub = node.create_publisher(PoseArray, "/mpc/object_poses", 10)
lock = threading.Lock()
st = {"q": list(Q0), "u": [0.0] * 6, "u_time": 0.0, "n": 0, "cmd_times": [], "max_abs_u": 0.0,
      "names_ok": True, "phases": [], "last": None, "target_moved": False}


def on_cmd(msg):
    with lock:
        if list(msg.name) != JOINTS:
            st["names_ok"] = False
        st["u"], st["u_time"] = list(msg.velocity), time.monotonic()
        st["n"] += 1
        st["cmd_times"].append(time.monotonic())
        st["max_abs_u"] = max([st["max_abs_u"]] + [abs(v) for v in msg.velocity])
        if args.exit_after and st["n"] >= args.exit_after and not st["target_moved"]:
            world[0][0] = x0 - 0.05   # target past the exit line (planner exit_x = x0)
            st["target_moved"] = True
            print(f"[fake_cell] moved the target out of the bin after {st['n']} commands", flush=True)


def on_status(msg):
    s = json.loads(msg.data)
    with lock:
        st["last"] = s
        if not st["phases"] or st["phases"][-1] != s["phase"]:
            st["phases"].append(s["phase"])
            print(f"[fake_cell] node phase -> {s['phase']}: {s.get('message', '')}", flush=True)


node.create_subscription(JointState, "/mpc/joint_velocity_cmd", on_cmd, 10)
node.create_subscription(String, "/mpc/status", on_status, 10)


def tick_arm():
    with lock:
        u = st["u"] if time.monotonic() - st["u_time"] < 0.5 else [0.0] * 6
        st["q"] = [q + v * 0.01 for q, v in zip(st["q"], u)]
        msg = JointState(name=JOINTS, position=st["q"], velocity=u)
    msg.header.stamp = node.get_clock().now().to_msg()
    js_pub.publish(msg)


def tick_poses():
    msg = PoseArray()
    msg.header.frame_id = "mpc_world"
    msg.header.stamp = node.get_clock().now().to_msg()
    for i, w in enumerate(world):
        p = Pose()
        if i == args.nan_object:
            w = [math.nan] * 7
        p.position.x, p.position.y, p.position.z = w[0], w[1], w[2]
        p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z = w[3], w[4], w[5], w[6]
        msg.poses.append(p)
    pose_pub.publish(msg)


node.create_timer(0.01, tick_arm)
node.create_timer(0.1, tick_poses)
t_end = time.monotonic() + args.timeout
want = "failed" if args.expect_failed else "done"
while rclpy.ok() and time.monotonic() < t_end:
    rclpy.spin_once(node, timeout_sec=0.05)
    with lock:
        if st["phases"] and st["phases"][-1] in ("done", "failed", "stopped"):
            break
time.sleep(0.2)
with lock:
    ct = st["cmd_times"]
    gaps = [b - a for a, b in zip(ct, ct[1:])]
    report = {"phases": st["phases"], "last_status": st["last"], "commands": st["n"], "names_ok": st["names_ok"],
              "max_abs_u": st["max_abs_u"], "median_gap_s": sorted(gaps)[len(gaps) // 2] if gaps else None,
              "max_gap_s": max(gaps) if gaps else None, "target_moved": st["target_moved"]}
print("[fake_cell] REPORT " + json.dumps(report), flush=True)
if args.report:
    json.dump(report, open(args.report, "w"), indent=1)
ok = bool(st["phases"]) and st["phases"][-1] == want and st["names_ok"]
print(f"[fake_cell] {'PASS' if ok else 'FAIL'} (wanted final phase '{want}')", flush=True)
sys.exit(0 if ok else 1)
