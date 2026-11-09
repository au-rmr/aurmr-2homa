"""S4 steps on the real arm, one per call. See docs/procedures/s4_first_arm.md.

Usage: python3 tests/arm/s4_steps.py <step>
No prompts: the procedure's rule (a person at the e-stop, area clear, slider low) is checked once
by the people running the session, before the first motion.
Results print as RESULT lines, to copy into the S4 ledger.
"""
import argparse
import math
import os
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tahoma_robot import ForceLimitExceeded, Robot, Stop

WORKCELL = os.environ.get("WORKCELL", "/repo/sites/uw_lab/workcell.yaml")
LIMIT_N = 50.0          # the cap_cup session limit
START = "pre_bin_3g"    # clear straight lines in every direction
WRIST_DEG = 5.0


def announce(what):
    print(f"\nNEXT: {what}", flush=True)


def rates(seconds=5.0):
    """Message rates and force noise, arm idle."""
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("s4_rates")
    seen = {"joint_states": [], "wrench": [], "speed_scaling": []}
    node.create_subscription(JointState, "/joint_states", lambda m: seen["joint_states"].append(m), 100)
    node.create_subscription(WrenchStamped, "/force_torque_sensor_broadcaster/wrench",
                             lambda m: seen["wrench"].append(m), 100)
    node.create_subscription(Float64, "/speed_scaling_state_broadcaster/speed_scaling",
                             lambda m: seen["speed_scaling"].append(m), 100)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.01)
    for name, msgs in seen.items():
        print(f"RESULT rate {name} {len(msgs) / seconds:.0f} Hz")
    forces = [m.wrench.force for m in seen["wrench"]]
    if forces:
        size = [math.sqrt(f.x ** 2 + f.y ** 2 + f.z ** 2) for f in forces]
        mean = sum(size) / len(size)
        spread = math.sqrt(sum((s - mean) ** 2 for s in size) / len(size))
        print(f"RESULT force at rest {mean:.1f} N, noise (1 sd) {spread:.2f} N, "
              f"min {min(size):.1f} max {max(size):.1f}")
    if seen["speed_scaling"]:
        print(f"RESULT speed scaling {seen['speed_scaling'][-1].data:.0f} %")
    node.destroy_node()


def watch_stop(robot):
    """How far and how long the arm kept moving after a stop."""
    start, t0 = robot.joints(), time.monotonic()
    last, last_change = start, t0
    while time.monotonic() - t0 < 1.5:
        time.sleep(0.005)
        now = robot.joints()
        if max(abs(now[k] - last[k]) for k in now) > 1e-5:
            last_change = time.monotonic()
        last = now
    far = max(abs(last[k] - start[k]) for k in start)
    return far, last_change - t0


def set_payload(args):
    """Sets the tool's mass and centre of gravity on the arm, from tool.yaml unless given."""
    import yaml
    from ament_index_python.packages import get_package_share_directory
    from ur_msgs.srv import SetPayload
    with open(os.path.join(get_package_share_directory("capcup"), "tool.yaml")) as f:
        tool = yaml.safe_load(f)
    mass = args.mass if args.mass is not None else tool.get("mass_kg")
    cog = args.cog if args.cog is not None else tool.get("cog_m")
    if mass is None or cog is None:
        sys.exit("tool.yaml has no mass_kg or cog_m yet: weigh the tool first, nothing was set")
    if not 0 < mass < 10 or len(cog) != 3 or max(abs(c) for c in cog) > 0.5:
        sys.exit(f"implausible payload {mass} kg at {cog}, nothing was set")
    # The driver's services are on the control host's private domain: run this step there
    # (scripts/private_shell.sh), not from the compute host.
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    context = Context()
    rclpy.init(context=context, domain_id=int(os.environ.get("AURMR_PRIVATE_DOMAIN", "142")))
    node = rclpy.create_node("s4_payload", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    client = node.create_client(SetPayload, "/io_and_status_controller/set_payload")
    if not client.wait_for_service(timeout_sec=10):
        sys.exit("set_payload service not available: run this step on the control host, "
                 "in scripts/private_shell.sh")
    request = SetPayload.Request(mass=float(mass))
    request.center_of_gravity.x, request.center_of_gravity.y, request.center_of_gravity.z = map(float, cog)
    future = client.call_async(request)
    executor.spin_until_future_complete(future, timeout_sec=10)
    ok = future.done() and future.result().success
    print(f"RESULT payload {mass} kg at {cog}: {'set' if ok else 'FAILED'}")
    node.destroy_node()
    if not ok:
        sys.exit(1)


def step(args):
    s = args.step
    if s == "check":
        rates()
        print("CHECK BY HAND: joints match the tablet; launch log has no 'calibration parameters' "
              "error and no controller overrun warnings over 10 minutes.")
        return
    if s == "force":
        print("Press on the tool by hand now, then let go. Reading for 10 s.")
        rates(10.0)
        return
    if s == "payload":
        set_payload(args)
        return
    with Robot.connect("capcup", LIMIT_N, WORKCELL) as robot:
        robot.scene.use("limits_full", "table_down")
        if s == "wrist":
            j = robot.joints()
            target = dict(j)
            target["arm_wrist_3_joint"] += math.radians(WRIST_DEG)
            announce(f"wrist 3 turns {WRIST_DEG} degrees and back, slowly")
            print(f"RESULT {robot.go_joints(target, speed=0.05).reason}")
            print(f"RESULT {robot.go_joints(j, speed=0.05).reason}")
        elif s == "named":
            announce(f"planned move to {START} at 10% speed")
            print(f"RESULT {robot.go_named(START, speed=0.1).reason}")
        elif s == "straight":
            announce("tool moves 2 cm along its own axis and back, 0.01 m/s")
            print(f"RESULT {robot.move_straight((0, 0, 0.02), speed=0.01).reason}")
            print(f"RESULT {robot.move_straight((0, 0, -0.02), speed=0.01).reason}")
        elif s == "force_stop":
            announce("tool moves 5 cm at 0.01 m/s; press it with a soft object, it must stop "
                     "at 10 N, then it moves back 1 cm: let go after the stop")
            r = robot.move_straight((0, 0, 0.05), speed=0.01, stop=Stop(force_n=10))
            far, took = watch_stop(robot)
            print(f"RESULT reason {r.reason}, peak {r.peak_force_n:.1f} N, kept moving "
                  f"{far:.5f} rad for {took:.3f} s")
            robot.move_straight((0, 0, -0.01), speed=0.01)
        elif s == "limit":
            announce(f"tool moves 5 cm at 0.01 m/s; press past {LIMIT_N} N: it must raise")
            try:
                r = robot.move_straight((0, 0, 0.05), speed=0.01)
                print(f"RESULT no limit reached: reason {r.reason}, peak {r.peak_force_n:.1f} N")
            except ForceLimitExceeded as e:
                far, took = watch_stop(robot)
                print(f"RESULT ForceLimitExceeded ({e}), kept moving {far:.5f} rad for {took:.3f} s")
        elif s == "stop":
            announce("tool moves 5 cm at 0.02 m/s and is stopped by software after 1 s")
            threading.Timer(1.0, robot.stop).start()
            r = robot.move_straight((0, 0, 0.05), speed=0.02)
            far, took = watch_stop(robot)
            print(f"RESULT reason {r.reason}, kept moving {far:.5f} rad for {took:.3f} s")
        else:
            sys.exit(f"unknown step {s}")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("step", choices=["check", "force", "wrist", "named", "straight", "force_stop",
                                    "limit", "stop", "payload"])
    p.add_argument("--mass", type=float, help="payload: kg, instead of tool.yaml")
    p.add_argument("--cog", type=float, nargs=3, help="payload: x y z in m, instead of tool.yaml")
    step(p.parse_args())
