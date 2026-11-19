"""Smoke test for the planner image: GPU, Isaac Lab (headless or with the viewer), bundled ROS 2.

    python docker/planner/smoke_test.py [--gui] [--topic /joint_states] [--seconds 5]

1. torch sees the GPU.
2. Isaac Lab starts, steps a ground plane + cube 120 times, closes (--gui: opens the viewer).
3. Isaac Sim's bundled ROS 2 Jazzy rclpy (Cyclone, the host's ROBOT_LAN_IP) receives a topic from another
   host on the robot network. Exit code 0 only if all pass.
"""
import argparse
import os
import sys
import time

parser = argparse.ArgumentParser()
parser.add_argument("--gui", action="store_true")
parser.add_argument("--topic", default="/joint_states")
parser.add_argument("--seconds", type=float, default=5.0)
args = parser.parse_args()
failures = []

# 1. GPU
import torch  # noqa: E402

if torch.cuda.is_available():
    print(f"[1] GPU ok: {torch.cuda.get_device_name(0)}, torch {torch.__version__}, CUDA {torch.version.cuda}")
    x = torch.randn(2048, 2048, device="cuda")
    assert torch.isfinite((x @ x).sum()).item()
else:
    failures.append("torch sees no GPU")

# 3 (run before Isaac starts). ROS 2: Kit loads its own copy of the ROS libraries only if its bridge extension is enabled,
# which the planner doesn't do; the bundled rclpy works in the plain process.
import rclpy  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402

rclpy.init()
node = rclpy.create_node("planner_smoke_test")
count = {"n": 0}
node.create_subscription(JointState, args.topic, lambda _: count.__setitem__("n", count["n"] + 1),
                         qos_profile_sensor_data)
end = time.monotonic() + args.seconds
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.05)
rate = count["n"] / args.seconds
print(f"[3] ROS 2: {count['n']} messages on {args.topic} in {args.seconds:.0f} s ({rate:.0f} Hz), "
      f"domain {os.environ.get('ROS_DOMAIN_ID')}, interface {os.environ.get('ROBOT_LAN_IP')}")
if count["n"] == 0:
    failures.append(f"no messages on {args.topic}")
node.destroy_node()
rclpy.shutdown()

# 2. Isaac Lab
from isaaclab.app import AppLauncher  # noqa: E402

app = AppLauncher(headless=not args.gui).app
import isaaclab.sim as sim_utils  # noqa: E402

sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / 60, device="cuda:0"))
sim_utils.GroundPlaneCfg().func("/World/ground", sim_utils.GroundPlaneCfg())
cube = sim_utils.CuboidCfg(size=(0.05, 0.05, 0.05), rigid_props=sim_utils.RigidBodyPropertiesCfg(),
                           mass_props=sim_utils.MassPropertiesCfg(mass=0.1),
                           collision_props=sim_utils.CollisionPropertiesCfg())
cube.func("/World/cube", cube, translation=(0.0, 0.0, 0.5))
sim.reset()
t0 = time.monotonic()
for _ in range(120):
    sim.step(render=args.gui)
print(f"[2] Isaac Lab ok: 120 steps in {time.monotonic() - t0:.2f} s ({'viewer' if args.gui else 'headless'})")
if args.gui:
    time.sleep(5)   # time to look at the window
print("SMOKE TEST", "PASSED" if not failures else f"FAILED: {failures}", flush=True)
# Kit's close hangs in this image (seen 2026-11-18); nothing is left to save, so leave hard.
os._exit(1 if failures else 0)
