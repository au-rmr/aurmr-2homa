"""Listens to /joint_states for N seconds and prints wall time, stamp, max |velocity| per message.

Sends nothing. Used at S4 to time how fast the arm stops after a kill (step 10) or a network cut
(step 11). For a cut, run it on the control host's private domain, so the cut doesn't blind it:
    scripts/private_shell.sh python3 /repo/tests/arm/record_joints.py 40 > joints.csv
Usage: python3 tests/arm/record_joints.py <seconds>
"""
import sys
import time

import rclpy
from sensor_msgs.msg import JointState


def main():
    seconds = float(sys.argv[1])
    rclpy.init()
    node = rclpy.create_node("s4_recorder")

    def on_joints(m):
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        speed = max((abs(v) for v in m.velocity), default=0.0)
        print(f"{time.time():.4f},{stamp:.4f},{speed:.6f}", flush=True)

    node.create_subscription(JointState, "/joint_states", on_joints, 100)
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
