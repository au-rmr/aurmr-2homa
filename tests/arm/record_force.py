"""Records the wrist force and the cap contact to CSV while a pick runs, to see what ends an
approach (S6 L5, 2026-11-13: approaches ended on the 15 N force stop before reaching the item;
the hose and cable pull on the wrist).

    python3 tests/arm/record_force.py OUT.csv [seconds]

Columns: t (s, monotonic from start), fx, fy, fz (N, raw), contact (last /cap_contact, or empty).
Thinned to one wrench sample per 10 ms. Read-only: subscribes, never commands.
"""
import csv
import sys
import time

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool


def main():
    out, seconds = sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 120.0
    rclpy.init()
    node = rclpy.create_node("record_force")
    t0, state = time.monotonic(), {"last": 0.0, "contact": ""}
    f = open(out, "w", newline="")
    w = csv.writer(f)
    w.writerow(["t", "fx", "fy", "fz", "contact"])

    def on_wrench(m):
        now = time.monotonic()
        if now - state["last"] < 0.01:
            return
        state["last"] = now
        F = m.wrench.force
        w.writerow([f"{now - t0:.3f}", f"{F.x:.3f}", f"{F.y:.3f}", f"{F.z:.3f}", state["contact"]])

    node.create_subscription(WrenchStamped, "/force_torque_sensor_broadcaster/wrench", on_wrench,
                             qos_profile_sensor_data)
    node.create_subscription(Bool, "/cap_contact", lambda m: state.update(contact=int(m.data)),
                             qos_profile_sensor_data)
    try:
        while rclpy.ok() and time.monotonic() - t0 < seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    f.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
