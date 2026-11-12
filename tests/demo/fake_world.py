"""A pretend item for the cap_cup demo on mock hardware.

Streams cap board packets over UDP as the real board would, from where the mock arm's cup is
relative to a flat, tilted item surface. Each of the 8 electrodes sits on the cup rim (radius 2 cm,
the haptic search's own layout) and reads base + 10000 counts per mm it presses past the surface:
the scale at which the search's plane fit (calibration 200000) reads the tilt correctly.
Publishes /fake/item_under_cup once every electrode presses at least 1 mm: the whole rim touches,
so the fake vacuum seals. At that moment it publishes /fake/seal_tilt_deg, the angle between the
cup axis and the surface normal, so a test can check the search turned the cup the right way.
Once sealed, the item comes with the cup until the vacuum level drops (off or blow-off).

Packets go out at the real board's rate, one every 0.1715 s (5.83 Hz, measured 2026-11-12; S6
found the code had assumed 50 Hz). FAKE_CAP_PERIOD_S overrides it. Contact, seal and the item
are still worked out at 50 Hz.

Commands on /fake/world (std_msgs/String): "place <ahead_m> <tilt_deg>" puts the surface ahead of
the cup along its axis, tilted about the cup's x axis; "clear" removes it.
"""
import math
import os
import socket
import struct
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Bool, Float32, String
from tf2_ros import Buffer, TransformListener

TCP = "gripper_equilibrium_grasp"
BOARD_PERIOD_S = float(os.environ.get("FAKE_CAP_PERIOD_S", "0.1715"))
RADIUS = 0.02
COUNTS_PER_M = 1e7
ANGLES = np.linspace(0, 2 * np.pi, 8, endpoint=False) + np.pi / 8
BASE = np.array([3_000_000 + 100_000 * k for k in range(8)], dtype=float)


def rotation(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def main():
    rclpy.init()
    node = rclpy.create_node("fake_world")
    tf = Buffer()
    TransformListener(tf, node)
    item_pub = node.create_publisher(Bool, "/fake/item_under_cup", 10)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    tilt_pub = node.create_publisher(Float32, "/fake/seal_tilt_deg", latched)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    surface = {"point": None, "normal": None}
    state = {"held": False, "sent": 0.0}
    rng = np.random.default_rng(7)
    t0 = time.monotonic()

    def cup():
        t = tf.lookup_transform("base_link", TCP, Time(), timeout=Duration(seconds=0.05))
        p, q = t.transform.translation, t.transform.rotation
        return np.array([p.x, p.y, p.z]), rotation((q.x, q.y, q.z, q.w))

    def on_command(msg):
        words = msg.data.split()
        state["held"] = False
        if words[0] == "clear":
            surface["point"] = None
            return
        ahead, tilt = float(words[1]), math.radians(float(words[2]))
        pos, rot = cup()
        axis = rot[:, 2]
        tilted = rot @ np.array([0.0, -math.sin(tilt), math.cos(tilt)])   # about the cup's x
        surface["point"] = pos + ahead * axis
        surface["normal"] = tilted
        node.get_logger().info(f"item placed {ahead} m ahead, tilted {words[2]} deg")

    def tick():
        depth = np.zeros(8)
        if surface["point"] is not None:
            try:
                pos, rot = cup()
            except Exception:
                return
            rim = np.stack([pos + rot @ np.array([RADIUS * math.cos(a), RADIUS * math.sin(a), 0.0])
                            for a in ANGLES])
            depth = np.maximum(0.0, (rim - surface["point"]) @ surface["normal"])
        now = time.monotonic()
        if now - state["sent"] >= BOARD_PERIOD_S:
            state["sent"] = now
            counts = BASE + COUNTS_PER_M * depth + rng.integers(-300, 300, 8)
            ms = int((now - t0) * 1000) + 1_000_000
            sock.sendto(struct.pack("<Q8l", ms, *[int(c) for c in counts]), ("127.0.0.1", 12345))
        held = state["held"] or bool(np.all(depth > 0.001))
        if held and not state["held"]:
            _, rot = cup()
            tilt = math.degrees(math.acos(min(1.0, abs(float(rot[:, 2] @ surface["normal"])))))
            tilt_pub.publish(Float32(data=tilt))
            node.get_logger().info(f"seal: tilt {tilt:.1f} deg, rim depth "
                                   f"{np.round(depth * 1000, 1).tolist()} mm")
        state["held"] = held
        item_pub.publish(Bool(data=held))

    def on_vacuum(msg):
        if msg.data < 100 and state["held"]:
            state["held"] = False
            surface["point"] = None   # the item went with the cup, then dropped
            node.get_logger().info("released")

    node.create_subscription(String, "/fake/world", on_command, 10)
    node.create_subscription(Float32, "/vacuum/system_vacuum_mbar", on_vacuum, qos_profile_sensor_data)
    node.create_timer(0.02, tick)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
