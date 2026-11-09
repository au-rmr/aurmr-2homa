"""Stands in for the wrench and the tool's stop signals, in its own process so the test's Python
pauses can't starve it. Controlled over /fake/* topics.

The wrench and speed scaling come from the UR driver on the real arm, so they are published on
the control host's private domain (robot.launch.py private_domain), as the driver's would be.
The tool's stop signals and the /fake/* controls are on the shared domain."""
import os
import threading

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Vector3, WrenchStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data as SENSOR
from std_msgs.msg import Bool, Float64, String

rclpy.init()
node = rclpy.create_node("fake_sensors")
private = Context()
rclpy.init(context=private, domain_id=int(os.environ.get("AURMR_PRIVATE_DOMAIN", "142")))
hw = rclpy.create_node("fake_arm_sensors", context=private)
state = {"force": Vector3(), "wrench": True, "signals": True, "contact": False, "seal": False,
         "scaling": 100.0, "scaling_on": True}
latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
wrench = hw.create_publisher(WrenchStamped, "/wrench", SENSOR)
scaling = hw.create_publisher(Float64, "/fake/speed_scaling", SENSOR)
pubs = {"contact": node.create_publisher(Bool, "/cap_contact", SENSOR),
        "seal": node.create_publisher(Bool, "/vacuum_seal", SENSOR)}


def on_force(msg):
    state["force"] = msg


def on_flags(msg):
    for item in msg.data.split():
        key, value = item.split("=")
        state[key] = float(value) if key == "scaling" else value == "1"


def tick_scaling():
    if state["scaling_on"]:
        scaling.publish(Float64(data=state["scaling"]))


def tick():
    if state["wrench"]:
        m = WrenchStamped()
        m.wrench.force = state["force"]
        wrench.publish(m)


def tick_signals():
    if state["signals"]:
        for name, pub in pubs.items():
            pub.publish(Bool(data=state[name]))


node.create_subscription(Vector3, "/fake/force", on_force, latched)
node.create_subscription(String, "/fake/flags", on_flags, latched)
hw.create_timer(0.002, tick)
node.create_timer(0.02, tick_signals)
hw.create_timer(0.01, tick_scaling)   # UR publishes at 100 Hz
hw_executor = SingleThreadedExecutor(context=private)
hw_executor.add_node(hw)
threading.Thread(target=hw_executor.spin, daemon=True).start()
rclpy.spin(node)
