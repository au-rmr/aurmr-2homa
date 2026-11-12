"""The tablet slider's current value, for real-arm scripts to print a move's real speed before it
runs (user 2026-11-12: speeds in code are safe at 100 %, the slider only slows them down)."""
import os
import threading

from std_msgs.msg import Float64

TOPIC = os.environ.get("SPEED_SCALING_TOPIC", "/speed_scaling_state_broadcaster/speed_scaling")


def read(robot, timeout_s=1.0):
    """Slider as a fraction (0-1), or None if nothing arrives (e.g. on mock)."""
    got, value = threading.Event(), []

    def on(m):
        if not got.is_set():
            value.append(m.data)
            got.set()

    sub = robot._node.create_subscription(Float64, TOPIC, on, 10)
    try:
        got.wait(timeout_s)
    finally:
        robot._node.destroy_subscription(sub)
    return value[0] / 100.0 if value else None


def note(robot, speed, unit="mm/s", scale=1000.0):
    """' = 1.5 mm/s at slider 50 %' for a move capped at speed (m/s)."""
    s = read(robot)
    if s is None:
        return f" (slider unknown: {speed * scale:.1f} {unit} at 100 %)"
    return f" = {speed * scale * s:.1f} {unit} at slider {100 * s:.0f} %"
