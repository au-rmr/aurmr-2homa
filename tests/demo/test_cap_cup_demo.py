"""The cap_cup demo end to end on mock hardware, with the real capcup drivers, a fake ejector
and a pretend item (fake_world.py). Run by scripts/test_demo.sh."""
import math
import os
import subprocess
import threading
import time

import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float32, String

SITE = os.environ.get("AURMR_SITE_DIR", "/repo/sites/uw_lab")


@pytest.fixture(scope="module")
def world():
    rclpy.init()
    node = rclpy.create_node("demo_test")
    pub = node.create_publisher(String, "/fake/world", 10)
    tilts = []
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(Float32, "/fake/seal_tilt_deg", lambda m: tilts.append(m.data), latched)
    stop = threading.Event()
    spin = threading.Thread(target=lambda: [rclpy.spin_once(node, timeout_sec=0.05)
                                            for _ in iter(stop.is_set, True)], daemon=True)
    spin.start()
    time.sleep(1.0)

    def send(text):
        if text.startswith("place"):
            tilts.clear()
        pub.publish(String(data=text))
    send.tilts = tilts
    yield send
    stop.set()
    spin.join()
    node.destroy_node()
    rclpy.shutdown()


def run_demo(on_approach=None, timeout=240):
    """Runs the demo; on_approach() is called when it reaches the approach pose."""
    p = subprocess.Popen(["ros2", "run", "cap_cup_demo", "run", "--site-dir", SITE],
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                         env=dict(os.environ, PYTHONUNBUFFERED="1"))
    lines = []
    killer = threading.Timer(timeout, p.kill)
    killer.start()
    for line in p.stdout:
        lines.append(line.rstrip())
        print(line.rstrip())
        if "at the approach pose" in line and on_approach:
            on_approach()
    p.wait()
    killer.cancel()
    outcome = [ln for ln in lines if ln.startswith("OUTCOME")]
    return p.returncode, (outcome[-1] if outcome else ""), lines


@pytest.mark.parametrize("tilt", [10, -10])
def test_picks_a_tilted_item(world, tilt):
    """Both signs: a search that turned the wrong way would double the tilt and never seal."""
    world("clear")
    rc, outcome, lines = run_demo(on_approach=lambda: world(f"place 0.03 {tilt}"))
    assert outcome.startswith("OUTCOME picked"), "\n".join(lines[-30:])
    assert rc == 0
    assert "approach ended: contact" in "\n".join(lines)
    moves = int(outcome.split("'search_moves': ")[1].split(",")[0].rstrip("}"))
    assert moves >= 1, "the search must have moved the cup at least once"
    time.sleep(0.5)
    assert world.tilts, "the fake world saw no seal"
    print(f"tilt at the seal {world.tilts[0]:.1f} deg, from {abs(tilt)}")
    # The old search alternates turn and move proposals every 20 ms and the demo takes the next
    # one, so a pick may seal by pressing deeper without turning. It must never turn the wrong way
    # (tests/capcup checks the direction exactly).
    assert world.tilts[0] < abs(tilt) + 1.0, "the search tilted the cup further"


def test_nothing_to_touch_ends_without_searching(world):
    world("clear")
    rc, outcome, lines = run_demo()
    assert outcome.startswith("OUTCOME no_contact"), "\n".join(lines[-30:])
    assert rc == 1
