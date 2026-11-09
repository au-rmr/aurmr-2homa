"""The capcup nodes running: UDP in, topics and the vacuum service out. Needs the launch from
scripts/test_tools.sh (part all, fake vacuum)."""
import socket
import struct
import threading
import time

import pytest
import rclpy
from capcup_interfaces.msg import CapData
from capcup_interfaces.srv import SetVacuum
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from sequence import samples


class Listen:
    def __init__(self, node, kind, topic, qos=qos_profile_sensor_data):
        self.msgs = []
        node.create_subscription(kind, topic, lambda m: self.msgs.append((time.monotonic(), m)), qos)

    def rate(self, since):
        times = [t for t, _ in self.msgs if t >= since]
        return (len(times) - 1) / (times[-1] - times[0]) if len(times) > 2 else 0.0


@pytest.fixture(scope="module")
def node():
    rclpy.init()
    n = rclpy.create_node("capcup_test")
    stop = threading.Event()
    t = threading.Thread(target=lambda: [rclpy.spin_once(n, timeout_sec=0.01) for _ in iter(stop.is_set, True)],
                         daemon=True)
    t.start()
    yield n
    stop.set()
    t.join()
    rclpy.shutdown()


def send_sequence(n=None):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    start = time.monotonic()
    for i, (ms, values) in enumerate(samples()[:n]):
        sock.sendto(struct.pack("<Q8l", ms, *values), ("127.0.0.1", 12345))
        time.sleep(max(0.0, start + 0.02 * (i + 1) - time.monotonic()))
    sock.close()


def test_board_packets_reach_contact_and_search(node):
    data = Listen(node, CapData, "/cap_data")
    contact = Listen(node, Bool, "/cap_contact")
    turns = Listen(node, PoseStamped, "/haptic_search/proposed_turn", 1)
    moves = Listen(node, PoseStamped, "/haptic_search/proposed_move", 1)
    aligned = Listen(node, Bool, "/haptic_search/alignment", 1)
    time.sleep(2.0)   # discovery
    t0 = time.monotonic()
    send_sequence()
    time.sleep(0.5)
    assert len(data.msgs) >= 590, f"{len(data.msgs)} of 600 packets arrived"
    assert data.msgs[0][1].device_time_s == pytest.approx(1000.0)
    assert contact.rate(t0) > 40, "contact must be published for every sample"
    flags = [m.data for _, m in contact.msgs]
    assert any(flags) and not all(flags)
    assert len(turns.msgs) > 150 and len(moves.msgs) > 150
    assert moves.msgs[-1][1].header.frame_id == "gripper_equilibrium_grasp"
    # A turn has no sideways part, a move no tilt.
    assert all(abs(m.pose.position.x) < 1e-12 and abs(m.pose.position.y) < 1e-12
               for _, m in turns.msgs)
    assert all(abs(m.pose.orientation.w) == pytest.approx(1.0) for _, m in moves.msgs)
    assert any(abs(m.pose.orientation.w) < 0.99999 for _, m in turns.msgs)
    assert any(m.data for _, m in aligned.msgs)
    # The board went quiet: no more proposals.
    n = len(turns.msgs) + len(moves.msgs)
    time.sleep(0.6)
    assert len(turns.msgs) + len(moves.msgs) == n, "haptic search kept proposing without fresh data"


def test_wrong_size_packet_is_ignored(node):
    data = Listen(node, CapData, "/cap_data")
    time.sleep(1.0)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(b"\x00" * 10, ("127.0.0.1", 12345))
    sock.close()
    time.sleep(0.5)
    assert data.msgs == []


def call(node, mode):
    client = node.create_client(SetVacuum, "/vacuum/set")
    assert client.wait_for_service(timeout_sec=10)
    future = client.call_async(SetVacuum.Request(mode=mode))
    end = time.monotonic() + 5
    while not future.done() and time.monotonic() < end:
        time.sleep(0.01)
    return future.result()


def test_vacuum_seal_follows_the_valve(node):
    seal = Listen(node, Bool, "/vacuum_seal")
    time.sleep(1.5)
    t0 = time.monotonic()
    assert call(node, SetVacuum.Request.SUCK).success
    time.sleep(1.0)
    assert seal.msgs[-1][1].data, "no seal while sucking on an item"
    assert seal.rate(t0) > 40, "seal must be fresh enough for a stop signal (100 ms)"
    assert call(node, SetVacuum.Request.BLOW).success
    time.sleep(0.3)
    assert not seal.msgs[-1][1].data
    assert call(node, SetVacuum.Request.OFF).success


def test_unknown_vacuum_mode_refused(node):
    r = call(node, 7)
    assert not r.success and "unknown" in r.message
