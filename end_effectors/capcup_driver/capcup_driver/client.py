"""The capcup as a demo sees it: contact, alignment, haptic proposals and the vacuum.

Every reading must be fresh, or it raises ToolError: a stale "sealed" must not read as held.
"""
import threading
import time

import rclpy
from capcup_interfaces.msg import CapData
from capcup_interfaces.srv import SetVacuum
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

# How old a reading may be. contact comes once per cap board sample (every 0.171 s, S6
# 2026-11-12: the code had assumed 50 Hz and 0.2 s left 29 ms); alignment and seal come at 50 Hz.
FRESH_S = {"contact": 0.5, "aligned": 0.2, "sealed": 0.2}
SETTLE_S = 0.05   # after a new cap sample: the 50 Hz search has published from it by then


class ToolError(Exception):
    pass


class CapCup:
    def __init__(self, timeout_s=10.0):
        if not rclpy.ok():
            rclpy.init()
        self._node = rclpy.create_node("capcup_client")
        self._timeout = timeout_s
        self._latest = {}
        self._proposals = {"turn": None, "move": None}
        self._sample_stamp = None   # ROS time (s) of the newest cap sample
        self._lock = threading.Lock()
        for name, topic in (("contact", "/cap_contact"), ("aligned", "/haptic_search/alignment"),
                            ("sealed", "/vacuum_seal")):
            qos = 1 if name == "aligned" else qos_profile_sensor_data
            self._node.create_subscription(
                Bool, topic, lambda m, n=name: self._store(n, m.data), qos)
        self._node.create_subscription(CapData, "/cap_data", self._on_sample, qos_profile_sensor_data)
        for kind in self._proposals:
            self._node.create_subscription(PoseStamped, f"/haptic_search/proposed_{kind}",
                                           lambda m, k=kind: self._on_proposal(k, m), 1)
        self._vacuum = self._node.create_client(SetVacuum, "/vacuum/set")
        self._rezero = self._node.create_client(Trigger, "/haptic_search/rezero")
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        threading.Thread(target=self._executor.spin, daemon=True).start()

    def _store(self, name, value):
        with self._lock:
            self._latest[name] = (time.monotonic(), value)

    def _on_sample(self, msg):
        with self._lock:
            self._sample_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _on_proposal(self, kind, msg):
        # Stamped with the cap sample it was computed from (haptic_search), not when it was sent.
        p, o = msg.pose.position, msg.pose.orientation
        made = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            self._proposals[kind] = (made, (p.x, p.y, p.z), (o.x, o.y, o.z, o.w))

    def now(self):
        """ROS time (s) on this host's clock: what proposals and samples are stamped with. The
        bridge stamps samples on the compute host, so the demo must run there too."""
        return self._node.get_clock().now().nanoseconds * 1e-9

    def wait_for_sample(self, after, wait_s=1.0):
        """Waits for a cap sample taken after `after` (now()), then SETTLE_S so alignment and
        proposals have been worked out from it. Raises ToolError if none comes."""
        end = time.monotonic() + wait_s
        while time.monotonic() < end:
            with self._lock:
                got = self._sample_stamp
            if got is not None and got > after:
                time.sleep(SETTLE_S)
                return
            time.sleep(0.01)
        raise ToolError(f"no cap sample within {wait_s} s")

    def _fresh(self, name):
        with self._lock:
            got = self._latest.get(name)
        if got is None or time.monotonic() - got[0] > FRESH_S[name]:
            raise ToolError(f"no recent {name} reading from the capcup")
        return got[1]

    def contact(self):
        return self._fresh("contact")

    def aligned(self):
        return self._fresh("aligned")

    def sealed(self):
        return self._fresh("sealed")

    def rezero(self, samples=4):
        """The search takes a new in-air zero (haptic_search ~/rezero averages its next 3 samples);
        returns once `samples` cap samples have come after the call, so it has been applied. Call
        with the cup in free air, in the pose it will touch from."""
        if not self._rezero.wait_for_service(timeout_sec=self._timeout):
            raise ToolError("haptic_search rezero service is not available")
        asked = self.now()
        future = self._rezero.call_async(Trigger.Request())
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(self._timeout) or not future.result().success:
            raise ToolError("haptic_search did not re-zero")
        for _ in range(samples):
            self.wait_for_sample(after=asked)
            asked = self.now()

    def wait_ready(self):
        """Waits until every reading is fresh: drivers up, board streaming."""
        end = time.monotonic() + self._timeout
        while True:
            try:
                return self.contact(), self.sealed()
            except ToolError:
                if time.monotonic() > end:
                    raise
                time.sleep(0.05)

    def proposal(self, kind, newer_than, wait_s=1.0):
        """The first haptic proposal of this kind ("turn" or "move") computed from a cap sample
        taken after newer_than (now()), as (xyz, quat) in the tool frame, or None if none comes
        within wait_s. No turn comes while the contact is already level."""
        if kind not in self._proposals:
            raise ValueError(f"kind must be turn or move, got {kind}")
        end = time.monotonic() + wait_s
        while time.monotonic() < end:
            with self._lock:
                got = self._proposals[kind]
            if got is not None and got[0] > newer_than:
                return got[1], got[2]
            time.sleep(0.01)
        return None

    def _set(self, mode):
        if not self._vacuum.wait_for_service(timeout_sec=self._timeout):
            raise ToolError("vacuum service is not available")
        future = self._vacuum.call_async(SetVacuum.Request(mode=mode))
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(self._timeout):
            raise ToolError("vacuum did not answer")
        if not future.result().success:
            raise ToolError(f"vacuum refused: {future.result().message}")

    def suck(self):
        self._set(SetVacuum.Request.SUCK)

    def blow(self):
        self._set(SetVacuum.Request.BLOW)

    def off(self):
        self._set(SetVacuum.Request.OFF)

    def close(self):
        self._executor.shutdown(timeout_sec=1.0)
        self._node.destroy_node()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
