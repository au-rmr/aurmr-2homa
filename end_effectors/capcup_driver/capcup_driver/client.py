"""The capcup as a demo sees it: contact, alignment, haptic proposals and the vacuum.

Every reading must be fresh, or it raises ToolError: a stale "sealed" must not read as held.
"""
import threading
import time

import rclpy
from capcup_interfaces.srv import SetVacuum
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool

FRESH_S = 0.2


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
        self._lock = threading.Lock()
        for name, topic in (("contact", "/cap_contact"), ("aligned", "/haptic_search/alignment"),
                            ("sealed", "/vacuum_seal")):
            qos = 1 if name == "aligned" else qos_profile_sensor_data
            self._node.create_subscription(
                Bool, topic, lambda m, n=name: self._store(n, m.data), qos)
        for kind in self._proposals:
            self._node.create_subscription(PoseStamped, f"/haptic_search/proposed_{kind}",
                                           lambda m, k=kind: self._on_proposal(k, m), 1)
        self._vacuum = self._node.create_client(SetVacuum, "/vacuum/set")
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        threading.Thread(target=self._executor.spin, daemon=True).start()

    def _store(self, name, value):
        with self._lock:
            self._latest[name] = (time.monotonic(), value)

    def _on_proposal(self, kind, msg):
        p, o = msg.pose.position, msg.pose.orientation
        with self._lock:
            self._proposals[kind] = (time.monotonic(), (p.x, p.y, p.z), (o.x, o.y, o.z, o.w))

    def _fresh(self, name):
        with self._lock:
            got = self._latest.get(name)
        if got is None or time.monotonic() - got[0] > FRESH_S:
            raise ToolError(f"no recent {name} reading from the capcup")
        return got[1]

    def contact(self):
        return self._fresh("contact")

    def aligned(self):
        return self._fresh("aligned")

    def sealed(self):
        return self._fresh("sealed")

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
        """The first haptic proposal of this kind ("turn" or "move") made after newer_than
        (time.monotonic()), as (xyz, quat) in the tool frame, or None if none comes within
        wait_s. No turn comes while the contact is already level."""
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
