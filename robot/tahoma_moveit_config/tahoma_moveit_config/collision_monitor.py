"""Collision monitor for streamed motion (S7): is the arm in collision now, or about to be?

Streaming bypasses MoveIt's planning, so nothing else checks where a streamed command takes the arm. This node checks
the robot model (with its tool) against a named set of the site's workcell objects, and against itself (the SRDF's
allowed collisions apply), for the current joint state and for predicted states up to horizon_s ahead:
q + v * t for v = the measured joint velocity and v = the latest streamed command. It publishes a
tahoma_interfaces/CollisionStatus at rate_hz. motion_server's stream session needs a fresh clear status for its
scene and stops the arm otherwise, so a silent or slow monitor stops streaming too (fail closed).

    python3 -m tahoma_moveit_config.collision_monitor --ros-args -p site_dir:=<site> -p tool:=capcup \
        -p scene:=limits_full,pod_partial

scene: comma-separated workcell scene and object names (workcell.object_names); it is also the status's scene
label, which a session must name. Objects are boxes, grown by padding_m on every side (default 0: the robot's
own frame, back_frame, is within 1 cm of its base; the look-ahead gives margin in the direction of motion). Contact with things that
are not in the scene (the cubes a demo pushes, a bin's soft walls) is not checked here; leave them out of the scene.
This monitor is not a planner and adds no motion: it only says clear or not.
"""
import tempfile
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from tahoma_interfaces.msg import CollisionStatus
from tahoma_moveit_config import workcell as wc
from tahoma_moveit_config.model import robot_description, robot_semantic
from tahoma_moveit_config.srdf import ARM_JOINTS

GROUP = "manipulator"


def _file(text, suffix):
    f = tempfile.NamedTemporaryFile("w", suffix=suffix, delete=False)
    f.write(text)
    f.close()
    return f.name


class Checker:
    """The robot model and a scene of workcell boxes. No ROS: usable from tests.

    moveit_py's PlanningScene objects all share one world in a process (found 2026-11-19: an object applied
    to one scene shows up in every other, even on another RobotModel). So there is exactly one scene here,
    and culprits() takes objects out and puts them back to name what collides. One Checker per process.
    """

    def __init__(self, site_dir, tool, names, padding_m=0.0):
        from moveit.core.planning_scene import PlanningScene
        from moveit.core.robot_model import RobotModel
        from moveit.core.robot_state import RobotState
        from moveit_msgs.msg import CollisionObject
        self._CollisionObject = CollisionObject
        self.model = RobotModel(urdf_xml_path=_file(robot_description(site_dir, tool), ".urdf"),
                                srdf_xml_path=_file(robot_semantic(site_dir, tool), ".srdf"))
        workcell = wc.load(f"{site_dir}/workcell.yaml")
        self.objects = {}
        for name in wc.object_names(workcell, names):
            co = wc.collision_object(workcell, name)
            co.primitives[0].dimensions = [d + 2 * padding_m for d in co.primitives[0].dimensions]
            self.objects[name] = co
        self.scene = PlanningScene(self.model)
        self.scene.remove_all_collision_objects()
        for co in self.objects.values():
            self.scene.apply_collision_object(co)
        self._check_world()
        self.state = RobotState(self.model)

    def _check_world(self):
        have = {o.id for o in self.scene.planning_scene_message.world.collision_objects}
        if have != set(self.objects):
            raise RuntimeError(f"collision scene holds {sorted(have)}, expected {sorted(self.objects)}")

    def _set(self, q):
        self.state.set_joint_group_positions(GROUP, np.asarray(q, dtype=float))
        self.state.update()

    def colliding(self, q):
        self._set(q)
        return self.scene.is_state_colliding(self.state, GROUP, False)

    def _remove(self, name):
        self.scene.apply_collision_object(self._CollisionObject(id=name, operation=self._CollisionObject.REMOVE))

    def culprits(self, q):
        """What the state q collides with: ['self'] or object names (slow path, only after a collision).
        Restores the full scene before returning, and checks it did."""
        self._set(q)
        out = []
        try:
            for name in self.objects:
                self._remove(name)
            if self.scene.is_state_colliding(self.state, GROUP, False):
                out = ["self"]   # with one object back, every object would then show as colliding
            else:
                for name, co in self.objects.items():
                    self.scene.apply_collision_object(co)
                    if self.scene.is_state_colliding(self.state, GROUP, False):
                        out.append(name)
                    self._remove(name)
        finally:
            self.scene.remove_all_collision_objects()
            for co in self.objects.values():
                self.scene.apply_collision_object(co)
            self._check_world()
        return out


def first_collision(checker, q, velocities, horizon_s, step_s):
    """(t, source, q_t) of the first predicted state in collision, or None. velocities: {source: v}."""
    steps = max(1, int(round(horizon_s / step_s)))
    q = np.asarray(q, dtype=float)
    if checker.colliding(q):
        return 0.0, "now", q
    for k in range(1, steps + 1):
        t = k * horizon_s / steps
        for source, v in velocities.items():
            qt = q + np.asarray(v, dtype=float) * t
            if checker.colliding(qt):
                return t, source, qt
    return None


class CollisionMonitor(Node):
    def __init__(self):
        super().__init__("collision_monitor")
        p = lambda name, default: self.declare_parameter(name, default).value
        site, tool = p("site_dir", ""), p("tool", "capcup")
        self.scene_label = p("scene", "")
        if not site or not self.scene_label:
            raise RuntimeError("site_dir and scene are required parameters")
        self.horizon = float(p("horizon_s", 0.25))
        self.step = float(p("step_s", 0.05))
        self.joint_stale = float(p("joint_stale_s", 0.05))
        self.command_stale = float(p("command_stale_s", 0.3))
        names = [n.strip() for n in self.scene_label.split(",") if n.strip()]
        padding = float(p("padding_m", 0.0))
        self.checker = Checker(site, tool, names, padding)
        self.q = self.v = self.u = None
        self.q_time = self.u_time = 0.0
        self.create_subscription(JointState, p("joints_topic", "/joint_states"), self._on_joints,
                                 qos_profile_sensor_data)
        self.create_subscription(JointState, p("command_topic", "/mpc/joint_velocity_cmd"), self._on_command,
                                 qos_profile_sensor_data)
        self.pub = self.create_publisher(CollisionStatus, p("status_topic", "/collision_monitor/status"), 10)
        self.create_timer(1.0 / float(p("rate_hz", 100.0)), self._tick)
        self._last_reason = None
        self._culprit_cache = (0.0, "")
        self.get_logger().info(f"checking scene '{self.scene_label}' ({len(self.checker.objects)} objects, "
                               f"padding {padding} m) {self.horizon} s ahead")

    @staticmethod
    def _pick(msg, field):
        values = getattr(msg, field)
        if len(values) != len(msg.name):
            return None
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(j in idx for j in ARM_JOINTS):
            return None
        return [values[idx[j]] for j in ARM_JOINTS]

    def _on_joints(self, msg):
        q = self._pick(msg, "position")
        if q is not None:
            self.q, self.v, self.q_time = q, self._pick(msg, "velocity") or [0.0] * 6, time.monotonic()

    def _on_command(self, msg):
        u = self._pick(msg, "velocity")
        if u is not None and all(np.isfinite(u)):
            self.u, self.u_time = u, time.monotonic()

    def _tick(self):
        now = time.monotonic()
        status = CollisionStatus(scene=self.scene_label, horizon_s=self.horizon)
        status.header.stamp = self.get_clock().now().to_msg()
        age = now - self.q_time if self.q is not None else float("inf")
        status.joint_state_age_s = age if np.isfinite(age) else -1.0
        if age > self.joint_stale:
            status.clear, status.reason = False, f"no joint state for {age:.3f} s"
        else:
            velocities = {"measured": self.v}
            if self.u is not None and now - self.u_time < self.command_stale:
                velocities["command"] = self.u
            hit = first_collision(self.checker, self.q, velocities, self.horizon, self.step)
            if hit is None:
                status.clear = True
            else:
                t, source, qt = hit
                if now - self._culprit_cache[0] > 0.5:   # naming is slow: at most twice a second
                    self._culprit_cache = (now, ", ".join(self.checker.culprits(qt)) or "unknown")
                what = self._culprit_cache[1]
                when = "now" if source == "now" else f"in {t:.2f} s at the {source} velocity"
                status.clear, status.reason = False, f"collision {when}: {what}"
        # Log changes of verdict, not every tick (a stale joint state's age changes each tick).
        if status.clear != self._last_reason:
            if status.clear:
                self.get_logger().info("clear")
            else:
                self.get_logger().warn(status.reason)
        self._last_reason = status.clear
        self.pub.publish(status)


def main():
    rclpy.init()
    node = CollisionMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
