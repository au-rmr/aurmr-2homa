"""Robot API. MoveIt plans, the motion server runs the plan.

robot = Robot.connect(tool="capcup", force_limit_n=50, workcell=path)
robot.scene.use("limits_full", "table_down")
robot.go_named("drop_hide")
result = robot.move_straight((0, 0, .05), frame="tool", stop=Stop(contact=True, force_n=15))
"""
import math
import os
import threading
import time
import uuid
from dataclasses import dataclass, field

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory as share
from bondpy.bondpy import Bond
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (AttachedCollisionObject, BoundingVolume, CollisionObject, Constraints, JointConstraint,
                             MoveItErrorCodes, OrientationConstraint, PlanningScene,
                             PlanningSceneComponents, PositionConstraint)
from moveit_msgs.msg import RobotState as RobotStateMsg
from moveit_msgs.srv import (ApplyPlanningScene, GetCartesianPath, GetMotionPlan,
                             GetPlanningScene, GetPositionFK, GetStateValidity)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tahoma_interfaces.action import ExecuteTrajectory
from tahoma_interfaces.srv import Connect
from tahoma_moveit_config import workcell as W
from tahoma_moveit_config.srdf import ARM_JOINTS
from tf2_ros import Buffer, TransformListener

from tahoma_robot import path_check
from tahoma_robot.errors import (ExecutionFailed, ForceLimitExceeded, GuardLost, PartialPath,
                                 PlanningFailed, RobotError, SensorStale, Stalled)

GROUP = "manipulator"
BASE = "base_link"
TIMEOUT_S = 30.0
DEFAULT_STRAIGHT_SPEED = 0.03   # m/s, about what the old 5% scaling gave
MAX_SIGNAL_SPEED = 0.03         # m/s when a stop signal is what ends the move
MAX_STRAIGHT_SPEED = 0.25       # m/s, a typo guard; UR's own tool speed limit is 1.5 m/s
MAX_FORCE_LIMIT_N = 150.0
DEFAULT_ANGULAR_SPEED = 0.2     # rad/s for the tool's turn in a straight move, about 11 deg/s
# Planned moves (user 2026-11-12): every move must be safe with the tablet slider at 100 %, so a
# planned move is capped at a tool tip speed too, not only by joint scaling. The slider is set once
# per session and only slows things down.
DEFAULT_PLANNED_SPEED = 0.3          # joint velocity/acceleration scaling (fraction of the limits)
DEFAULT_PLANNED_TIP_SPEED = 0.25     # m/s, tool tip
DEFAULT_PLANNED_ANGULAR_SPEED = 0.8  # rad/s, tool turn, about 46 deg/s
MAX_PLANNED_TIP_SPEED = 0.5          # m/s, a typo guard
MAX_ANGULAR_SPEED = 1.0       # same as motion_server and UR's own tool force limit
PLAN_RETRIES = 2                # nothing has moved at that point
# FAILURE covers plans the check rejected and goals that are truly impossible: MoveIt gives both
# the same code, so an impossible goal takes all the attempts to fail, and counts in plan_retries.
RETRY_CODES = (MoveItErrorCodes.INVALID_MOTION_PLAN, MoveItErrorCodes.FAILURE)
HEARTBEAT_TIMEOUT_S = 0.5
HEARTBEAT_PERIOD_S = 0.1
_Out = ExecuteTrajectory.Result


@dataclass
class Stop:
    """Expected ways for a move to end early."""

    contact: bool = False
    seal: bool = False
    force_n: float = 0.0

    def __post_init__(self):
        if not math.isfinite(self.force_n) or self.force_n < 0:
            raise ValueError(f"force_n must be finite and not negative, got {self.force_n}")

    def signals(self):
        return [name for name in ("contact", "seal") if getattr(self, name)]


@dataclass
class Result:
    reason: str    # done, force, contact, seal, stopped
    peak_force_n: float = 0.0
    joints: dict = field(default_factory=dict)


def _qmul(a, b):
    """Hamilton product of (x, y, z, w) quaternions: a then b, b in a's frame."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _rotate(q, v):
    x, y, z, w = q
    tx, ty, tz = (2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0]))
    return (v[0] + w * tx + y * tz - z * ty,
            v[1] + w * ty + z * tx - x * tz,
            v[2] + w * tz + x * ty - y * tx)


class Scene:
    def __init__(self, robot):
        self._robot = robot
        self._placed = []

    def use(self, *names):
        """Replaces the whole scene with these scenes and objects from the site's workcell.
        Removes every world object MoveIt holds, also ones an earlier process placed (a demo run's
        last scene stayed in the world for the next run, 2026-11-12) and ones the workcell no
        longer names (an object deleted from workcell.yaml blocked the next plan, 2026-11-13).
        An item attached to the tool (hold) is not a world object and stays."""
        request = ApplyPlanningScene.Request(scene=PlanningScene(is_diff=True))
        wanted = W.object_names(self._robot.workcell, names)
        ask = GetPlanningScene.Request()
        ask.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        present = self._robot._wait(self._robot._get_scene.call_async(ask))
        for name in sorted({o.id for o in present.scene.world.collision_objects}):
            request.scene.world.collision_objects.append(
                CollisionObject(id=name, operation=CollisionObject.REMOVE))
        for name in wanted:
            request.scene.world.collision_objects.append(
                W.collision_object(self._robot.workcell, name))
        if not self._robot._wait(self._robot._apply_scene.call_async(request)).success:
            raise RobotError(f"MoveIt refused scene {names}")
        self._placed = wanted

    HELD = "held_item"

    def hold(self, size, start=0.0):
        """Attaches a box to the tool's TCP as the item it may be holding (user 2026-11-13), so
        MoveIt keeps the item, not just the tool, clear of every obstacle in any orientation.
        size: (x, y, z) in the TCP frame, z along the tool axis; the box runs from start (m, may
        be negative: a sealed suction cup pulls the item back) to start + size[2]. The tool's
        own links (tool.yaml links) may touch it. Stays until release(), also across scene.use."""
        co = CollisionObject(id=self.HELD, operation=CollisionObject.ADD)
        co.header.frame_id = self._robot.tcp
        co.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX,
                                            dimensions=[float(v) for v in size]))
        pose = Pose()
        pose.position.z = float(start + size[2] / 2)
        pose.orientation.w = 1.0
        co.primitive_poses.append(pose)
        held = AttachedCollisionObject(link_name=self._robot.tcp, object=co,
                                       touch_links=list(self._robot.tool["links"]))
        request = ApplyPlanningScene.Request(scene=PlanningScene(is_diff=True))
        request.scene.robot_state.is_diff = True
        request.scene.robot_state.attached_collision_objects.append(held)
        if not self._robot._wait(self._robot._apply_scene.call_async(request)).success:
            raise RobotError(f"MoveIt refused the held item {size}")

    def release(self):
        """Removes the held item, also one an earlier process attached and never released.
        MoveIt puts a detached object into the world, so it is removed from there too."""
        ask = GetPlanningScene.Request()
        ask.components.components = (PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS |
                                     PlanningSceneComponents.WORLD_OBJECT_NAMES)
        present = self._robot._wait(self._robot._get_scene.call_async(ask)).scene
        attached = any(a.object.id == self.HELD for a in present.robot_state.attached_collision_objects)
        in_world = any(o.id == self.HELD for o in present.world.collision_objects)
        if not attached and not in_world:
            return
        request = ApplyPlanningScene.Request(scene=PlanningScene(is_diff=True))
        if attached:
            request.scene.robot_state.is_diff = True
            request.scene.robot_state.attached_collision_objects.append(AttachedCollisionObject(
                link_name=self._robot.tcp,
                object=CollisionObject(id=self.HELD, operation=CollisionObject.REMOVE)))
        request.scene.world.collision_objects.append(
            CollisionObject(id=self.HELD, operation=CollisionObject.REMOVE))
        if not self._robot._wait(self._robot._apply_scene.call_async(request)).success:
            raise RobotError("MoveIt refused to release the held item")


class Robot:
    def __init__(self, tool, force_limit_n, workcell):
        if not force_limit_n or not 0 < force_limit_n <= MAX_FORCE_LIMIT_N:
            raise ValueError(f"force_limit_n is required, above 0 up to {MAX_FORCE_LIMIT_N}")
        with open(os.path.join(share(tool), "tool.yaml")) as f:
            self.tool = yaml.safe_load(f)
        self.force_limit_n = force_limit_n
        self.workcell = W.load(workcell)
        self.tcp = self.tool["tcp_link"]
        self.scene = Scene(self)
        self._joints = {}
        self._guard_lost = threading.Event()
        self._move_goal = None
        self._stop_asked = False
        self._bond_id = ""
        self._closed = False
        self.plan_retries = 0   # worth watching: a high count means planning is struggling

        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init()
        self._node = rclpy.create_node(f"robot_{uuid.uuid4().hex[:8]}")
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self._node)
        self._node.create_subscription(JointState, "joint_states", self._on_joints, 10)
        self._plan = self._node.create_client(GetMotionPlan, "plan_kinematic_path")
        self._cartesian = self._node.create_client(GetCartesianPath, "compute_cartesian_path")
        self._fk = self._node.create_client(GetPositionFK, "compute_fk")
        self._apply_scene = self._node.create_client(ApplyPlanningScene, "apply_planning_scene")
        self._get_scene = self._node.create_client(GetPlanningScene, "get_planning_scene")
        self._validity = self._node.create_client(GetStateValidity, "check_state_validity")
        self._connect = self._node.create_client(Connect, "motion_server/connect")
        self._execute = ActionClient(self._node, ExecuteTrajectory, "motion_server/execute")
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin.start()

    @classmethod
    def connect(cls, tool, force_limit_n, workcell):
        """Opens the motion guard. Every later move needs this connection alive."""
        robot = cls(tool, force_limit_n, workcell)
        try:
            robot._open_guard()
        except Exception:
            robot.close()
            raise
        return robot

    def _open_guard(self):
        for client in (self._plan, self._cartesian, self._fk, self._apply_scene, self._connect):
            if not client.wait_for_service(timeout_sec=TIMEOUT_S):
                raise RobotError(f"{client.srv_name} is not available")
        if not self._execute.wait_for_server(timeout_sec=TIMEOUT_S):
            raise RobotError("motion server is not available")
        self._bond_id = uuid.uuid4().hex
        answer = self._wait(self._connect.call_async(Connect.Request(bond_id=self._bond_id)))
        if not answer.accepted:
            raise GuardLost(answer.message)
        self._bond = Bond(self._node, "/tahoma/bond", self._bond_id,
                          on_broken=self._guard_lost.set)
        self._bond.set_heartbeat_timeout(HEARTBEAT_TIMEOUT_S)
        self._bond.set_heartbeat_period(HEARTBEAT_PERIOD_S)
        self._bond.start()
        if not self._bond.wait_until_formed(Duration(seconds=TIMEOUT_S)):
            raise GuardLost("the motion guard did not form")

    def close(self):
        if self._closed:
            return
        self._closed = True
        bond = getattr(self, "_bond", None)
        if bond is not None:
            bond.break_bond()
            # The bond's thread publishes until the other side answers, so it must end first.
            bond.wait_until_broken(Duration(seconds=1.0))
            bond.shutdown()
            # bondpy's shutdown cancels only its connect timer. The heartbeat and disconnect
            # timers are daemon threads that can fire after the node is gone, even during
            # interpreter exit, and call into it (T12 item 5: rare segfault at exit).
            for timer in (bond.connect_timer, bond.heartbeat_timer, bond.disconnect_timer):
                timer.cancel()
                if timer.timer.is_alive():
                    timer.timer.join(timeout=1.0)
            thread = getattr(bond, "thread", None)
            if thread is not None:
                thread.join(timeout=1.0)
        self._executor.shutdown(timeout_sec=1.0)
        self._spin.join(timeout=1.0)
        self._node.destroy_node()
        if self._owns_rclpy:
            rclpy.try_shutdown()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    # --- State ---------------------------------------------------------------------------------
    def _on_joints(self, msg):
        self._joints.update(zip(msg.name, msg.position))

    def joints(self):
        return dict(self._joints)

    def tcp_pose(self):
        t = self._tf.lookup_transform(BASE, self.tcp, Time(), timeout=Duration(seconds=2))
        p, q = t.transform.translation, t.transform.rotation
        return (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)

    # --- Moves ---------------------------------------------------------------------------------
    def go_named(self, name, speed=DEFAULT_PLANNED_SPEED, tip_speed=DEFAULT_PLANNED_TIP_SPEED,
                 angular_speed=DEFAULT_PLANNED_ANGULAR_SPEED, planner="ompl"):
        return self.go_joints(self.workcell["named_poses"][name], speed, tip_speed, angular_speed,
                              planner)

    def go_joints(self, values, speed=DEFAULT_PLANNED_SPEED, tip_speed=DEFAULT_PLANNED_TIP_SPEED,
                  angular_speed=DEFAULT_PLANNED_ANGULAR_SPEED, planner="ompl"):
        """Planned move to joint values, in ARM_JOINTS order or as a {joint: value} dict.
        speed scales the joint limits; tip_speed (m/s) and angular_speed (rad/s) cap the tool.
        planner: "ompl" (sampled path, finds a way round obstacles, differs every plan) or "ptp"
        (Pilz: straight line in joint space, the same path every time, fails if it collides)."""
        if isinstance(values, dict):
            values = [values[n] for n in ARM_JOINTS]
        goal = Constraints(joint_constraints=[
            JointConstraint(joint_name=n, position=float(v), tolerance_above=0.001,
                            tolerance_below=0.001, weight=1.0)
            for n, v in zip(ARM_JOINTS, values, strict=True)])
        return self._run(self._planned(goal, speed, tip_speed, angular_speed, planner))

    def plan_joints(self, values, speed=DEFAULT_PLANNED_SPEED, tip_speed=DEFAULT_PLANNED_TIP_SPEED,
                    angular_speed=DEFAULT_PLANNED_ANGULAR_SPEED, planner="ompl"):
        """Plans as go_joints does and returns the trajectory without moving (to keep a good path:
        run it with run_path, save it with path_to_dict)."""
        if isinstance(values, dict):
            values = [values[n] for n in ARM_JOINTS]
        goal = Constraints(joint_constraints=[
            JointConstraint(joint_name=n, position=float(v), tolerance_above=0.001,
                            tolerance_below=0.001, weight=1.0)
            for n, v in zip(ARM_JOINTS, values, strict=True)])
        return self._planned(goal, speed, tip_speed, angular_speed, planner)

    def run_path(self, trajectory):
        """Runs a stored joint trajectory (a recorded path, 2026-11-13). Every point is checked
        first against the current scene, a held item included; raises PlanningFailed if one
        collides. motion_server refuses it unless the arm is at its first point."""
        if not self._validity.wait_for_service(timeout_sec=TIMEOUT_S):
            raise RobotError("check_state_validity is not available")
        for i, point in enumerate(trajectory.points):
            state = RobotStateMsg(is_diff=True)
            state.joint_state.name = list(trajectory.joint_names)
            state.joint_state.position = list(point.positions)
            answer = self._wait(self._validity.call_async(
                GetStateValidity.Request(robot_state=state, group_name=GROUP)))
            if not answer.valid:
                pairs = sorted({f"{c.contact_body_1}-{c.contact_body_2}" for c in answer.contacts})
                raise PlanningFailed(f"stored path point {i} of {len(trajectory.points)} collides: {pairs}")
        return self._run(trajectory)

    def plan_to(self, xyz, quat, speed=DEFAULT_PLANNED_SPEED, position_tolerance=0.005,
                angle_tolerance=0.02, link=None, tip_speed=DEFAULT_PLANNED_TIP_SPEED,
                angular_speed=DEFAULT_PLANNED_ANGULAR_SPEED):
        """Planned move putting link (default: the tool's TCP) at xyz, quat in base_link.
        speed scales the joint limits; tip_speed (m/s) and angular_speed (rad/s) cap the tool."""
        link = link or self.tcp
        target = PoseStamped()
        target.header.frame_id = BASE
        target.pose = Pose()
        target.pose.position.x, target.pose.position.y, target.pose.position.z = map(float, xyz)
        o = target.pose.orientation
        o.x, o.y, o.z, o.w = map(float, quat)
        region = BoundingVolume()
        region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE,
                                                dimensions=[position_tolerance]))
        region.primitive_poses.append(target.pose)
        goal = Constraints(
            position_constraints=[PositionConstraint(
                header=target.header, link_name=link, constraint_region=region, weight=1.0)],
            orientation_constraints=[OrientationConstraint(
                header=target.header, link_name=link, orientation=o, weight=1.0,
                absolute_x_axis_tolerance=angle_tolerance,
                absolute_y_axis_tolerance=angle_tolerance,
                absolute_z_axis_tolerance=angle_tolerance)])
        return self._run(self._planned(goal, speed, tip_speed, angular_speed))

    def move_straight(self, offset, frame="tool", stop=None, speed=DEFAULT_STRAIGHT_SPEED,
                      check_collisions=True, rotation=None, angular_speed=DEFAULT_ANGULAR_SPEED):
        """Moves the tool tip in a line. Raises unless the whole path can be planned.

        rotation: optional (x, y, z, w) turn of the tool about its tip, in the tool frame, done
        along the way. angular_speed (rad/s) limits it as speed limits the tip.
        """
        if not 0 < speed <= MAX_STRAIGHT_SPEED:
            raise ValueError(f"speed is in m/s, above 0 up to {MAX_STRAIGHT_SPEED}, got {speed}")
        if not 0 < angular_speed <= MAX_ANGULAR_SPEED:
            raise ValueError(f"angular_speed is in rad/s, above 0 up to {MAX_ANGULAR_SPEED}")
        if rotation is not None:
            if len(rotation) != 4 or not all(math.isfinite(v) for v in rotation) or \
                    abs(math.sqrt(sum(v * v for v in rotation)) - 1) > 1e-3:
                raise ValueError(f"rotation must be a unit quaternion (x, y, z, w), got {rotation}")
        if not all(math.isfinite(v) for v in offset):
            raise ValueError(f"offset must be finite, got {offset}")
        self._stop_asked = False
        stop = stop or Stop()
        for name in stop.signals():
            speed = min(speed, self._signal_max_speed(name))
        if frame not in ("tool", "base"):
            raise ValueError("frame must be tool or base")
        xyz, quat = self.tcp_pose()
        delta = _rotate(quat, offset) if frame == "tool" else offset
        target = Pose()
        target.position.x = xyz[0] + delta[0]
        target.position.y = xyz[1] + delta[1]
        target.position.z = xyz[2] + delta[2]
        o = target.orientation
        o.x, o.y, o.z, o.w = quat if rotation is None else _qmul(quat, rotation)
        request = GetCartesianPath.Request(
            group_name=GROUP, link_name=self.tcp, waypoints=[target], max_step=0.005,
            avoid_collisions=check_collisions, max_velocity_scaling_factor=1.0,
            max_acceleration_scaling_factor=1.0)
        request.header.frame_id = BASE
        request.start_state.is_diff = True
        answer = self._wait(self._cartesian.call_async(request))
        if answer.error_code.val != MoveItErrorCodes.SUCCESS:
            raise PlanningFailed(f"Cartesian path failed, code {answer.error_code.val}")
        if answer.fraction < 0.9999:
            raise PartialPath(f"only {answer.fraction:.0%} of the path is reachable")
        trajectory = answer.solution.joint_trajectory
        tips = self._tip_poses(trajectory)
        jump = path_check.find_jump([p.positions for p in trajectory.points], tips,
                                    (xyz, quat), ((target.position.x, target.position.y,
                                                   target.position.z), (o.x, o.y, o.z, o.w)))
        if jump:
            raise PartialPath(f"straight path refused: {jump}")
        return self._run(self._limit_tip_speed(trajectory, speed, angular_speed, tips), stop)

    def _tip_poses(self, trajectory):
        out = []
        for point in trajectory.points:
            request = GetPositionFK.Request(fk_link_names=[self.tcp])
            request.header.frame_id = BASE
            request.robot_state.joint_state.name = trajectory.joint_names
            request.robot_state.joint_state.position = list(point.positions)
            answer = self._wait(self._fk.call_async(request))
            if answer.error_code.val != MoveItErrorCodes.SUCCESS:
                raise PlanningFailed(f"FK failed, code {answer.error_code.val}")
            p, o = answer.pose_stamped[0].pose.position, answer.pose_stamped[0].pose.orientation
            out.append(((p.x, p.y, p.z), (o.x, o.y, o.z, o.w)))
        return out

    def _limit_tip_speed(self, trajectory, speed, angular_speed=DEFAULT_ANGULAR_SPEED, tips=None):
        """Stretches the timing so the tip stays under speed and its turn under angular_speed.

        Jazzy's compute_cartesian_path ignores max_cartesian_speed without an error.
        Scaling time keeps the path and scales every velocity by the same factor.
        """
        points = trajectory.points
        tips = tips if tips is not None else self._tip_poses(trajectory)
        seconds = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in points]
        stretch = 0.0
        for (a, qa), (b, qb), ta, tb in zip(tips, tips[1:], seconds, seconds[1:]):
            if tb > ta:
                turn = 2 * math.acos(min(1.0, abs(sum(x * y for x, y in zip(qa, qb)))))
                stretch = max(stretch, math.dist(a, b) / (tb - ta) / speed,
                              turn / (tb - ta) / angular_speed)
        if stretch <= 1.0:
            return trajectory
        for point, t in zip(points, seconds):
            scaled = Duration(seconds=t * stretch).to_msg()
            point.time_from_start = scaled
            point.velocities = [v / stretch for v in point.velocities]
            point.accelerations = [a / stretch ** 2 for a in point.accelerations]
        return trajectory

    def stop(self):
        """Stops the current move, even while it is still planning.

        The move's call returns reason 'stopped'.
        """
        self._stop_asked = True
        goal = self._move_goal
        if goal is not None:
            self._wait(goal.cancel_goal_async())

    # --- Internals -----------------------------------------------------------------------------
    def _signal_max_speed(self, name):
        """MAX_SIGNAL_SPEED, or lower if the tool says so: a slow signal (contact at 5.8 Hz) lets
        the tip travel further before the stop (tool.yaml stop_signals)."""
        spec = self.tool.get("stop_signals", {}).get(name)
        if isinstance(spec, dict) and "max_speed" in spec:
            return min(MAX_SIGNAL_SPEED, float(spec["max_speed"]))
        return MAX_SIGNAL_SPEED

    def _planned(self, goal, speed, tip_speed, angular_speed, planner="ompl"):
        if not math.isfinite(tip_speed) or not 0 < tip_speed <= MAX_PLANNED_TIP_SPEED:
            raise ValueError(f"tip_speed is in m/s, above 0 up to {MAX_PLANNED_TIP_SPEED}, got {tip_speed}")
        if not math.isfinite(angular_speed) or not 0 < angular_speed <= MAX_ANGULAR_SPEED:
            raise ValueError(f"angular_speed is in rad/s, above 0 up to {MAX_ANGULAR_SPEED}")
        trajectory = self._plan_to_constraints(goal, speed, planner)
        return self._limit_tip_speed(trajectory, tip_speed, angular_speed)

    def _plan_to_constraints(self, goal, speed, planner="ompl"):
        if not math.isfinite(speed) or not 0 < speed <= 1:
            raise ValueError(f"speed is a fraction of full speed, above 0 up to 1, got {speed}")
        if planner not in ("ompl", "ptp"):
            raise ValueError(f"planner is ompl or ptp, got {planner}")
        self._stop_asked = False
        self._check_guard()
        request = GetMotionPlan.Request()
        r = request.motion_plan_request
        r.group_name = GROUP
        r.num_planning_attempts = 4
        r.allowed_planning_time = 3.0
        r.max_velocity_scaling_factor = float(speed)
        r.max_acceleration_scaling_factor = float(speed)
        r.start_state.is_diff = True
        r.goal_constraints = [goal]
        if planner == "ptp":
            r.pipeline_id, r.planner_id, r.num_planning_attempts = "pilz", "PTP", 1
        for attempt in range(PLAN_RETRIES + 1):
            answer = self._wait(self._plan.call_async(request)).motion_plan_response
            code = answer.error_code.val
            if code == MoveItErrorCodes.SUCCESS:
                return answer.trajectory.joint_trajectory
            if code not in RETRY_CODES or attempt == PLAN_RETRIES:
                break
            self.plan_retries += 1
            self._node.get_logger().warning(
                f"plan rejected, code {code}, retry {attempt + 1} of {PLAN_RETRIES}")
        raise PlanningFailed(f"no plan, MoveIt code {code}")

    def _check_guard(self):
        if self._guard_lost.is_set():
            raise GuardLost("the motion guard is lost; connect again")

    def _run(self, trajectory, stop=None):
        self._check_guard()
        stop = stop or Stop()
        if self._stop_asked:
            return Result("stopped", 0.0, self.joints())
        goal = ExecuteTrajectory.Goal(
            trajectory=trajectory, bond_id=self._bond_id, force_limit_n=float(self.force_limit_n),
            stop_force_n=float(stop.force_n), stop_signals=stop.signals())
        handle = self._wait(self._execute.send_goal_async(goal))
        if not handle.accepted:
            raise ExecutionFailed("motion server refused the goal")
        self._move_goal = handle
        try:
            if self._stop_asked:
                self._wait(handle.cancel_goal_async())
            try:
                result = self._final_result(handle)
            except RobotError:
                handle.cancel_goal_async()   # don't leave it running; the error is what matters
                raise
        finally:
            self._move_goal = None
        return self._outcome(result)

    def _final_result(self, handle, unknown_limit=TIMEOUT_S):
        """Waits for the move's result, for as long as the motion guard holds.

        No time limit while the bond is up: at a low tablet slider a move can take many times
        its planned length, and motion_server's stall and overtime checks already end a move
        that stops progressing, in scaled time. A dead or hung server breaks the bond.

        A result request can reach the server before it has registered the goal. The answer is
        then status UNKNOWN with an empty result, and an empty result's outcome reads as DONE,
        so it asks again, for up to unknown_limit seconds.
        """
        unknown_since = None
        while True:
            answer = self._wait_guarded(handle.get_result_async())
            if answer.status != GoalStatus.STATUS_UNKNOWN:
                return answer.result
            unknown_since = unknown_since or time.monotonic()
            if time.monotonic() - unknown_since > unknown_limit:
                raise RobotError("timed out waiting for the robot stack")
            time.sleep(0.01)

    def _wait_guarded(self, future):
        """Waits for future while the motion guard holds; raises GuardLost once it breaks."""
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        while not done.wait(0.05):
            if self._guard_lost.is_set():
                raise GuardLost("the motion guard broke while waiting for the move; "
                                "motion_server stops the arm when it loses the guard")
        return future.result()

    def _outcome(self, r):
        joints = dict(zip(r.final_state.name, r.final_state.position))
        expected = {_Out.DONE: "done", _Out.STOPPED_FORCE: "force",
                    _Out.STOPPED_SIGNAL: r.stop_signal, _Out.STOPPED_REQUEST: "stopped"}
        if r.outcome in expected:
            return Result(expected[r.outcome], r.peak_force_n, joints)
        errors = {_Out.ABORTED_FORCE_LIMIT: ForceLimitExceeded,
                  _Out.ABORTED_SENSOR_STALE: SensorStale, _Out.ABORTED_GUARD_LOST: GuardLost,
                  _Out.ABORTED_STALLED: Stalled}
        raise errors.get(r.outcome, ExecutionFailed)(r.message or f"outcome {r.outcome}")

    def _wait(self, future, timeout=TIMEOUT_S):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            raise RobotError("timed out waiting for the robot stack")
        return future.result()


def path_to_dict(trajectory):
    """A joint trajectory as plain data, for a YAML file."""
    return {"joint_names": list(trajectory.joint_names),
            "points": [{"t": p.time_from_start.sec + p.time_from_start.nanosec * 1e-9,
                        "q": [float(v) for v in p.positions],
                        "v": [float(v) for v in p.velocities],
                        "a": [float(v) for v in p.accelerations]} for p in trajectory.points]}


def path_from_dict(data, reverse=False):
    """The trajectory path_to_dict saved; reverse=True runs it backwards (same timing)."""
    pts = data["points"]
    end = pts[-1]["t"]
    seq = list(reversed(pts)) if reverse else pts
    out = JointTrajectory(joint_names=list(data["joint_names"]))
    for p in seq:
        t = end - p["t"] if reverse else p["t"]
        point = JointTrajectoryPoint(positions=list(p["q"]))
        if p.get("v"):
            point.velocities = [-v for v in p["v"]] if reverse else list(p["v"])
        if p.get("a"):
            point.accelerations = list(p["a"])
        point.time_from_start = Duration(seconds=t).to_msg()
        out.points.append(point)
    # At rest at both ends: the controller refuses a last point with any velocity, and a planned
    # path's first point carries a tiny one (2e-4 rad/s), which becomes the last when reversed.
    for point in (out.points[0], out.points[-1]):
        point.velocities = [0.0] * len(point.positions) if point.velocities else []
        point.accelerations = [0.0] * len(point.positions) if point.accelerations else []
    return out
