"""Planning on mock hardware through MoveIt's action and scene interfaces, and the private
domain that keeps everything but motion_server away from the controllers."""
import os
import subprocess
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (BoundingVolume, CollisionObject, Constraints, JointConstraint,
                             MoveItErrorCodes, OrientationConstraint, PlanningScene,
                             PositionConstraint)
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tahoma_moveit_config import workcell as W
from tahoma_moveit_config.srdf import ARM_JOINTS

TIP = "gripper_equilibrium_grasp"
DOWN = (1.0, 0.0, 0.0, 0.0)      # cup pointing at the floor
FORWARD = (0.0, 0.7071068, 0.0, 0.7071068)  # cup pointing at the pod


class Client:
    def __init__(self):
        self.node = rclpy.create_node("mock_test")
        self.move = ActionClient(self.node, MoveGroup, "move_action")
        self.apply = self.node.create_client(ApplyPlanningScene, "apply_planning_scene")
        self.joints = {}
        self.node.create_subscription(JointState, "joint_states", self.on_joints, 10)
        self.wc = W.load(os.environ["WORKCELL"])
        self.placed = []
        # robot.launch.py starts move_group after the controller is active.
        assert self.move.wait_for_server(timeout_sec=60)
        assert self.apply.wait_for_service(timeout_sec=60)

    def on_joints(self, msg):
        self.joints.update(zip(msg.name, msg.position))

    def wait(self, future, timeout=60.0):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        assert future.done()
        return future.result()

    def scene(self, *names):
        """Replaces the scene, then checks MoveIt accepted it."""
        request = ApplyPlanningScene.Request(scene=PlanningScene(is_diff=True))
        for name in self.placed:
            request.scene.world.collision_objects.append(
                CollisionObject(id=name, operation=CollisionObject.REMOVE))
        self.placed = W.object_names(self.wc, names)
        for name in self.placed:
            request.scene.world.collision_objects.append(W.collision_object(self.wc, name))
        assert self.wait(self.apply.call_async(request)).success

    def run(self, goal, execute):
        request = MoveGroup.Goal()
        r = request.request
        r.group_name = "manipulator"
        r.num_planning_attempts = 4
        r.allowed_planning_time = 3.0
        r.max_velocity_scaling_factor = 0.5
        r.max_acceleration_scaling_factor = 0.5
        r.start_state.is_diff = True
        r.goal_constraints = [goal]
        request.planning_options.plan_only = not execute
        handle = self.wait(self.move.send_goal_async(request))
        assert handle.accepted
        return self.wait(handle.get_result_async()).result.error_code.val

    def joint_goal(self, pose):
        values = self.wc["named_poses"][pose]
        return Constraints(joint_constraints=[
            JointConstraint(joint_name=n, position=float(v), tolerance_above=0.001,
                            tolerance_below=0.001, weight=1.0)
            for n, v in zip(ARM_JOINTS, values, strict=True)])

    def tip_goal(self, xyz, quat):
        target = PoseStamped()
        target.header.frame_id = "base_link"
        target.pose.position.x, target.pose.position.y, target.pose.position.z = xyz
        o = target.pose.orientation
        o.x, o.y, o.z, o.w = quat
        region = BoundingVolume()
        region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.005]))
        region.primitive_poses.append(target.pose)
        position = PositionConstraint(header=target.header, link_name=TIP, weight=1.0,
                                      constraint_region=region)
        orientation = OrientationConstraint(header=target.header, link_name=TIP, orientation=o,
                                            absolute_x_axis_tolerance=0.02,
                                            absolute_y_axis_tolerance=0.02,
                                            absolute_z_axis_tolerance=0.02, weight=1.0)
        return Constraints(position_constraints=[position], orientation_constraints=[orientation])

    def at(self, pose, tolerance=0.01, timeout=5.0):
        target = dict(zip(ARM_JOINTS, self.wc["named_poses"][pose], strict=True))
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.joints and all(abs(self.joints[n] - v) < tolerance for n, v in target.items()):
                return True
        return False


@pytest.fixture(scope="module")
def client():
    rclpy.init()
    c = Client()
    yield c
    c.node.destroy_node()
    rclpy.shutdown()


@pytest.fixture(autouse=True)
def clear_scene(client):
    client.scene()


def test_starts_at_home(client):
    assert client.at("home")


def test_moveit_plans_but_cannot_execute(client):
    """MoveIt has no controllers: motion_server is the only way to move (T12 item 3)."""
    client.scene("table_down", "limits_drop_hide")
    assert client.run(client.joint_goal("drop_hide"), execute=False) == MoveItErrorCodes.SUCCESS
    assert client.run(client.joint_goal("drop_hide"), execute=True) != MoveItErrorCodes.SUCCESS
    time.sleep(0.5)
    assert client.at("home")


PRIVATE = os.environ.get("AURMR_PRIVATE_DOMAIN", "142")


def ros2(*args, domain=None, timeout=20):
    env = dict(os.environ)
    if domain is not None:
        env["ROS_DOMAIN_ID"] = domain
        env["CYCLONEDDS_URI"] = os.environ.get("AURMR_PRIVATE_URI", "")
    return subprocess.run(["ros2", *args], capture_output=True, text=True, timeout=timeout, env=env)


def test_controllers_unreachable_from_the_shared_domain(client):
    """On the shared domain nothing of ros2_control exists: no controller topic or action, no
    controller manager. One message on the controller's topic used to move the wrist 0.1 rad."""
    names = ros2("node", "list", "--no-daemon", "-a").stdout.split()
    assert "/motion_server" in names   # control: discovery worked
    # MoveIt's own /moveit_simple_controller_manager is fine: it has no controllers.
    leaks = [n for n in names if n == "/controller_manager" or "arm_controller" in n
             or "broadcaster" in n]
    assert leaks == [], names
    topics = ros2("topic", "list", "--no-daemon").stdout
    assert "/arm_controller/joint_trajectory" not in topics, topics
    assert "/joint_states" in topics   # copied out by motion_server
    actions = ros2("action", "list").stdout
    assert "/arm_controller/follow_joint_trajectory" not in actions, actions
    pub = client.node.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
    t = JointTrajectory(joint_names=ARM_JOINTS)
    point = JointTrajectoryPoint(positions=[client.joints[n] for n in ARM_JOINTS])
    point.positions[-1] += 0.1
    point.time_from_start.sec = 1
    t.points = [point]
    for _ in range(5):
        pub.publish(t)
        time.sleep(0.1)
    time.sleep(1.5)
    rclpy.spin_once(client.node, timeout_sec=0.2)
    assert client.at("home")
    client.node.destroy_publisher(pub)


def test_controllers_reachable_from_the_private_domain():
    """Control for the test above: the controllers are there, on the private domain."""
    out = ros2("control", "list_controllers", domain=PRIVATE).stdout
    assert "arm_controller" in out and "active" in out, out


@pytest.mark.parametrize("names, xyz, quat", [
    (["table_down"], (0.40, -0.10, 0.71), DOWN),
    (["limits_full"], (1.02, 0.0, 1.30), FORWARD),
], ids=["inside_table", "past_pod_limit"])
def test_goal_depends_on_scene(client, names, xyz, quat):
    """The same goal plans without the object and is refused with it."""
    goal = client.tip_goal(xyz, quat)
    assert client.run(goal, execute=False) == MoveItErrorCodes.SUCCESS
    client.scene(*names)
    assert client.run(goal, execute=False) != MoveItErrorCodes.SUCCESS


def test_scene_removal_restores_plan(client):
    goal = client.tip_goal((0.40, -0.10, 0.71), DOWN)
    client.scene("table_down")
    assert client.run(goal, execute=False) != MoveItErrorCodes.SUCCESS
    client.scene()
    assert client.run(goal, execute=False) == MoveItErrorCodes.SUCCESS
