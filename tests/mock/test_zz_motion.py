"""Robot API and motion server on mock hardware, with simulated force and stop signals.
Named test_zz so it runs after test_mock; the last test kills the stack."""
import math
import os
import signal
import subprocess
import sys
import threading
import time

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import Vector3
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from std_msgs.msg import String
from tahoma_interfaces.action import ExecuteTrajectory
from tahoma_robot import (ForceLimitExceeded, GuardLost, PartialPath, PlanningFailed, Robot,
                          SensorStale, Stop)

WORKCELL = os.environ["WORKCELL"]
LIMIT = 50.0
START = "pre_bin_3g"     # clear straight lines in every direction
SLOW = 0.03
BACK = (0, 0, -0.12)     # 4 s at SLOW
HERE = os.path.dirname(__file__)


class Sensors:
    """Controls fake_sensors.py, which runs as its own process."""

    def __init__(self):
        self.process = subprocess.Popen([sys.executable, os.path.join(HERE, "fake_sensors.py")])
        self.node = rclpy.create_node("fake_sensor_control")
        # Depth 10: several quick commands must all arrive, in order.
        latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.force_pub = self.node.create_publisher(Vector3, "/fake/force", latched)
        self.flags_pub = self.node.create_publisher(String, "/fake/flags", latched)

    def set_force(self, x=0.0, y=0.0, z=0.0):
        self.force_pub.publish(Vector3(x=float(x), y=float(y), z=float(z)))

    def set_flags(self, **flags):
        text = " ".join(f"{k}={int(v)}" for k, v in flags.items())
        self.flags_pub.publish(String(data=text))

    def reset(self):
        self.set_force()
        self.set_flags(wrench=True, signals=True, contact=False, seal=False, scaling_on=True,
                       scaling=100)

    def set_scaling(self, percent):
        self.flags_pub.publish(String(data=f"scaling={float(percent)}"))


@pytest.fixture(scope="module")
def sensors():
    if not rclpy.ok():
        rclpy.init()
    s = Sensors()
    time.sleep(1.0)
    yield s
    s.process.kill()


@pytest.fixture
def robot(sensors):
    sensors.reset()
    time.sleep(0.2)
    with Robot.connect("capcup", LIMIT, WORKCELL) as r:
        r.scene.use()
        r.go_named(START, speed=0.8)
        yield r
    time.sleep(0.8)   # the server must see the bond end before the next connect


def later(delay, action):
    t = threading.Timer(delay, action)
    t.daemon = True
    t.start()
    return t


def moved(a, b):
    return float(np.linalg.norm(np.subtract(a, b)))


def still(robot):
    """Watches the arm after a stop: reports how far and how long it kept moving."""
    start = robot.joints()
    t0 = time.monotonic()
    last_change, previous, furthest = t0, start, 0.0
    while time.monotonic() - t0 < 1.0:
        time.sleep(0.01)
        now = robot.joints()
        if max(abs(now[k] - previous[k]) for k in now) > 1e-6:
            last_change = time.monotonic()
        furthest = max(furthest, max(abs(now[k] - start[k]) for k in now))
        previous = now
    print(f"after stop: moved {furthest:.5f} rad for {last_change - t0:.3f} s")
    return last_change - t0 < 0.5 and furthest < 0.02


# --- API ---------------------------------------------------------------------------------------
def test_force_limit_is_required(sensors):
    with pytest.raises(ValueError):
        Robot.connect("capcup", None, WORKCELL)


def test_go_named(robot):
    assert robot.go_named("drop_hide").reason == "done"
    target = robot.workcell["named_poses"]["drop_hide"]
    from tahoma_moveit_config.srdf import ARM_JOINTS
    assert all(abs(robot.joints()[n] - v) < 0.01 for n, v in zip(ARM_JOINTS, target))


def test_unknown_or_unreachable_raise(robot):
    with pytest.raises(KeyError):
        robot.go_named("nope")
    with pytest.raises(PlanningFailed):
        robot.plan_to((3.0, 0, 1.0), (0, 0, 0, 1))


def test_plan_to(robot):
    xyz, quat = robot.tcp_pose()
    goal = (xyz[0] + 0.05, xyz[1], xyz[2] - 0.05)
    assert robot.plan_to(goal, quat).reason == "done"
    time.sleep(0.2)
    assert moved(robot.tcp_pose()[0], goal) < 0.01


def test_move_straight_is_straight_and_ends_at_goal(robot):
    start = robot.tcp_pose()
    assert robot.move_straight((0, 0, -0.05), speed=0.1).reason == "done"
    time.sleep(0.2)
    assert moved(robot.tcp_pose()[0], start[0]) == pytest.approx(0.05, abs=0.003)


def tip_speed_while(robot, move):
    """Peak tip speed over 0.1 s windows, from joint feedback timed by its own stamps.

    Timing TF lookups by the wall clock read up to 40% high at 500 Hz joint states (UR mock).
    """
    from moveit_msgs.srv import GetPositionFK
    from sensor_msgs.msg import JointState
    from tahoma_moveit_config.srdf import ARM_JOINTS
    msgs = []
    sub = robot._node.create_subscription(JointState, "joint_states", msgs.append, 1000)
    try:
        move()
    finally:
        robot._node.destroy_subscription(sub)
    samples, last = [], -1.0
    for m in msgs:
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if t - last < 0.02 or not set(ARM_JOINTS) <= set(m.name):
            continue
        last = t
        q = GetPositionFK.Request(fk_link_names=[robot.tcp])
        q.header.frame_id = "base_link"
        q.robot_state.joint_state.name = list(ARM_JOINTS)
        q.robot_state.joint_state.position = [m.position[m.name.index(n)] for n in ARM_JOINTS]
        p = robot._wait(robot._fk.call_async(q)).pose_stamped[0].pose.position
        samples.append((t, (p.x, p.y, p.z)))
    speeds = []
    for i, (ta, a) in enumerate(samples):
        later_samples = [(tb, b) for tb, b in samples[i:] if tb - ta >= 0.1]
        if later_samples:
            tb, b = later_samples[0]
            speeds.append(moved(a, b) / (tb - ta))
    return max(speeds)


@pytest.mark.parametrize("speed", [0.03, 0.1])
def test_tip_speed_limit_is_obeyed(robot, speed):
    """Jazzy's own max_cartesian_speed is silently ignored, so this measures the real motion."""
    peak = tip_speed_while(robot, lambda: robot.move_straight((0, 0, -0.12), speed=speed))
    assert 0.5 * speed < peak < 1.25 * speed


def test_stop_signal_caps_speed(robot, sensors):
    peak = tip_speed_while(
        robot, lambda: robot.move_straight((0, 0, -0.08), speed=0.2, stop=Stop(contact=True)))
    assert peak < 0.03 * 1.25


def test_partial_path_raises_and_nothing_moves(robot):
    before = robot.joints()
    with pytest.raises(PartialPath):
        robot.move_straight((0, 0, 3.0))
    assert max(abs(before[k] - robot.joints()[k]) for k in before) < 1e-3


def test_joint_jump_check_reaches_straight_moves(robot, monkeypatch):
    """With the per-step joint limit far below a normal step, an ordinary line is refused and
    nothing moves, so the check really runs on every straight move."""
    from tahoma_robot import path_check
    monkeypatch.setattr(path_check, "MAX_JOINT_STEP_RAD", 0.0005)
    before = robot.joints()
    with pytest.raises(PartialPath, match="rad per 5 mm"):
        robot.move_straight((0, 0, -0.05), speed=0.1)
    assert max(abs(before[k] - robot.joints()[k]) for k in before) < 1e-3


def test_joint_jump_check_on_made_up_paths():
    """MoveIt 2.12.4 ignores its own jump thresholds, so this check is ours (path_check.py)."""
    from tahoma_robot.path_check import find_jump
    q = (0.0, 0.0, 0.0, 1.0)
    line = [((0.005 * i, 0.0, 0.0), q) for i in range(21)]   # 10 cm in 5 mm steps
    good = [[0.01 * i] * 6 for i in range(21)]
    assert find_jump(good, line, line[0], line[-1]) is None
    flip = [row[:4] + [row[4] + (math.pi if i >= 10 else 0)] + row[5:]
            for i, row in enumerate(good)]
    assert "rad per 5 mm" in find_jump(flip, line, line[0], line[-1])
    off = list(line)
    off[10] = ((0.05, 0.02, 0.0), q)
    assert "leaves the line" in find_jump(good, off, line[0], line[-1])
    turn = [((0.0, 0.0, 0.0), (0.0, 0.0, math.sin(0.0125 * i), math.cos(0.0125 * i)))
            for i in range(21)]
    assert find_jump([[0.025 * i] * 6 for i in range(21)], turn, turn[0], turn[-1]) is None


def test_straight_move_respects_scene(robot):
    """The same line runs without the limits and is refused with them."""
    across = (0.25, 0, 0)
    assert robot.move_straight(across, frame="base", speed=0.2).reason == "done"
    robot.go_named(START, speed=0.8)
    robot.scene.use("limits_full")
    with pytest.raises(PlanningFailed):
        robot.move_straight(across, frame="base", speed=0.2)


def failing_plans(robot, code, times):
    """Makes the next plan requests come back with this code, then lets real ones through."""
    calls = []
    real = robot._plan.call_async

    def call(request):
        calls.append(request)
        if len(calls) > times:
            return real(request)
        future = Future()
        response = GetMotionPlan.Response()
        response.motion_plan_response.error_code.val = code
        future.set_result(response)
        return future

    robot._plan.call_async = call
    return calls


def test_rejected_plan_is_retried_and_counted(robot):
    calls = failing_plans(robot, MoveItErrorCodes.INVALID_MOTION_PLAN, 2)
    assert robot.go_named("drop_hide").reason == "done"
    assert len(calls) == 3
    assert robot.plan_retries == 2


def test_retries_are_limited(robot):
    calls = failing_plans(robot, MoveItErrorCodes.INVALID_MOTION_PLAN, 99)
    with pytest.raises(PlanningFailed):
        robot.go_named("drop_hide")
    assert len(calls) == 3


def test_other_planning_failures_are_not_retried(robot):
    calls = failing_plans(robot, MoveItErrorCodes.GOAL_IN_COLLISION, 99)
    with pytest.raises(PlanningFailed):
        robot.go_named("drop_hide")
    assert len(calls) == 1


# --- Stops -------------------------------------------------------------------------------------
def test_control_no_trigger_finishes(robot):
    start = robot.tcp_pose()[0]
    r = robot.move_straight(BACK, speed=SLOW, stop=Stop(force_n=15))
    assert r.reason == "done"
    time.sleep(0.2)
    assert moved(robot.tcp_pose()[0], start) == pytest.approx(0.12, abs=0.005)


def test_force_stop_returns_result(robot, sensors):
    start = robot.tcp_pose()[0]
    later(1.0, lambda: sensors.set_force(z=20.0))
    r = robot.move_straight(BACK, speed=SLOW, stop=Stop(force_n=15))
    assert r.reason == "force"
    assert r.peak_force_n >= 15
    assert still(robot)
    assert moved(robot.tcp_pose()[0], start) < 0.06


def test_session_limit_raises(robot, sensors):
    later(1.0, lambda: sensors.set_force(z=LIMIT + 10))
    with pytest.raises(ForceLimitExceeded):
        robot.move_straight(BACK, speed=SLOW)
    assert still(robot)


def test_sideways_force_of_equal_size_is_seen(robot, sensors):
    """Old bug: comparing magnitudes misses this."""
    sensors.set_force(z=30.0)
    time.sleep(0.2)
    later(1.0, lambda: sensors.set_force(x=30.0))
    assert robot.move_straight(BACK, speed=SLOW, stop=Stop(force_n=15)).reason == "force"


def test_limit_is_relative_to_pre_move_reading(robot, sensors):
    sensors.set_force(z=40.0)
    time.sleep(0.2)
    assert robot.move_straight((0, 0, -0.03), speed=0.1, stop=Stop(force_n=15)).reason == "done"


def test_stop_still_works_late_in_a_long_move(robot, sensors):
    """Old bug: monitoring gave up after 5 s."""
    later(6.0, lambda: sensors.set_force(z=20.0))
    r = robot.move_straight((0, 0, -0.16), speed=0.02, stop=Stop(force_n=15))
    assert r.reason == "force"


@pytest.mark.parametrize("name", ["contact", "seal"])
def test_stop_signal_returns_result(robot, sensors, name):
    start = robot.tcp_pose()[0]
    later(1.0, lambda: sensors.set_flags(**{name: True}))
    r = robot.move_straight(BACK, speed=SLOW, stop=Stop(**{name: True}))
    assert r.reason == name
    assert still(robot)
    assert moved(robot.tcp_pose()[0], start) < 0.06


def test_signal_not_asked_for_is_ignored(robot, sensors):
    later(0.2, lambda: sensors.set_flags(seal=True))
    r = robot.move_straight((0, 0, -0.03), speed=0.1, stop=Stop(contact=True))
    assert r.reason == "done"


def test_robot_stop(robot):
    later(1.0, robot.stop)
    assert robot.move_straight(BACK, speed=SLOW).reason == "stopped"
    assert still(robot)


# --- Validation and stale sensors --------------------------------------------------------------
def execute_directly(robot, trajectory, force_limit=LIMIT):
    goal = ExecuteTrajectory.Goal(trajectory=trajectory, bond_id=robot._bond_id,
                                  force_limit_n=float(force_limit))
    handle = robot._wait(robot._execute.send_goal_async(goal))
    return robot._final_result(handle)


def a_plan(robot):
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from tahoma_moveit_config.srdf import ARM_JOINTS
    now = robot.joints()
    t = JointTrajectory(joint_names=ARM_JOINTS)
    p = JointTrajectoryPoint(positions=[now[n] for n in ARM_JOINTS])
    q = JointTrajectoryPoint(positions=[now[n] for n in ARM_JOINTS])
    q.positions[0] += 0.02
    q.time_from_start.sec = 1
    t.points = [p, q]
    return t


def test_control_direct_goal_with_good_start_runs(robot):
    assert execute_directly(robot, a_plan(robot)).outcome == ExecuteTrajectory.Result.DONE


def test_trajectory_not_at_current_joints_refused(robot):
    t = a_plan(robot)
    t.points[0].positions[0] += 0.1
    r = execute_directly(robot, t)
    assert r.outcome == ExecuteTrajectory.Result.REJECTED
    assert "current joints" in r.message


def test_missing_force_limit_refused(robot):
    r = execute_directly(robot, a_plan(robot), force_limit=0.0)
    assert r.outcome == ExecuteTrajectory.Result.REJECTED
    assert "force_limit_n" in r.message


def test_second_move_refused_while_moving(robot):
    results = []
    later(0.7, lambda: results.append(execute_directly(robot, a_plan(robot))))
    assert robot.move_straight(BACK, speed=SLOW).reason == "done"
    assert results[0].outcome == ExecuteTrajectory.Result.REJECTED
    assert "another move" in results[0].message


def test_wrench_going_silent_aborts(robot, sensors):
    later(1.0, lambda: sensors.set_flags(wrench=False))
    with pytest.raises(SensorStale):
        robot.move_straight(BACK, speed=SLOW)
    assert still(robot)


def test_silent_wrench_refused_before_moving(robot, sensors):
    sensors.set_flags(wrench=False)
    time.sleep(0.25)
    r = execute_directly(robot, a_plan(robot))
    assert r.outcome == ExecuteTrajectory.Result.REJECTED


def test_requested_signal_silent_refused(robot, sensors):
    sensors.set_flags(signals=False)
    time.sleep(0.3)
    from tahoma_robot import ExecutionFailed
    with pytest.raises(ExecutionFailed):
        robot.move_straight((0, 0, -0.03), stop=Stop(contact=True))


# --- Guard -------------------------------------------------------------------------------------
def demo(sensors):
    env = dict(os.environ)
    return subprocess.Popen([sys.executable, os.path.join(HERE, "long_move_demo.py")], env=env,
                            stdout=subprocess.PIPE, text=True)


def watcher():
    return Robot("capcup", LIMIT, WORKCELL)


def test_no_guard_no_motion(sensors):
    with watcher() as w:
        assert w._execute.wait_for_server(timeout_sec=10)
        time.sleep(0.3)
        r = execute_directly(w, a_plan(w))
        assert r.outcome == ExecuteTrajectory.Result.REJECTED
        assert "no demo holds" in r.message


def test_kill_demo_stops_arm_and_guard_can_reconnect(sensors):
    sensors.reset()
    p = demo(sensors)
    assert p.stdout.readline().strip() == "moving"
    with watcher() as w:
        time.sleep(1.0)
        a = w.joints()
        time.sleep(0.2)
        assert max(abs(a[k] - w.joints()[k]) for k in a) > 1e-4, "arm must be moving before the kill"
        p.send_signal(signal.SIGKILL)
        p.wait()
        killed = time.monotonic()
        last_change, previous = killed, w.joints()
        while time.monotonic() - killed < 2.0:
            time.sleep(0.01)
            now = w.joints()
            if max(abs(now[k] - previous[k]) for k in now) > 1e-6:
                last_change = time.monotonic()
            previous = now
        stopped_after = last_change - killed
        print(f"stopped {stopped_after:.2f} s after kill")
        assert 0.0 < stopped_after < 1.0
        assert execute_directly(w, a_plan(w)).outcome == ExecuteTrajectory.Result.REJECTED
    time.sleep(0.5)
    with Robot.connect("capcup", LIMIT, WORKCELL) as again:
        assert again.move_straight((0, 0, 0.02), speed=0.1).reason == "done"


def test_second_demo_refused_while_guard_held(sensors):
    sensors.reset()
    with Robot.connect("capcup", LIMIT, WORKCELL):
        with pytest.raises(GuardLost):
            Robot.connect("capcup", LIMIT, WORKCELL)
    time.sleep(0.8)


def test_zz_kill_motion_server_shuts_the_stack_down(sensors):
    subprocess.run(["pkill", "-9", "-f", "tahoma_control/motion_server"], check=True)
    time.sleep(3.0)
    procs = subprocess.run(["pgrep", "-f", "ros2_control_node|move_group"],
                           capture_output=True, text=True).stdout.split()
    assert procs == []
