"""The real-arm bringup against URSim. Runs in order: the arm is powered and moved first."""
import os
import signal
import subprocess
import sys
import time

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger
from tahoma_interfaces.action import ExecuteTrajectory
from tahoma_moveit_config.srdf import ARM_JOINTS
from tahoma_robot import ExecutionFailed, Robot, RobotError, Stalled, Stop
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ur_msgs.srv import SetSpeedSliderFraction

WORKCELL = os.environ["WORKCELL"]
LIMIT = 50.0
START = "pre_bin_3g"
HERE = os.path.dirname(__file__)


class Stack:
    """Dashboard and driver services, as an operator on the control host would use them: they
    are on the control host's private domain (robot.launch.py private_domain)."""

    def __init__(self):
        self.context = Context()
        rclpy.init(context=self.context,
                   domain_id=int(os.environ.get("AURMR_PRIVATE_DOMAIN", "142")))
        self.node = rclpy.create_node("ursim_test", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)

    def call(self, srv_type, name, request, timeout=30.0):
        client = self.node.create_client(srv_type, name)
        assert client.wait_for_service(timeout_sec=timeout), name
        future = client.call_async(request)
        self.executor.spin_until_future_complete(future, timeout_sec=timeout)
        assert future.done(), name
        return future.result()

    def trigger(self, name):
        return self.call(Trigger, name, Trigger.Request())

    def slider(self, fraction):
        r = self.call(SetSpeedSliderFraction, "/io_and_status_controller/set_speed_slider",
                      SetSpeedSliderFraction.Request(speed_slider_fraction=float(fraction)))
        assert r.success

    def start_program(self):
        assert self.trigger("/io_and_status_controller/resend_robot_program").success
        time.sleep(2.0)


@pytest.fixture(scope="module")
def stack():
    s = Stack()
    for name in ("/dashboard_client/power_on", "/dashboard_client/brake_release"):
        assert s.trigger(name).success, name
        time.sleep(5.0)
    s.start_program()
    time.sleep(3.0)   # controllers come back once the program runs
    yield s


@pytest.fixture
def robot(stack):
    stack.slider(1.0)
    with Robot.connect("capcup", LIMIT, WORKCELL) as r:
        yield r
    time.sleep(0.8)


def joints_trajectory(robot, target, seconds):
    now = robot.joints()
    t = JointTrajectory(joint_names=ARM_JOINTS)
    t.points = [JointTrajectoryPoint(positions=[now[n] for n in ARM_JOINTS]),
                JointTrajectoryPoint(positions=[float(v) for v in target])]
    t.points[1].time_from_start.sec = seconds
    return t


def test_0_bring_arm_to_a_known_pose(robot):
    """URSim starts in its own pose, which may collide with our frame model, so the first move
    goes straight through motion_server, without MoveIt. Only safe because this is a simulator."""
    target = robot.workcell["named_poses"][START]
    goal = ExecuteTrajectory.Goal(trajectory=joints_trajectory(robot, target, 15),
                                  bond_id=robot._bond_id, force_limit_n=LIMIT)
    handle = robot._wait(robot._execute.send_goal_async(goal))
    r = robot._wait(handle.get_result_async(), 60).result
    assert r.outcome == ExecuteTrajectory.Result.DONE, r.message
    assert max(abs(robot.joints()[n] - v) for n, v in zip(ARM_JOINTS, target)) < 0.01


def test_1_planned_and_straight_moves(robot):
    assert robot.go_named("drop_hide", speed=0.5).reason == "done"
    assert robot.go_named(START, speed=0.5).reason == "done"
    start, _ = robot.tcp_pose()
    assert robot.move_straight((0, 0, -0.02)).reason == "done"
    end, _ = robot.tcp_pose()
    assert 0.018 < sum((a - b) ** 2 for a, b in zip(start, end)) ** 0.5 < 0.022


def test_2_force_reading_is_live(robot):
    """URSim's force sensor reads near zero but must arrive, or every move would be refused."""
    assert robot.move_straight((0, 0, 0.02), stop=Stop(force_n=20)).reason == "done"


def test_3_slider_at_zero_stalls(robot, stack):
    t = time.monotonic()
    # The driver accepts 0.01 at the lowest; the tablet slider goes to 0.
    timer = __import__("threading").Timer(1.0, lambda: stack.slider(0.01))
    timer.start()
    try:
        with pytest.raises(Stalled, match="slider"):
            robot.move_straight((0, 0, -0.1), speed=0.03)
    finally:
        timer.join()
        stack.slider(1.0)
    took = time.monotonic() - t
    print(f"stalled after {took:.1f} s")
    assert 3.5 < took < 7.0


def test_4_slow_slider_still_finishes(robot, stack):
    """Half speed doubles the time, and that must not count as a stall or overtime."""
    stack.slider(0.5)
    t = time.monotonic()
    assert robot.move_straight((0, 0, 0.1), speed=0.03).reason == "done"
    print(f"0.1 m at 0.03 m/s with slider 0.5 took {time.monotonic() - t:.1f} s")
    stack.slider(1.0)


def test_4b_low_slider_long_move_finishes(robot, stack):
    """S4 runs at slider 10 to 20%. Robot used to give up at planned + 30 s (T12 item 2)."""
    stack.slider(0.1)
    t = time.monotonic()
    try:
        assert robot.move_straight((0, 0, -0.05), speed=0.01).reason == "done"
    finally:
        stack.slider(1.0)
    took = time.monotonic() - t
    print(f"5 cm at 0.01 m/s with slider 0.1 took {took:.1f} s")
    assert took > 35.0   # past the old limit
    assert robot.move_straight((0, 0, 0.05), speed=0.03).reason == "done"


def test_5_robot_stop_on_the_real_controller(robot):
    timer = __import__("threading").Timer(1.5, robot.stop)
    timer.start()
    before, _ = robot.tcp_pose()
    assert robot.move_straight((0, 0, -0.1), speed=0.03).reason == "stopped"
    timer.join()
    time.sleep(0.5)
    a, _ = robot.tcp_pose()
    time.sleep(0.3)
    b, _ = robot.tcp_pose()
    assert sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5 < 1e-4, "still moving after stop"


def test_6_program_stopped_mid_move_raises(robot, stack):
    """Pressing stop on the tablet: the controller stopper drops the controller."""
    timer = __import__("threading").Timer(1.0, lambda: stack.trigger("/dashboard_client/stop"))
    timer.start()
    try:
        with pytest.raises((ExecutionFailed, RobotError)) as e:
            robot.move_straight((0, 0, 0.1), speed=0.03)
        print(f"program stopped mid-move: {type(e.value).__name__}: {e.value}")
    finally:
        timer.join()
        stack.start_program()
        time.sleep(3.0)
    assert robot.move_straight((0, 0, -0.01)).reason == "done"


def test_7_killed_demo_stops_the_arm(stack):
    demo = subprocess.Popen([sys.executable, os.path.join(HERE, "long_move.py")],
                            stdout=subprocess.PIPE, text=True)
    try:
        assert demo.stdout.readline().strip() == "moving"
        with Robot("capcup", LIMIT, WORKCELL) as watch:
            time.sleep(1.0)
            demo.send_signal(signal.SIGKILL)
            demo.wait()
            killed = time.monotonic()
            last, moving_until = watch.joints(), killed
            while time.monotonic() - killed < 3.0:
                time.sleep(0.01)
                now = watch.joints()
                if max(abs(now[k] - last[k]) for k in now) > 1e-5:
                    moving_until = time.monotonic()
                last = now
            took = moving_until - killed
            print(f"arm stopped {took:.2f} s after kill -9 of the demo")
            assert took < 1.0
    finally:
        demo.kill()
    time.sleep(1.0)


def test_9_payload_step(stack):
    script = os.path.join(HERE, "..", "arm", "s4_steps.py")
    refused = subprocess.run([sys.executable, script, "payload", "--mass", "20"],
                             capture_output=True, text=True, timeout=60)
    assert refused.returncode != 0 and "implausible" in refused.stderr
    from_yaml = subprocess.run([sys.executable, script, "payload"], capture_output=True,
                               text=True, timeout=60)
    print(from_yaml.stdout, from_yaml.stderr[-500:])
    assert from_yaml.returncode == 0 and "RESULT payload 0.205 kg" in from_yaml.stdout
    done = subprocess.run([sys.executable, script, "payload", "--mass", "1.2", "--cog", "0", "0",
                           "0.1"], capture_output=True, text=True, timeout=60)
    print(done.stdout, done.stderr[-500:])
    assert done.returncode == 0 and "RESULT payload 1.2 kg" in done.stdout


@pytest.mark.parametrize("step", ["check", "wrist", "named", "straight", "force_stop", "stop"])
def test_8_s4_steps_rehearsal(stack, step):
    """The S4 step script, run against the simulator so the first real run isn't its first run."""
    stack.slider(1.0)
    done = subprocess.run([sys.executable, os.path.join(HERE, "..", "arm", "s4_steps.py"), step],
                          capture_output=True, text=True, timeout=120)
    print(done.stdout)
    assert done.returncode == 0, done.stderr[-2000:]
    assert "RESULT" in done.stdout
    if step in ("wrist", "named", "straight"):
        assert done.stdout.count("RESULT done") >= 1
    if step == "force_stop":
        assert "reason done" in done.stdout   # nobody presses in the simulator
    if step == "stop":
        assert "reason stopped" in done.stdout
    time.sleep(1.0)


def test_9b_driver_services_not_on_the_shared_domain(stack):
    """From the robot network, the dashboard and driver services don't exist (T12 item 3)."""
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("ursim_stranger")
    try:
        time.sleep(2.0)
        names = {n for n, _ in node.get_service_names_and_types()}
        assert not any(n.startswith(("/dashboard_client", "/io_and_status_controller",
                                     "/controller_manager")) for n in names), sorted(names)
        assert "/motion_server/connect" in names
    finally:
        node.destroy_node()
