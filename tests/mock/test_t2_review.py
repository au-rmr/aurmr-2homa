"""Holes found in the S3 review (board T5). Each test states the correct behaviour.
All are fixed; these keep them fixed."""
import math
import os
import signal
import subprocess
import threading
import time

import numpy as np
import pytest
from action_msgs.msg import GoalStatus
from rclpy.task import Future
from tahoma_interfaces.action import ExecuteTrajectory
from tahoma_robot import (ExecutionFailed, ForceLimitExceeded, GuardLost, Robot, RobotError,
                          SensorStale, Stop)
from tahoma_robot import robot as robot_module
from tahoma_robot.robot import _rotate

from test_zz_motion import (BACK, LIMIT, SLOW, START, WORKCELL, a_plan, demo, execute_directly,
                            later, moved, robot, sensors, watcher)  # noqa: F401

Out = ExecuteTrajectory.Result
# The controllers are on the control host's private domain: tests reach them as an operator on
# that host would.
PRIVATE_ENV = dict(os.environ, ROS_DOMAIN_ID=os.environ.get("AURMR_PRIVATE_DOMAIN", "142"),
                   CYCLONEDDS_URI=os.environ.get("AURMR_PRIVATE_URI", ""))
GetResult = ExecuteTrajectory.Impl.GetResultService.Response


def freeze_controller(sig):
    subprocess.run(["pkill", sig, "-f", "lib/controller_manager/ros2_control_node"], check=True)


@pytest.fixture
def thaw():
    yield
    freeze_controller("-CONT")
    time.sleep(0.5)


def test_client_without_a_bond_cannot_move_while_a_demo_holds_the_guard(sensors):
    sensors.reset()
    time.sleep(0.2)
    with Robot.connect("capcup", LIMIT, WORKCELL):
        with watcher() as stranger:
            assert stranger._execute.wait_for_server(timeout_sec=10)
            time.sleep(0.3)
            r = execute_directly(stranger, a_plan(stranger))
            assert r.outcome == Out.REJECTED
            assert "does not hold" in r.message
    time.sleep(0.8)


def test_infinite_force_limit_refused_by_server(robot):
    r = execute_directly(robot, a_plan(robot), force_limit=math.inf)
    assert r.outcome == Out.REJECTED
    assert "force_limit_n" in r.message


def test_infinite_force_limit_refused_by_robot(sensors):
    with pytest.raises(ValueError):
        Robot("capcup", math.inf, WORKCELL)


def test_nan_wrench_before_the_move_does_not_blind_the_force_check(robot, sensors):
    """One NaN in the baseline makes every later comparison false."""
    sensors.set_force(z=math.nan)
    time.sleep(0.2)
    later(1.0, lambda: sensors.set_force(z=500.0))
    with pytest.raises(RobotError):
        r = robot.move_straight(BACK, speed=SLOW)
        print(f"500 N during the move, reason {r.reason}, peak {r.peak_force_n}")


def test_nan_wrench_during_a_move_stops_it(robot, sensors):
    later(1.0, lambda: sensors.set_force(z=math.nan))
    with pytest.raises(RobotError):
        r = robot.move_straight(BACK, speed=SLOW)
        print(f"NaN during the move, reason {r.reason}")


@pytest.mark.parametrize("speed", [-0.01, 0.0, math.nan, math.inf])
def test_bad_straight_speed_refused(robot, speed):
    with pytest.raises(ValueError):
        robot.move_straight((0, 0, -0.02), speed=speed)


@pytest.mark.parametrize("speed", [-0.1, 0.0, math.nan, math.inf, 1.5])
def test_bad_planned_speed_refused(robot, speed):
    with pytest.raises(ValueError):
        robot.go_named("drop_hide", speed=speed)


@pytest.mark.parametrize("force", [-5.0, math.nan, math.inf])
def test_bad_stop_force_refused(robot, force):
    with pytest.raises(ValueError):
        Stop(force_n=force)
    goal = ExecuteTrajectory.Goal(trajectory=a_plan(robot), bond_id=robot._bond_id,
                                  force_limit_n=LIMIT, stop_force_n=force)
    handle = robot._wait(robot._execute.send_goal_async(goal))
    r = robot._wait(handle.get_result_async()).result
    assert r.outcome == Out.REJECTED
    assert "stop_force_n" in r.message


def test_controller_failing_mid_move_is_not_done(robot):
    """A controller that drops the goal, as a protective stop may, must raise."""
    def switch(*args):
        subprocess.run(["ros2", "control", "switch_controllers", *args], check=True, timeout=30,
                       env=PRIVATE_ENV)

    later(1.0, lambda: switch("--deactivate", "arm_controller"))
    try:
        with pytest.raises(ExecutionFailed, match="trajectory controller"):
            robot.move_straight(BACK, speed=SLOW)
    finally:
        time.sleep(0.5)
        switch("--activate", "arm_controller")
        time.sleep(1.0)


def test_goal_the_controller_refuses_does_not_block_the_next_move(robot):
    def switch(*args):
        subprocess.run(["ros2", "control", "switch_controllers", *args], check=True, timeout=30,
                       env=PRIVATE_ENV)

    switch("--deactivate", "arm_controller")
    try:
        r = execute_directly(robot, a_plan(robot))
        assert r.outcome == Out.FAILED
        assert "refused" in r.message
    finally:
        switch("--activate", "arm_controller")
        time.sleep(1.0)
    assert execute_directly(robot, a_plan(robot)).outcome == Out.DONE


def test_goal_refused_in_the_send_timeout_does_not_run_later(robot, sensors, thaw):
    """The server reports FAILED after 2 s, but the goal is still on its way to the controller."""
    plan = a_plan(robot)
    plan.points[1].positions[0] = plan.points[0].positions[0] + 0.2
    plan.points[1].time_from_start.sec = 2
    with watcher() as w:
        time.sleep(0.5)
        freeze_controller("-STOP")
        r = execute_directly(robot, plan)
        print(f"outcome {r.outcome}: {r.message}")
        assert r.outcome != Out.DONE
        before = dict(w.joints())
        freeze_controller("-CONT")
        time.sleep(2.5)
        drift = max(abs(before[k] - w.joints()[k]) for k in before)
        print(f"arm moved {drift:.4f} rad of the 0.2 planned")
        assert drift < 0.02
    assert execute_directly(robot, a_plan(robot)).outcome == Out.DONE


def test_unconfirmed_stop_is_not_reported_as_a_normal_stop(robot, sensors, thaw):
    later(1.0, lambda: freeze_controller("-STOP"))
    later(1.2, lambda: sensors.set_force(z=20.0))
    with pytest.raises(ExecutionFailed):
        robot.move_straight(BACK, speed=SLOW, stop=Stop(force_n=15))
    r = execute_directly(robot, a_plan(robot))
    assert r.outcome == Out.REJECTED
    assert "not confirmed" in r.message
    freeze_controller("-CONT")
    time.sleep(0.5)
    assert execute_directly(robot, a_plan(robot)).outcome == Out.DONE


def test_session_limit_passed_during_an_expected_stop_still_raises(robot, sensors, thaw):
    later(1.0, lambda: freeze_controller("-STOP"))
    later(1.1, lambda: sensors.set_force(z=20.0))
    later(1.3, lambda: sensors.set_force(z=LIMIT + 50))
    later(1.6, lambda: freeze_controller("-CONT"))
    with pytest.raises(ForceLimitExceeded):
        robot.move_straight(BACK, speed=SLOW, stop=Stop(force_n=15))


def test_cancel_right_after_accept_is_not_lost(robot):
    """The cancel can land before run() clears stop_reason_. The window is too small to hit at
    normal timing, so this only fails with a sleep added before validate() (40 of 40 lost)."""
    lost = 0
    for _ in range(40):
        goal = ExecuteTrajectory.Goal(trajectory=a_plan(robot), bond_id=robot._bond_id,
                                      force_limit_n=LIMIT)
        handle = robot._wait(robot._execute.send_goal_async(goal))
        robot._wait(handle.cancel_goal_async())
        r = robot._wait(handle.get_result_async()).result
        lost += r.outcome == Out.DONE
    print(f"cancel lost {lost} of 40")
    assert lost == 0


def test_reconnect_straight_after_close(sensors):
    sensors.reset()
    time.sleep(0.2)
    with Robot.connect("capcup", LIMIT, WORKCELL):
        pass
    with Robot.connect("capcup", LIMIT, WORKCELL):
        pass
    time.sleep(0.8)


def test_tool_frame_move_follows_the_tool_axis_in_a_line(robot):
    """test_move_straight_is_straight_and_ends_at_goal only measures the distance."""
    start, quat = robot.tcp_pose()
    offset = (0.03, 0.0, -0.05)
    delta = _rotate(quat, offset)
    goal = tuple(a + b for a, b in zip(start, delta))
    assert moved(delta, offset) > 0.02, "the tool frame must differ from base here"
    samples, done = [], threading.Event()

    def watch():
        while not done.is_set():
            samples.append(robot.tcp_pose()[0])
            time.sleep(0.02)

    t = threading.Thread(target=watch, daemon=True)
    t.start()
    assert robot.move_straight(offset, speed=0.05).reason == "done"
    done.set()
    t.join()
    time.sleep(0.2)
    assert moved(robot.tcp_pose()[0], goal) < 0.003
    assert len(samples) > 20
    line = np.subtract(goal, start)
    line = line / np.linalg.norm(line)
    off_line = [np.linalg.norm(np.cross(np.subtract(p, start), line)) for p in samples]
    assert max(off_line) < 0.003


def test_requested_signal_going_silent_aborts(robot, sensors):
    later(1.0, lambda: sensors.set_flags(signals=False))
    with pytest.raises(SensorStale):
        robot.move_straight(BACK, speed=SLOW, stop=Stop(contact=True))


def test_signal_already_on_means_no_motion(robot, sensors):
    """Passes even without the check at the start of run(): the signal's next tick stops it too."""
    sensors.set_flags(contact=True)
    time.sleep(0.2)
    before = robot.joints()
    assert robot.move_straight((0, 0, -0.03), stop=Stop(contact=True)).reason == "contact"
    time.sleep(0.2)
    assert max(abs(before[k] - robot.joints()[k]) for k in before) < 1e-4


def test_guard_cannot_be_switched_off_from_outside(sensors):
    """bondcpp reads this parameter when a bond starts, so it disables the next demo's guard."""
    sensors.reset()
    time.sleep(0.2)
    with Robot.connect("capcup", LIMIT, WORKCELL):  # the parameter only exists during a bond
        done = subprocess.run(["ros2", "param", "set", "/motion_server",
                               "/bond_disable_heartbeat_timeout", "true"],
                              capture_output=True, text=True, timeout=30)
        assert "successful" not in done.stdout
    time.sleep(0.8)
    p = demo(sensors)
    assert p.stdout.readline().strip() == "moving"
    with watcher() as w:
        time.sleep(1.0)
        p.send_signal(signal.SIGKILL)
        p.wait()
        time.sleep(1.5)
        a = dict(w.joints())
        time.sleep(0.2)
        still_moving = max(abs(a[k] - w.joints()[k]) for k in a)
        print(f"demo killed 1.7 s ago, arm moved {still_moving:.5f} rad in the last 0.2 s")
        assert still_moving < 1e-4


def test_slow_move_has_no_client_time_limit(robot, sensors, thaw, monkeypatch):
    """At a low slider a move takes many times its planned length: Robot must keep waiting
    while the guard holds (T12 item 2; it used to give up at planned + TIMEOUT_S). Scaling at
    10% with a frozen controller stands in for the slow arm, as in the overtime tests."""
    monkeypatch.setattr(robot_module, "TIMEOUT_S", 0.5)
    sensors.set_scaling(10)
    time.sleep(0.3)
    later(0.3, lambda: freeze_controller("-STOP"))
    later(4.0, lambda: freeze_controller("-CONT"))
    t0 = time.monotonic()
    try:
        r = robot._run(a_plan(robot))
    finally:
        sensors.set_scaling(100)
        time.sleep(0.3)
    assert r.reason == "done"
    assert time.monotonic() - t0 > 3.5


def test_guard_lost_while_waiting_raises_and_cancels(robot):
    """The bond breaking mid-move ends the wait and cancels the move."""
    later(1.0, robot._guard_lost.set)
    with pytest.raises(GuardLost, match="while waiting"):
        robot.move_straight(BACK, speed=SLOW)
    time.sleep(0.5)
    before = robot.joints()
    time.sleep(0.2)
    assert max(abs(before[k] - robot.joints()[k]) for k in before) < 1e-4


class FakeHandle:
    """Answers with an empty UNKNOWN result first, as the server does when a request overtakes
    the goal, then with the real one."""

    def __init__(self, unknown_answers):
        self.answers = [GetResult(status=GoalStatus.STATUS_UNKNOWN)] * unknown_answers
        self.answers.append(GetResult(status=GoalStatus.STATUS_SUCCEEDED,
                                      result=Out(outcome=Out.ABORTED_FORCE_LIMIT)))

    def get_result_async(self):
        future = Future()
        future.set_result(self.answers.pop(0))
        return future


def test_empty_result_is_not_taken_for_done(robot):
    """5 in 23578 goals under load got this answer, and the move returned done at once."""
    assert robot._final_result(FakeHandle(2)).outcome == Out.ABORTED_FORCE_LIMIT


def test_result_that_stays_empty_raises(robot):
    with pytest.raises(RobotError, match="timed out"):
        robot._final_result(FakeHandle(10 ** 4), unknown_limit=0.05)


@pytest.mark.parametrize("point,bad", [(0, math.nan), (1, math.nan), (1, math.inf)])
def test_non_finite_positions_refused(robot, point, bad):
    plan = a_plan(robot)
    plan.points[point].positions[0] = bad
    r = execute_directly(robot, plan)
    assert r.outcome == Out.REJECTED
    assert "not finite" in r.message


def test_point_missing_a_joint_refused(robot):
    plan = a_plan(robot)
    plan.points[1].positions = plan.points[1].positions[:-1]
    r = execute_directly(robot, plan)
    assert r.outcome == Out.REJECTED
    assert "does not match" in r.message


@pytest.mark.parametrize("speed", [0.3, 1000.0])
def test_straight_speed_has_a_maximum(robot, speed):
    with pytest.raises(ValueError):
        robot.move_straight((0, 0, -0.02), speed=speed)


@pytest.mark.parametrize("limit", [151.0, 1e9])
def test_force_limit_has_a_maximum(robot, sensors, limit):
    with pytest.raises(ValueError):
        Robot("capcup", limit, WORKCELL)
    r = execute_directly(robot, a_plan(robot), force_limit=limit)
    assert r.outcome == Out.REJECTED
    assert "force_limit_n" in r.message


def test_empty_result_is_not_done():
    """An empty result must never read as success, whatever path delivers it."""
    assert ExecuteTrajectory.Result().outcome == Out.UNSET != Out.DONE


def test_second_connect_while_the_first_is_forming_is_refused(sensors):
    from tahoma_interfaces.srv import Connect
    with watcher() as w:
        client = w._connect
        assert client.wait_for_service(timeout_sec=10)
        first = w._wait(client.call_async(Connect.Request(bond_id="never-forms")))
        second = w._wait(client.call_async(Connect.Request(bond_id="other")))
        assert first.accepted
        assert not second.accepted and "connecting" in second.message
    time.sleep(6.0)   # the bond that never formed must free the guard after the connect timeout
    with Robot.connect("capcup", LIMIT, WORKCELL):
        pass
    time.sleep(0.8)


def test_stop_while_planning_is_not_lost(robot):
    before = robot.joints()
    later(0.02, robot.stop)
    assert robot.go_named("drop_hide").reason == "stopped"
    time.sleep(0.2)
    assert max(abs(before[k] - robot.joints()[k]) for k in before) < 0.1


# --- Speed scaling and stalls (S4 prep). The fake scaling only reaches motion_server: the mock
# controller keeps moving, so these check that motion_server notices, not what the arm does.

def test_scaling_at_zero_too_long_raises_stalled(robot, sensors):
    from tahoma_robot import Stalled
    later(1.0, lambda: sensors.set_scaling(0))
    t = time.monotonic()
    with pytest.raises(Stalled, match="slider"):
        robot.move_straight((0, 0, -0.2), speed=0.03)
    assert 3.5 < time.monotonic() - t < 6.0


def test_short_slowdown_does_not_stall(robot, sensors):
    later(1.0, lambda: sensors.set_scaling(0))
    later(2.5, lambda: sensors.set_scaling(100))
    assert robot.move_straight(BACK, speed=SLOW).reason == "done"


def test_scaling_going_silent_aborts(robot, sensors):
    later(1.0, lambda: sensors.set_flags(scaling_on=False))
    with pytest.raises(SensorStale):
        robot.move_straight(BACK, speed=SLOW)


def test_silent_scaling_refused_before_moving(robot, sensors):
    sensors.set_flags(scaling_on=False)
    time.sleep(0.7)
    r = execute_directly(robot, a_plan(robot))
    assert r.outcome == Out.REJECTED
    assert "speed scaling" in r.message


# --- Straight moves that turn the tool (S6 haptic search) ---

def tool_turn_while(robot, move):
    """Peak turn rate of the tool (rad/s) over 0.1 s windows, from joint feedback stamps."""
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
        o = robot._wait(robot._fk.call_async(q)).pose_stamped[0].pose.orientation
        samples.append((t, (o.x, o.y, o.z, o.w)))
    rates = []
    for i, (ta, a) in enumerate(samples):
        later_ = [(tb, b) for tb, b in samples[i:] if tb - ta >= 0.1]
        if later_:
            tb, b = later_[0]
            dot = min(1.0, abs(sum(x * y for x, y in zip(a, b))))
            rates.append(2 * math.acos(dot) / (tb - ta))
    return max(rates)


def test_straight_move_can_turn_the_tool_slowly(robot):
    angle = math.radians(10)
    turn = (math.sin(angle / 2), 0.0, 0.0, math.cos(angle / 2))   # about the tool's x axis
    start, q0 = robot.tcp_pose()
    peak = tool_turn_while(robot, lambda: robot.move_straight((0, 0, 0), rotation=turn,
                                                              angular_speed=0.1))
    end, q1 = robot.tcp_pose()
    dot = abs(sum(a * b for a, b in zip(q0, q1)))
    assert 2 * math.acos(min(1.0, dot)) == pytest.approx(angle, abs=math.radians(0.5))
    # About the tool's own x axis: the turn from q0 to q1, in the tool frame, is the one asked for.
    # Written out here rather than robot._qmul, so a mistake there can't hide in both.
    (ax, ay, az, aw), (bx, by, bz, bw) = (-q0[0], -q0[1], -q0[2], q0[3]), q1
    rel = (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
           aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)
    assert abs(sum(a * b for a, b in zip(rel, turn))) > math.cos(math.radians(0.5) / 2), \
        f"turned about the wrong axis: {rel}"
    assert moved(start, end) < 0.002, "the tip must stay put while the tool turns"
    assert 0.03 < peak < 0.1 * 1.25, f"turned at {peak:.3f} rad/s"
    robot.move_straight((0, 0, 0), rotation=(-turn[0], 0.0, 0.0, turn[3]))


@pytest.mark.parametrize("angular_speed", [0.0, 1.5, math.inf, math.nan])
def test_bad_angular_speed_refused(robot, angular_speed):
    with pytest.raises(ValueError):
        robot.move_straight((0, 0, 0.01), angular_speed=angular_speed)


@pytest.mark.parametrize("rotation", [(0, 0, 0, 2.0), (math.nan, 0, 0, 1), (0, 0, 1)])
def test_bad_rotation_refused(robot, rotation):
    with pytest.raises(ValueError):
        robot.move_straight((0, 0, 0.01), rotation=rotation)


# --- A stop that reaches the controller just as it finishes a move (D10 review) ---

def plan_back_to(robot, target, seconds=1.0):
    p = a_plan(robot)
    from tahoma_moveit_config.srdf import ARM_JOINTS
    p.points[1].positions = [target[n] for n in ARM_JOINTS]
    p.points[1].time_from_start.sec = int(seconds)
    return p


def test_stop_as_the_move_ends_leaves_the_server_usable(robot, sensors):
    """A cancel that reaches the controller after it has finished the goal, but before it has
    sent the result (up to 50 ms at action_monitor_rate 20), leaves the goal canceling with no
    result: realtime_tools 3.13 skips succeed while canceling. The server used to refuse every
    later move until restart; now a hold goal settles the controller. Force stops swept across a 1 s move's end, each followed by a move."""
    home = robot.joints()
    for delay in [0.995 + 0.005 * i for i in range(8)] * 2:
        sensors.reset()
        time.sleep(0.2)
        box = {}
        mover = threading.Thread(
            target=lambda: box.update(r=execute_directly(robot, a_plan(robot))), daemon=True)
        mover.start()
        later(delay, lambda: sensors.set_force(z=LIMIT + 50))
        mover.join(10)
        time.sleep(0.3)
        sensors.reset()
        time.sleep(0.3)
        assert "r" in box, f"stop at {delay:.3f} s: no result"
        assert box["r"].outcome != Out.FAILED, f"stop at {delay:.3f} s: {box['r'].message}"
        r = execute_directly(robot, plan_back_to(robot, home))
        assert r.outcome == Out.DONE, f"move after a stop at {delay:.3f} s: {r.message}"


def test_move_running_long_past_its_planned_end_raises_stalled(robot, thaw):
    """Only the overtime check can end this move: the controller is frozen mid-move while speed
    scaling stays at 100%, so the server sees time pass but no end."""
    later(0.3, lambda: freeze_controller("-STOP"))
    later(3.6, lambda: freeze_controller("-CONT"))
    t0 = time.monotonic()
    r = execute_directly(robot, a_plan(robot))
    took = time.monotonic() - t0
    assert r.outcome == Out.ABORTED_STALLED, r.message
    assert "after its planned end" in r.message
    assert took > 2.9, f"1 s planned plus 2 s overtime, ended after {took:.2f} s"
    time.sleep(max(0.0, 4.1 - took))
    assert execute_directly(robot, a_plan(robot)).outcome == Out.DONE


def test_scaling_at_one_percent_counts_as_stopped(robot, sensors):
    """The driver can't set the slider below 1%, so 1% must stall like 0."""
    from tahoma_robot import Stalled
    sensors.set_scaling(1)
    time.sleep(0.3)
    plan = a_plan(robot)
    plan.points[1].time_from_start.sec = 6
    try:
        r = execute_directly(robot, plan)
    finally:
        sensors.set_scaling(100)
        time.sleep(0.3)
    assert r.outcome == Out.ABORTED_STALLED and "slider" in r.message, r.message


def test_slow_scaling_is_not_overtime(robot, sensors, thaw):
    """At 10% a 1 s move may take 10 s: overtime counts scaled time, not wall time. The frozen
    controller stands in for one that runs slowly, as the real arm would at 10%."""
    sensors.set_scaling(10)
    time.sleep(0.3)
    later(0.3, lambda: freeze_controller("-STOP"))
    later(4.0, lambda: freeze_controller("-CONT"))
    try:
        r = execute_directly(robot, a_plan(robot))
    finally:
        sensors.set_scaling(100)
        time.sleep(0.3)
    assert r.outcome == Out.DONE, r.message


def test_close_leaves_no_bond_or_spin_threads_running(sensors):
    """T12 item 5: a process using Robot could segfault at exit, after close() had returned.
    bondpy left daemon timers that can fire after the node is gone. After close(), none of
    Robot's or the bond's threads may still run."""
    sensors.reset()
    time.sleep(0.2)
    before = set(threading.enumerate())
    r = Robot.connect("capcup", LIMIT, WORKCELL)
    r.joints()
    r.close()
    time.sleep(0.2)
    left = [t for t in set(threading.enumerate()) - before if t.is_alive()]
    assert left == [], [t.name for t in left]
    time.sleep(0.8)   # the server must see the bond end before the next connect
