"""Streaming (S7) on the UR driver's mock: motion_server's session checks and the streaming
controller's own stops. Runs before test_zz_motion, which kills the stack at its end.

Safety checks tested here (testing posture: safety before the arm): stale commands stop the arm,
bad commands end the session, force ends it, trajectories are refused during a session, a hung
motion_server and a killed client both stop the arm."""
import math
import os
import signal
import subprocess
import sys
import textwrap
import threading
import time

import pytest
import rclpy
from tahoma_moveit_config.srdf import ARM_JOINTS
from tahoma_robot import BadCommand, ExecutionFailed, ForceLimitExceeded, GuardLost, Robot, RobotError

from test_zz_motion import LIMIT, START, WORKCELL, Sensors

WRIST = ARM_JOINTS[-1]           # wrist 3: turning it in place is harmless anywhere
SPEED = 0.05                     # rad/s
MAX = 0.1                        # session limit
RATE = 50.0                      # Hz, above the planner's ~25


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


class Streamer:
    """Sends one velocity at RATE from a thread, like a planner, until stopped or paused."""

    def __init__(self, session, velocities):
        self.session, self.velocities = session, velocities
        self.on = threading.Event()
        self.on.set()
        self.done = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        while not self.done.is_set():
            if self.on.is_set():
                self.session.send(self.velocities)
            time.sleep(1.0 / RATE)

    def stop(self):
        self.done.set()
        self.thread.join()


def wrist_only(v):
    return {j: (v if j == WRIST else 0.0) for j in ARM_JOINTS}


def velocity_when_settled(robot, joint, window=0.3):
    """Mean velocity of one joint over window seconds, from positions."""
    a = robot.joints()[joint]
    time.sleep(window)
    return (robot.joints()[joint] - a) / window


def stopped_within(robot, seconds):
    """True if no joint moves more than 1e-5 rad between readings after `seconds`."""
    time.sleep(seconds)
    a = robot.joints()
    time.sleep(0.1)
    b = robot.joints()
    return max(abs(a[k] - b[k]) for k in a) < 1e-5


# --- Normal use -------------------------------------------------------------------------------
def test_stream_moves_at_the_commanded_velocity_then_trajectories_work(robot):
    start = robot.joints()[WRIST]
    with robot.streaming(max_velocity=MAX) as s:
        st = Streamer(s, wrist_only(SPEED))
        time.sleep(0.5)   # ramp at 0.5 rad/s^2 takes 0.1 s
        v = velocity_when_settled(robot, WRIST, 1.0)
        st.stop()
        assert s.commands > 0
    assert math.isclose(v, SPEED, rel_tol=0.15), f"wrist moved at {v:.4f} rad/s"
    assert robot.joints()[WRIST] - start > 0.05
    assert stopped_within(robot, 0.3)
    robot.go_named(START, speed=0.8)   # trajectory controller is back


def test_max_velocity_is_required(robot):
    for bad in (None, 0, -0.1, 0.3, float("nan")):
        with pytest.raises(ValueError):
            with robot.streaming(max_velocity=bad):
                pass


# --- Stale commands: the controller stops by itself; the session stays open -------------------
def test_stale_commands_stop_the_arm_and_fresh_ones_resume(robot):
    with robot.streaming(max_velocity=MAX) as s:
        st = Streamer(s, wrist_only(SPEED))
        time.sleep(0.5)
        st.on.clear()                      # planner goes quiet
        assert stopped_within(robot, 0.15 + 0.1 + 0.15), "arm must stop 0.15 s stale + 0.1 s ramp"
        assert not s.ended()
        st.on.set()
        time.sleep(0.5)
        assert velocity_when_settled(robot, WRIST, 0.3) > SPEED * 0.8
        st.stop()


# --- Bad commands end the session (no clamping) ------------------------------------------------
@pytest.mark.parametrize("command", [
    wrist_only(0.2),                                        # above the session's 0.1
    wrist_only(-0.2),                                       # same, other direction (T30: mutant m03)
    wrist_only(float("nan")),
    {j: 0.0 for j in ARM_JOINTS[:-1]},                      # a joint missing
    {**wrist_only(0.0), "not_a_joint": 0.0},                # a stranger
    {**{j: 0.0 for j in ARM_JOINTS[1:]}, "not_a_joint": 0.0},  # one joint renamed: right count (T30: m05)
])
def test_bad_command_ends_the_session(robot, command):
    with pytest.raises(BadCommand):
        with robot.streaming(max_velocity=MAX) as s:
            st = Streamer(s, wrist_only(SPEED))
            time.sleep(0.4)
            st.stop()
            s.send(command)
            time.sleep(0.5)
            assert s.ended()
    assert stopped_within(robot, 0.1)
    robot.go_named(START, speed=0.8)


# --- Force, guard, exclusivity ------------------------------------------------------------------
def test_force_limit_ends_the_session(robot, sensors):
    with pytest.raises(ForceLimitExceeded):
        with robot.streaming(max_velocity=MAX, force_limit_n=30) as s:
            st = Streamer(s, wrist_only(SPEED))
            time.sleep(0.4)
            sensors.set_force(z=40)
            try:
                s.wait(timeout=2.0)
            finally:
                st.stop()
    sensors.reset()
    assert stopped_within(robot, 0.1)


def test_trajectories_are_refused_while_streaming(robot):
    with robot.streaming(max_velocity=MAX):
        with pytest.raises(ExecutionFailed):
            robot.go_named(START, speed=0.8)


def test_a_second_session_is_refused(robot):
    with robot.streaming(max_velocity=MAX):
        with pytest.raises(RobotError):
            with robot.streaming(max_velocity=MAX):
                pass


CLIENT = textwrap.dedent("""
    import sys, time
    from tahoma_moveit_config.srdf import ARM_JOINTS
    from tahoma_robot import Robot
    with Robot.connect("capcup", 50.0, sys.argv[1]) as r:
        with r.streaming(max_velocity=0.1) as s:
            print("streaming", flush=True)
            while True:
                s.send({j: (0.05 if j == ARM_JOINTS[-1] else 0.0) for j in ARM_JOINTS})
                time.sleep(0.02)
""")


def last_motion(trace):
    """Time of the last position change in [(t, position)]. One sample that leaves the pose and comes straight back
    is skipped: when arm_controller takes over, the mock hardware reports its stale pre-session position command for
    one cycle (T30: every session end, the whole streamed distance). The UR driver resets that command on the switch
    (ur_robot_driver 3.10.0 hardware_interface.cpp:1510), so it is a mock artefact, not motion."""
    last = 0.0
    for i in range(1, len(trace)):
        t, x = trace[i]
        if abs(x - trace[i - 1][1]) <= 1e-6:
            continue
        blip_out = i + 1 < len(trace) and abs(trace[i + 1][1] - trace[i - 1][1]) <= 1e-6
        blip_back = i >= 2 and abs(x - trace[i - 2][1]) <= 1e-6 and abs(trace[i - 1][1] - x) > 1e-3
        if not (blip_out or blip_back):
            last = t
    return last


def test_killed_client_stops_the_arm_and_guard_can_reconnect(sensors):
    sensors.reset()
    p = subprocess.Popen([sys.executable, "-c", CLIENT, WORKCELL], stdout=subprocess.PIPE, text=True)
    hw = PrivateJoints()
    try:
        assert p.stdout.readline().strip() == "streaming"
        time.sleep(0.5)
        before = hw.wrist()
        time.sleep(0.5)
        assert hw.wrist() - before > 1e-3, "arm must be moving before the kill"
        p.send_signal(signal.SIGKILL)
        p.wait()
        killed = time.monotonic()
        trace = [(0.0, hw.wrist())]
        while time.monotonic() - killed < 1.5:
            time.sleep(0.01)
            trace.append((time.monotonic() - killed, hw.wrist()))
    finally:
        hw.close()
        if p.poll() is None:
            p.kill()
    stopped = last_motion(trace)
    print(f"stopped {stopped:.3f} s after the kill")
    assert stopped < 0.5, "commands stop with the client: 0.15 s stale + 0.1 s ramp"
    time.sleep(1.0)   # the guard frees when the server sees the bond die
    with Robot.connect("capcup", LIMIT, WORKCELL) as r:
        r.go_named(START, speed=0.8)


class PrivateJoints:
    """Joint states straight from the private domain, as the controllers see them: the shared
    copy comes through motion_server and would freeze with it."""

    def __init__(self):
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from sensor_msgs.msg import JointState
        self.ctx = Context()
        rclpy.init(context=self.ctx, domain_id=int(os.environ.get("AURMR_PRIVATE_DOMAIN", "142")))
        self.node = rclpy.create_node("stream_watch_private", context=self.ctx)
        self.joints = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.joints.update(zip(m.name, m.position)), 10)
        self.ex = SingleThreadedExecutor(context=self.ctx)
        self.ex.add_node(self.node)
        threading.Thread(target=self.ex.spin, daemon=True).start()
        deadline = time.monotonic() + 5
        while WRIST not in self.joints:
            assert time.monotonic() < deadline, "no joint states on the private domain"
            time.sleep(0.01)

    def wrist(self):
        return self.joints[WRIST]

    def stopped_within(self, seconds):
        time.sleep(seconds)
        a = self.wrist()
        time.sleep(0.1)
        return abs(self.wrist() - a) < 1e-5

    def close(self):
        self.ex.shutdown()
        self.node.destroy_node()
        rclpy.shutdown(context=self.ctx)


def motion_server_pid():
    out = subprocess.run(["pgrep", "-f", "tahoma_control/motion_server"], capture_output=True,
                         text=True).stdout.split()
    assert len(out) == 1, f"expected one motion_server, found {out}"
    return int(out[0])


def test_hung_motion_server_stops_the_arm(robot):
    """The case the controller exists for: motion_server frozen (not dead) mid-stream."""
    pid = motion_server_pid()
    hw = PrivateJoints()
    try:
        with pytest.raises((GuardLost, RobotError)):
            with robot.streaming(max_velocity=MAX) as s:
                st = Streamer(s, wrist_only(SPEED))
                time.sleep(0.5)
                moving = hw.wrist()
                os.kill(pid, signal.SIGSTOP)
                try:
                    frozen = hw.stopped_within(0.15 + 0.1 + 0.15)
                    drift = hw.wrist() - moving
                finally:
                    os.kill(pid, signal.SIGCONT)
                    st.stop()
                print(f"moved {drift:.4f} rad after the freeze")
                assert frozen, "the controller must stop the arm while motion_server is frozen"
                s.wait(timeout=3.0)   # the bond timed out during the freeze: the session ends
    finally:
        hw.close()
    time.sleep(1.0)
