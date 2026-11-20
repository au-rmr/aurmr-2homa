"""Streaming (S7), test agent T30: the streaming controller's own checks, which test_streaming.py
can't reach because motion_server refuses bad commands first, and the S7 topology where the
planner is a different process from the demo that holds the guard.

Direct commands go to the controller on the private domain, as a hung or buggy motion_server
would send them. Runs after test_streaming and before test_zz_motion (which kills the stack)."""
import os
import subprocess
import sys
import textwrap
import threading
import time

import pytest
import rclpy
from tahoma_moveit_config.srdf import ARM_JOINTS
from tahoma_robot import ForceLimitExceeded, Robot

from test_streaming import MAX, RATE, START, WRIST, Streamer, wrist_only
from test_zz_motion import LIMIT, WORKCELL, Sensors

CONTROLLER = "streaming_velocity_controller"
ACC = 0.5          # rad/s^2, ur_controllers.yaml max_acceleration
CEILING = 0.25     # rad/s, ur_controllers.yaml max_velocity
MARGIN = 0.05      # rad, ur_controllers.yaml limit_margin_rad


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
    time.sleep(0.8)


class Private:
    """The private domain, as the controllers see it: joint states with stamps, a publisher
    straight to the streaming controller, and its parameters."""

    def __init__(self):
        from rcl_interfaces.srv import GetParameters
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray
        self.Float64MultiArray = Float64MultiArray
        self.ctx = Context()
        rclpy.init(context=self.ctx, domain_id=int(os.environ.get("AURMR_PRIVATE_DOMAIN", "142")))
        self.node = rclpy.create_node("t30_private", context=self.ctx)
        self.lock = threading.Lock()
        self.samples = []      # (stamp s, position, velocity) of WRIST
        self.node.create_subscription(JointState, "/joint_states", self._on_joints, 50)
        self.pub = self.node.create_publisher(Float64MultiArray, f"/{CONTROLLER}/commands", 10)
        self.params = self.node.create_client(GetParameters, f"/{CONTROLLER}/get_parameters")
        self.GetParameters = GetParameters
        self.ex = SingleThreadedExecutor(context=self.ctx)
        self.ex.add_node(self.node)
        threading.Thread(target=self.ex.spin, daemon=True).start()
        deadline = time.monotonic() + 5
        while not self.samples:
            assert time.monotonic() < deadline, "no joint states on the private domain"
            time.sleep(0.01)

    def _on_joints(self, m):
        if WRIST not in m.name:
            return
        i = m.name.index(WRIST)
        v = m.velocity[i] if len(m.velocity) > i else float("nan")
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        with self.lock:
            self.samples.append((t, m.position[i], v))
            del self.samples[:-5000]

    def wrist(self):
        with self.lock:
            return self.samples[-1][1]

    def since(self, t0):
        with self.lock:
            return [s for s in self.samples if s[0] >= t0]

    def now(self):
        with self.lock:
            return self.samples[-1][0]

    def send(self, data, seconds, rate=RATE):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.pub.publish(self.Float64MultiArray(data=[float(x) for x in data]))
            time.sleep(1.0 / rate)

    def moved_while(self, fn):
        a = self.wrist()
        fn()
        time.sleep(0.05)
        return abs(self.wrist() - a)

    def parameter(self, name):
        assert self.params.wait_for_service(timeout_sec=5)
        f = self.params.call_async(self.GetParameters.Request(names=[name]))
        deadline = time.monotonic() + 5
        while not f.done():
            assert time.monotonic() < deadline
            time.sleep(0.01)
        return list(f.result().values[0].double_array_value)

    def close(self):
        self.ex.shutdown()
        self.node.destroy_node()
        rclpy.shutdown(context=self.ctx)


@pytest.fixture
def private():
    p = Private()
    yield p
    p.close()


def wrist_vector(v):
    return [v if j == WRIST else 0.0 for j in ARM_JOINTS]


# --- The controller's own command checks (motion_server bypassed) --------------------------------
def test_direct_valid_command_moves_the_arm(robot, private):
    """Control for the tests below: the private path really reaches the arm."""
    with robot.streaming(max_velocity=MAX):
        moved = private.moved_while(lambda: private.send(wrist_vector(0.05), 0.6))
    assert moved > 0.015, f"moved {moved:.4f} rad"


@pytest.mark.parametrize("bad", [
    wrist_vector(CEILING + 0.05),          # above the controller's ceiling
    wrist_vector(-(CEILING + 0.05)),       # same, other direction
    wrist_vector(float("nan")),
    wrist_vector(float("inf")),
    [0.05] * (len(ARM_JOINTS) - 1),         # wrong size
])
def test_direct_bad_command_latches_until_reactivated(robot, private, bad):
    with robot.streaming(max_velocity=MAX):
        moved_bad = private.moved_while(lambda: private.send(bad, 0.3))
        # A good command after a bad one must not move the arm: the latch holds.
        moved_after = private.moved_while(lambda: private.send(wrist_vector(0.05), 0.6))
    assert moved_bad < 1e-4, f"moved {moved_bad:.4f} rad on a bad command"
    assert moved_after < 1e-4, f"moved {moved_after:.4f} rad after a bad command: latch lost"
    # A new session activates the controller again: the latch is cleared.
    with robot.streaming(max_velocity=MAX):
        moved_new = private.moved_while(lambda: private.send(wrist_vector(0.05), 0.6))
    assert moved_new > 0.015, f"new session moved only {moved_new:.4f} rad"


def test_requested_stop_ramps_down_at_once(robot, private):
    """Leaving the block stops the arm by motion_server's zeros, not by waiting for the 0.15 s stale timeout."""
    with robot.streaming(max_velocity=MAX) as s:
        st = Streamer(s, wrist_only(MAX))
        time.sleep(0.6)
        # The planner keeps sending (as a real one would while the demo stops); motion_server must stop relaying.
        t_stop = private.now()           # stamp clock of the private joint states
        s._robot._wait(s._handle.cancel_goal_async())
        time.sleep(0.6)
        st.stop()
        samples = private.since(t_stop)
    moving = [t for t, _, v in samples if abs(v) > 1e-4]
    took = (max(moving) - t_stop) if moving else 0.0
    # Deceleration until the arm is still by motion_server's own test (stream_settle_velocity 0.005 rad/s): after
    # that it switches back, and the mock reports one bogus sample there (see test_streaming.last_motion).
    first_still = next((i for i, (_, _, v) in enumerate(samples) if abs(v) < 0.006), len(samples))
    dec = max_acceleration(samples[:first_still + 1])
    print(f"stopped {took:.3f} s after the cancel (ramp alone 0.2 s), max deceleration {dec:.3f} rad/s^2")
    assert took < 0.3, "the stop path must send zeros: 0.2 s ramp, not 0.15 s stale + ramp"
    assert dec < ACC * 1.3, "the arm must ramp down before trajectories take over, not stop dead"


def test_force_during_the_stop_ramp_still_raises(robot, private, sensors):
    """A requested stop wins the stop reason, but a force over the limit while ramping down must not be lost."""
    with pytest.raises(ForceLimitExceeded):
        with robot.streaming(max_velocity=MAX, force_limit_n=30) as s:
            st = Streamer(s, wrist_only(MAX))
            time.sleep(0.5)
            s._robot._wait(s._handle.cancel_goal_async())    # stop reason: request
            sensors.set_force(z=40)                           # then contact while the arm ramps down
            time.sleep(0.3)
            st.stop()
    sensors.reset()


# --- Acceleration limit ---------------------------------------------------------------------------
def max_acceleration(samples):
    """Largest |dv/dt| between joint-state samples, from the reported velocities."""
    worst = 0.0
    for (t0, _, v0), (t1, _, v1) in zip(samples, samples[1:]):
        if t1 - t0 > 1e-4:
            worst = max(worst, abs(v1 - v0) / (t1 - t0))
    return worst


def test_velocity_steps_are_ramped_up_and_down(robot, private):
    with robot.streaming(max_velocity=MAX) as s:
        t0 = private.now()
        st = Streamer(s, wrist_only(MAX))
        time.sleep(0.6)
        st.on.clear()                    # stale: the controller ramps down by itself
        time.sleep(0.6)
        st.stop()
        samples = private.since(t0)
    peak_v = max(abs(v) for _, _, v in samples)
    acc = max_acceleration(samples)
    print(f"peak velocity {peak_v:.4f} rad/s, max acceleration {acc:.3f} rad/s^2, "
          f"{len(samples)} samples")
    assert peak_v > MAX * 0.9, "control: the arm reached the commanded speed"
    assert acc < ACC * 1.3, f"acceleration {acc:.3f} rad/s^2 above the {ACC} limit"


# --- Position limits ------------------------------------------------------------------------------
@pytest.mark.parametrize("side", ["upper", "lower"])
def test_position_limit_stops_and_latches_and_only_away_is_allowed(robot, private, side):
    i = ARM_JOINTS.index(WRIST)
    sign = 1.0 if side == "upper" else -1.0          # direction towards the limit
    limit = private.parameter(f"position_{side}")[i]
    inside = lambda x: sign * (limit - x)            # distance from the limit, > 0 inside
    robot.go_joints(dict(robot.joints(), **{WRIST: limit - sign * 3 * MARGIN}), speed=0.8)
    try:
        # Towards the limit: stops at about the margin (minus the ramp-down distance, 0.01 rad).
        with robot.streaming(max_velocity=MAX) as s:
            st = Streamer(s, wrist_only(sign * MAX))
            time.sleep(2.5)                         # 0.1 rad at 0.1 rad/s would take 1 s
            st.velocities = wrist_only(-sign * 0.05)  # away: still latched, must not move
            time.sleep(0.3)
            latched_at = private.wrist()
            time.sleep(0.6)
            moved_away_latched = abs(private.wrist() - latched_at)
            st.stop()
        print(f"{side}: stopped {inside(latched_at):.4f} rad inside the limit (margin {MARGIN})")
        assert inside(latched_at) > MARGIN - 0.015, f"went {MARGIN - inside(latched_at):.4f} rad past the margin"
        assert inside(latched_at) < 2 * MARGIN, "control: the arm got near the limit"
        assert moved_away_latched < 1e-4, "latched controller moved"
        # New session, inside the margin: towards the limit is refused at once ...
        with robot.streaming(max_velocity=MAX) as s:
            st = Streamer(s, wrist_only(sign * 0.05))
            a = private.wrist()
            time.sleep(0.6)
            towards = sign * (private.wrist() - a)
            st.stop()
        assert towards < 1e-3, f"moved {towards:.4f} rad towards the limit inside the margin"
        # ... and away is allowed.
        with robot.streaming(max_velocity=MAX) as s:
            st = Streamer(s, wrist_only(-sign * 0.05))
            a = private.wrist()
            time.sleep(0.8)
            away = -sign * (private.wrist() - a)
            st.stop()
        assert away > 0.015, f"moved only {away:.4f} rad away from the limit"
    finally:
        robot.go_named(START, speed=0.8)


# --- S7 topology: the planner is not the demo --------------------------------------------------
PLANNER = textwrap.dedent("""
    import sys, time, rclpy
    from sensor_msgs.msg import JointState
    from tahoma_moveit_config.srdf import ARM_JOINTS
    rclpy.init()
    n = rclpy.create_node("t30_fake_planner")
    pub = n.create_publisher(JointState, "/mpc/joint_velocity_cmd", 1)
    print("planning", flush=True)
    while True:
        pub.publish(JointState(name=list(ARM_JOINTS),
                               velocity=[0.05 if j == ARM_JOINTS[-1] else 0.0 for j in ARM_JOINTS]))
        time.sleep(0.02)
""")

DEMO = textwrap.dedent("""
    import sys, time
    from tahoma_robot import Robot
    with Robot.connect("capcup", 50.0, sys.argv[1]) as r:
        with r.streaming(max_velocity=0.1) as s:
            print("streaming", flush=True)
            s.wait(timeout=600)
""")


def test_killed_demo_stops_the_arm_while_the_planner_keeps_streaming(sensors, private):
    sensors.reset()
    planner = subprocess.Popen([sys.executable, "-c", PLANNER], stdout=subprocess.PIPE, text=True)
    demo = subprocess.Popen([sys.executable, "-c", DEMO, WORKCELL], stdout=subprocess.PIPE,
                            text=True)
    try:
        assert planner.stdout.readline().strip() == "planning"
        assert demo.stdout.readline().strip() == "streaming"
        time.sleep(0.5)
        a = private.wrist()
        time.sleep(0.5)
        assert private.wrist() - a > 1e-3, "control: the planner moves the arm"
        demo.kill()
        demo.wait()
        killed = time.monotonic()
        last_change, previous = killed, private.wrist()
        while time.monotonic() - killed < 2.5:
            time.sleep(0.01)
            now = private.wrist()
            if abs(now - previous) > 1e-6:
                last_change = time.monotonic()
            previous = now
        assert planner.poll() is None, "control: the planner was still streaming"
    finally:
        for p in (demo, planner):
            if p.poll() is None:
                p.kill()
    print(f"stopped {last_change - killed:.3f} s after the demo was killed, planner still sending")
    # The bond's heartbeat timeout (0.5 s) + the ramp decide this, not the 0.15 s stale check.
    assert last_change - killed < 1.0, "arm kept following the planner after the guard died"
    time.sleep(1.0)
    with Robot.connect("capcup", LIMIT, WORKCELL) as r:
        r.go_named(START, speed=0.8)
