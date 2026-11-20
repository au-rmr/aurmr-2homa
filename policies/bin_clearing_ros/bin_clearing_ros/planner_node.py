"""ROS 2 planner node for the bin-clearing demo (S7): tracked poses -> search plan -> MPPI joint velocity commands.

Replaces the old zerorpc chain (bridge_server -> pipeline.py -> isaaclabmpc planner.py server <- mppi_bridge_node).
Runs on the planner host in the planner image, on Isaac Sim's bundled Jazzy rclpy (Python 3.11):

    python -m bin_clearing_ros.planner_node [--planner mcts] [--out /runs/bin_clearing/<name>] [--ros-args ...]

Topics (s7_design.md section 2; standard messages only):
  in   /joint_states            sensor_msgs/JointState   arm state; names mapped to the planner's joint order
  in   /mpc/object_poses        geometry_msgs/PoseArray  planner world frame; order = target, obstacles..., bin back
                                                         tracker, bin side tracker; a missing object = NaN pose
  out  /mpc/joint_velocity_cmd  sensor_msgs/JointState   names + velocity; header.stamp = when computed
  out  /mpc/status              std_msgs/String          JSON: phase, step, steps, message, commands, ...

One episode per run, as the old pipeline: wait for poses + joint state, plan (search in a child process: it starts
its own Isaac app), then start Isaac here and stream MPPI commands until the target has left the bin, a timeout, or
Ctrl-C. Whether the arm moves is decided on the robot side: motion_server passes commands only during a streaming
session that an operator holds (Robot.streaming / `tahoma stream`), and stops the arm on stale or bad commands.
If the search's plan fails its verification (a simulated replay on this scene), the run ends "failed" (no retry; user
+ Paolo 2026-11-20). Commands are clamped to --max-command (default 0.1 rad/s, the old bridge's max_velocity) after
MPPI's output filter, which overshoots MPPI's own u_max 0.08 (up to 0.133); each clamp is counted in the status. This is
not a safety layer: the robot side still ends the session on any command above its limit.
After the clamp, commands are smoothed as the old mppi_bridge did (exponential, alpha 0.2 per 1/20 s, rescaled by the
time since the last command, i.e. a ~0.22 s time constant at any rate); a gap longer than --max-gap restarts it from
zero (the robot side has ramped to zero by then). The planner needs the GPU to itself: before MPPI it reports GPU load
from other jobs (warning only), and if the median gap over the first --rate-check-commands commands exceeds --max-gap
(the robot side's stale limit) the run ends "failed". Status: command rate, gaps over --max-gap, and how often a
command change needs more than --accel-ref rad/s^2 (the streaming controller's cap; reported only).
"""
import argparse
import io
import json
import math
import os
import subprocess
import sys
import threading
import time

# Isaac's planner joint order (UR URDF) and the robot's JointState names (robot/tahoma_moveit_config srdf ARM_JOINTS).
_UR_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3")
DEFAULT_ROBOT_JOINTS = [f"arm_{n}_joint" for n in _UR_JOINTS]


def parse_args(argv):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--planner", default="mcts", choices=["mcts", "alphazero"])
    ap.add_argument("--override", nargs="*", default=[], help="search config overrides (hydra style), e.g. seed=1")
    ap.add_argument("--out", default=None, help="run record dir (default /runs/bin_clearing/<UTC time>)")
    ap.add_argument("--poses-topic", default="/mpc/object_poses")
    ap.add_argument("--joint-states-topic", default="/joint_states")
    ap.add_argument("--command-topic", default="/mpc/joint_velocity_cmd")
    ap.add_argument("--status-topic", default="/mpc/status")
    ap.add_argument("--world-frame", default="mpc_world",
                    help="required header.frame_id of the poses (the planner's world frame)")
    ap.add_argument("--robot-joints", nargs=6, default=DEFAULT_ROBOT_JOINTS)
    ap.add_argument("--state-timeout", type=float, default=0.5,
                    help="joint state / poses older than this (receive time) -> no new command")
    ap.add_argument("--search-timeout", type=float, default=500.0, help="old puzzle_timeout_s")
    ap.add_argument("--mpc-timeout", type=float, default=1000.0, help="old isaaclabmpc_timeout_s")
    ap.add_argument("--max-command", type=float, default=0.1,
                    help="clamp |command| per joint to this (rad/s); old mppi_bridge max_velocity")
    ap.add_argument("--smoothing-alpha", type=float, default=0.2, help="old bridge smoothing_alpha (0 < a <= 1; 1 = off)")
    ap.add_argument("--smoothing-rate", type=float, default=20.0, help="rate (Hz) the alpha is defined at (old control_rate)")
    ap.add_argument("--max-gap", type=float, default=0.15,
                    help="robot side's stale limit (s): rate check threshold, gap counting, smoothing restart")
    ap.add_argument("--rate-check-commands", type=int, default=10,
                    help="median gap over this many first commands must be <= --max-gap, else fail (0 = no check)")
    ap.add_argument("--accel-ref", type=float, default=0.5, help="streaming controller max_acceleration (reporting)")
    ap.add_argument("--gpu-busy-util", type=float, default=20.0,
                    help="GPU load (%%) before MPPI starts above which the node warns that another job is running")
    ap.add_argument("--viewer", action="store_true", help="search with the Isaac viewer (needs DISPLAY)")
    return ap.parse_known_args(argv)


class Inputs:
    """Latest joint state + poses, written by ROS callbacks, read by the planning loop (receive times on this host)."""

    def __init__(self, robot_joints, world_frame):
        self.lock = threading.Lock()
        self.robot_joints = list(robot_joints)
        self.world_frame = world_frame
        self.q = self.dq = None
        self.q_time = 0.0
        self.poses = None          # list of [x y z qw qx qy qz] or None per object (NaN = missing)
        self.poses_time = 0.0
        self.last_valid = None     # per object: last pose that was not NaN
        self.warnings = []

    def on_joint_state(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(j in idx for j in self.robot_joints):
            return
        q = [msg.position[idx[j]] for j in self.robot_joints]
        dq = [msg.velocity[idx[j]] if len(msg.velocity) == len(msg.name) else 0.0 for j in self.robot_joints]
        with self.lock:
            self.q, self.dq, self.q_time = q, dq, time.monotonic()

    def on_poses(self, msg):
        if msg.header.frame_id != self.world_frame:
            self.warnings.append(f"poses in frame '{msg.header.frame_id}', expected '{self.world_frame}': ignored")
            return
        poses = []
        for p in msg.poses:
            v = [p.position.x, p.position.y, p.position.z,
                 p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z]
            poses.append(None if any(math.isnan(x) for x in v) else v)
        with self.lock:
            if self.last_valid is None or len(self.last_valid) != len(poses):
                self.last_valid = [None] * len(poses)
            for i, v in enumerate(poses):
                if v is not None:
                    self.last_valid[i] = v
            self.poses, self.poses_time = poses, time.monotonic()

    def snapshot(self):
        with self.lock:
            return (self.q, self.dq, self.q_time, self.poses, self.poses_time,
                    None if self.last_valid is None else list(self.last_valid))


def _t2b(t):
    import torch
    buf = io.BytesIO()
    torch.save(t, buf)
    return buf.getvalue()


def _b2t(b):
    import torch
    return torch.load(io.BytesIO(b), weights_only=True)


class Smoother:
    """The old mppi_bridge's exponential smoothing, applied once per command instead of at 500 Hz.

    alpha is defined per 1/rate s (bridge: 0.2 at control_rate 20) and rescaled by the time since the last command,
    so the time constant (~0.22 s) does not depend on the planner's rate. After a gap longer than reset_gap (the robot
    side's stale limit: it has ramped to zero) it restarts from zero, as the arm's velocity has.
    """

    def __init__(self, n, alpha, rate, reset_gap):
        if not 0 < alpha <= 1 or rate <= 0:
            raise ValueError("smoothing alpha must be in (0, 1] and rate > 0")
        self.alpha, self.rate, self.reset_gap = alpha, rate, reset_gap
        self.y, self.t = [0.0] * n, None

    def __call__(self, u, t):
        dt = 1.0 / self.rate if self.t is None else t - self.t
        if self.t is not None and dt > self.reset_gap:
            self.y, dt = [0.0] * len(u), 1.0 / self.rate
        a = 1.0 - (1.0 - self.alpha) ** (max(dt, 0.0) * self.rate)
        self.y = [a * x + (1.0 - a) * y for x, y in zip(u, self.y)]
        self.t = t
        return list(self.y)


class PlannerNode:
    def __init__(self, args, ros_args):
        import rclpy
        from geometry_msgs.msg import PoseArray
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String

        self.args = args
        self._JointState, self._String = JointState, String
        rclpy.init(args=["planner_node", *ros_args])
        self.node = rclpy.create_node("bin_clearing_planner")
        self.inputs = Inputs(args.robot_joints, args.world_frame)
        self.node.create_subscription(JointState, args.joint_states_topic, self.inputs.on_joint_state,
                                      qos_profile_sensor_data)
        self.node.create_subscription(PoseArray, args.poses_topic, self.inputs.on_poses, 10)
        self.cmd_pub = self.node.create_publisher(JointState, args.command_topic, 10)
        self.status_pub = self.node.create_publisher(String, args.status_topic, 10)
        self.status = {"phase": "starting", "planner": args.planner, "step": None, "steps": None,
                       "commands": 0, "clamped": 0, "max_raw": 0.0, "rate_hz": None, "median_gap_s": None,
                       "max_gap_s": None, "gaps_over_max": 0, "over_accel": 0, "max_accel": 0.0, "gpu": None,
                       "message": ""}
        self.node.create_timer(1.0, self.publish_status)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self._spin = threading.Thread(target=self.executor.spin, daemon=True)
        self._spin.start()

    # ---- status -----------------------------------------------------------------------------------------------
    def set_status(self, **kw):
        self.status.update(kw)
        self.publish_status()
        if "message" in kw and kw["message"]:
            print(f"[planner_node] {self.status['phase']}: {kw['message']}", flush=True)

    def publish_status(self):
        while self.inputs.warnings:
            self.status["message"] = self.inputs.warnings.pop(0)
            print(f"[planner_node] WARNING {self.status['message']}", flush=True)
        self.status_pub.publish(self._String(data=json.dumps(self.status)))

    def publish_command(self, velocities):
        msg = self._JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = list(self.args.robot_joints)
        msg.velocity = [float(v) for v in velocities]
        self.cmd_pub.publish(msg)

    # ---- episode ----------------------------------------------------------------------------------------------
    def wait_for_inputs(self):
        self.set_status(phase="waiting", message=f"waiting for {self.args.poses_topic} (all objects seen, frame "
                                                 f"{self.args.world_frame}) and {self.args.joint_states_topic}")
        while True:
            q, _, q_t, poses, p_t, _ = self.inputs.snapshot()
            now = time.monotonic()
            if (q is not None and poses is not None and now - q_t < self.args.state_timeout
                    and now - p_t < self.args.state_timeout and len(poses) >= 3 and all(p is not None for p in poses)):
                return q, poses
            time.sleep(0.1)

    def plan(self, poses, out_dir):
        """Bin-frame scene (pipeline conversion, no Isaac needed) -> search in a child process -> solution path."""
        import torch
        from bin_clearing.search.real_robot import scene_from_tracked_poses, search_config, write_scenario_yaml

        cfg = search_config(self.args.planner, self.args.override)
        raw = torch.tensor([v for p in poses for v in p], dtype=torch.float32)
        initial_state, bin_center = scene_from_tracked_poses(raw, cfg)
        scenario = write_scenario_yaml(os.path.join(out_dir, "scenarios", "server_scenario.yaml"),
                                       initial_state, bin_center, cfg.bin_size)
        self.set_status(phase="planning", message=f"{len(poses) - 3} obstacles, scenario {scenario}")
        cmd = [sys.executable, "-m", "bin_clearing.search", "--scenario", str(scenario), "--out", out_dir,
               "--planner", self.args.planner, "--override", *self.args.override,
               *(["viewer=replay"] if self.args.viewer else [])]
        log = open(os.path.join(out_dir, "search.log"), "w")
        try:
            rc = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, timeout=self.args.search_timeout).returncode
        except subprocess.TimeoutExpired:
            raise RuntimeError(f"search timed out after {self.args.search_timeout:.0f} s") from None
        summary = json.load(open(os.path.join(out_dir, "plan.json"))) if os.path.exists(
            os.path.join(out_dir, "plan.json")) else None
        if rc != 0 or summary is None or not summary["solution_path"]:
            r = (summary or {}).get("result", {})
            raise RuntimeError(f"search found no verified plan (rc {rc}, plan_len {r.get('plan_length')}, "
                               f"verify {r.get('verify_rate')}); see {out_dir}/search.log")
        self.set_status(message=f"plan: {[a['action_type'] for a in summary['plan']]} "
                                f"({summary['result']['plan_time_s']:.0f} s)")
        return str(scenario), summary["solution_path"]

    def check_gpu(self):
        """GPU load + memory before our MPPI starts (the search child has exited): other jobs slow the planner."""
        util, mem = [], None
        for _ in range(5):
            try:
                out = subprocess.run(["nvidia-smi", "--query-gpu=utilization.gpu,memory.used,memory.total",
                                      "--format=csv,noheader,nounits"], capture_output=True, text=True, timeout=5).stdout
                u, used, total = (float(x) for x in out.splitlines()[0].split(","))
            except Exception as e:  # noqa: BLE001  (reporting only)
                self.set_status(gpu={"error": str(e)}, message=f"WARNING could not read GPU load: {e}")
                return
            util.append(u)
            mem = (used, total)
            time.sleep(0.2)
        util.sort()
        gpu = {"util_pct": util[len(util) // 2], "mem_used_mib": mem[0], "mem_total_mib": mem[1]}
        busy = gpu["util_pct"] > self.args.gpu_busy_util
        self.set_status(gpu=gpu, message=(f"WARNING GPU busy before MPPI ({gpu['util_pct']:.0f} % load, "
                                          f"{mem[0]:.0f} MiB used): another job will slow the planner"
                                          if busy else f"GPU free ({gpu['util_pct']:.0f} % load)"))

    def run_mpc(self, scenario, solution):
        import torch
        from isaaclab.app import AppLauncher
        self.check_gpu()
        self.set_status(phase="starting_mpc", message="starting Isaac for MPPI")
        AppLauncher(headless=True)
        from bin_clearing.mpc.stacked_bin.planner import build_planner
        planner = build_planner(scenario, solution)

        names = list(planner.sim.robot.joint_names)
        if [n.replace("_joint", "") for n in names] != list(_UR_JOINTS):
            raise RuntimeError(f"planner joint order {names} is not the UR order {_UR_JOINTS}")
        steps = len(getattr(planner.objective, "steps", []))
        self.set_status(phase="executing", step=0, steps=steps, message="streaming MPPI commands")
        root = _t2b(torch.zeros(1, 13))
        t_end = time.monotonic() + self.args.mpc_timeout
        n = 0
        smooth = Smoother(6, self.args.smoothing_alpha, self.args.smoothing_rate, self.args.max_gap)
        gaps, last_sent, last_u = [], None, [0.0] * 6
        while time.monotonic() < t_end:
            q, dq, q_t, _, p_t, last_valid = self.inputs.snapshot()
            now = time.monotonic()
            if now - q_t > self.args.state_timeout or now - p_t > self.args.state_timeout:
                self.set_status(message=f"inputs stale (joint state {now - q_t:.2f} s, poses {now - p_t:.2f} s): "
                                        "no command")
                time.sleep(0.05)
                continue
            state = torch.tensor(list(q) + list(dq) + [v for p in last_valid for v in p], dtype=torch.float32)
            u = _b2t(planner.compute_action_tensor(_t2b(state), root)).view(-1).cpu().tolist()
            lim = self.args.max_command
            raw = max(abs(x) for x in u)
            if raw > lim:
                u = [min(max(x, -lim), lim) for x in u]
                self.status["clamped"] += 1
            t = time.monotonic()
            u = smooth(u, t)
            self.publish_command(u)
            n += 1
            if last_sent is not None:
                gap = t - last_sent
                gaps.append(gap)
                acc = max(abs(a - b) for a, b in zip(u, last_u)) / gap
                st = self.status
                st["gaps_over_max"] += gap > self.args.max_gap
                st["over_accel"] += acc > self.args.accel_ref
                st["max_accel"] = max(st["max_accel"], acc)
                srt = sorted(gaps)
                st.update(median_gap_s=srt[len(srt) // 2], max_gap_s=srt[-1], rate_hz=1.0 / srt[len(srt) // 2])
                k = self.args.rate_check_commands
                if k > 0 and len(gaps) == k:
                    med = sorted(gaps)[k // 2]
                    if med > self.args.max_gap:
                        self.publish_command([0.0] * 6)
                        self.set_status(phase="failed", message=(
                            f"planner too slow: median gap {med:.3f} s over the first {k} commands > "
                            f"{self.args.max_gap} s (the robot side's stale limit); GPU shared? {self.status['gpu']}"))
                        return False
            last_sent, last_u = t, u
            self.status.update(commands=n, max_raw=max(self.status["max_raw"], raw),
                               step=int(getattr(planner.objective, "current_step", -1)))
            if planner.is_done():
                self.publish_command([0.0] * 6)
                self.set_status(phase="done", message=f"target left the bin after {n} commands")
                return True
        self.publish_command([0.0] * 6)
        self.set_status(phase="failed", message=f"MPC timeout ({self.args.mpc_timeout:.0f} s)")
        return False

    def run(self):
        out_dir = self.args.out or os.path.join("/runs", "bin_clearing", time.strftime("%Y-%m-%d_%H%M%S", time.gmtime()))
        os.makedirs(out_dir, exist_ok=True)
        try:
            _, poses = self.wait_for_inputs()
            scenario, solution = self.plan(poses, out_dir)
            ok = self.run_mpc(scenario, solution)
        except KeyboardInterrupt:
            self.publish_command([0.0] * 6)
            self.set_status(phase="stopped", message="Ctrl-C")
            ok = False
        except Exception as e:  # noqa: BLE001  (reported on /mpc/status, then the process exits non-zero)
            self.set_status(phase="failed", message=f"{type(e).__name__}: {e}")
            ok = False
        time.sleep(0.3)   # let the last status go out
        return ok


def main(argv=None):
    args, ros_args = parse_args(sys.argv[1:] if argv is None else argv)
    ok = PlannerNode(args, ros_args).run()
    sys.stdout.flush()
    os._exit(0 if ok else 1)   # Isaac teardown can hang


if __name__ == "__main__":
    main()
