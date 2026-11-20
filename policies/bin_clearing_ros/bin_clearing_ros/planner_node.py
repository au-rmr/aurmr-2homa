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
This node adds no safety of its own and does not clamp commands (open question, s7_design.md 6b MPPI finding 2).
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
                       "commands": 0, "message": ""}
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

    def run_mpc(self, scenario, solution):
        import torch
        from isaaclab.app import AppLauncher
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
            self.publish_command(u)
            n += 1
            self.status.update(commands=n, step=int(getattr(planner.objective, "current_step", -1)))
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
