# bin_clearing_ros: ROS 2 planner node for the bin-clearing demo

Thin ROS 2 wrapper around `policies/bin_clearing` (Paolo Torrado's MCTS search + MPPI planner). It replaces the
old zerorpc chain (bridge_server, pipeline.py subprocesses, the MPPI zerorpc server and the robot's
mppi_bridge_node client). Runs on the planner host (pinkman) in the planner image, on Isaac Sim's bundled Jazzy
rclpy (Python 3.11), so standard messages only.

```
AURMR_HOST=pinkman docker compose -f compose/planner.yaml run --rm planner [--planner mcts] [--out /runs/bin_clearing/<name>]
```

| Topic | Type | Direction | Notes |
|---|---|---|---|
| /joint_states | sensor_msgs/JointState | in | `arm_*_joint` names, mapped to the planner's UR joint order |
| /mpc/object_poses | geometry_msgs/PoseArray | in | frame `mpc_world` (checked); target, obstacles..., bin back tracker, bin side tracker; missing object = NaN pose |
| /mpc/joint_velocity_cmd | sensor_msgs/JointState | out | names + velocity, stamp = when computed (motion_server's stream session reads it) |
| /mpc/status | std_msgs/String | out | JSON: phase (waiting, planning, starting_mpc, executing, done, failed, stopped), step, steps, commands, message |

One episode per run, as the old pipeline:
1. Wait until every object has a pose and the joint state is fresh.
2. Convert the poses to the bin frame (the pipeline's conversion, `bin_clearing.search.real_robot`), write the
   scenario, run the search in a child process (it starts its own Isaac app), and require a verified plan.
3. Start Isaac in this process, build the MPPI planner and stream its commands as fast as it computes them, until the
   target has left the bin (then one zero command), the MPC timeout, Ctrl-C or an error (reported on /mpc/status).
   Inputs older than 0.5 s: no new command. A missing object keeps its last seen pose.
Run records: `<out>/scenarios`, `<out>/solutions`, `<out>/plan.json`, `<out>/search.log` (repo `runs/`, gitignored).

Safety is on the robot side, not here: commands move the arm only during a streaming session held by an operator,
and motion_server + the streaming controller stop on stale, bad or over-limit commands. This node does not clamp.

## Test without robot or cameras
`tests/fake_cell.py`: kinematic arm following the commands + still cubes from a scenario YAML; `--exit-after N`
moves the target out after N commands so the node should end in "done".
```
AURMR_HOST=pinkman docker compose -f compose/planner.yaml run -d --name s7node planner --out /runs/t1
AURMR_HOST=pinkman docker compose -f compose/planner.yaml run --rm planner_fake_cell \
    --scenario /data/reference/mpc_mcts_real_robot_2026-09/runs/5obs_stacked2_mcts_05/scenarios/server_scenario.yaml --exit-after 20
```
2026-11-19, one run (seed 43): waiting -> planning (4-step plan, 59 s) -> executing -> done after 22 commands;
commands every 0.28 s (median), largest 0.133 rad/s. With the default seed 42 the search's plan failed verification
on this scene (heights snapped to stack levels, which the September runs' code did not do), so the node stopped
before MPC, as the pipeline would.

## Open (to decide with the planner's author)
- Command rate ~3.6 Hz vs the robot side's 0.15 s stale limit; commands above the planner's own 0.08 rad/s limit
  (up to 0.133, from the output filter) vs motion_server's max_velocity check. As is, a stream session would stop.
- Search retry when the plan fails verification (new seed? re-perceive?): now the episode ends.
- The planner world frame `mpc_world` vs the robot's base frame: comes with the perception port (S7 stage 4).
