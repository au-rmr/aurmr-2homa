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
and motion_server + the streaming controller stop on stale, bad or over-limit commands. The node clamps commands to
`--max-command` (0.1 rad/s, the old bridge's value) after MPPI's output filter and counts clamps in the status
(`clamped`, `max_raw`), then smooths them as the old bridge did (alpha 0.2 per 1/20 s, rescaled by time: ~0.22 s
time constant; `--smoothing-alpha 1` = off); that is matching the old setup, not safety.
The planner needs pinkman's GPU to itself (the old demo ran at ~25 Hz; with another job on the GPU it runs at
2.5-4 Hz). Before MPPI the node reports GPU load (warning above `--gpu-busy-util` 20 %); if the median gap over
the first 10 commands is above `--max-gap` 0.15 s (= the robot side's stale limit) the run ends "failed"
(`--rate-check-commands 0` turns the check off, e.g. for fake-cell runs on a shared GPU). Status also has
`rate_hz`, `median_gap_s`, `max_gap_s`, `gaps_over_max`, `over_accel` / `max_accel` (command changes above the
streaming controller's 0.5 rad/s^2, reported only).
A plan that fails its verification ends the run "failed": no retry (user + Paolo 2026-11-20).

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
- MPPI's rollouts assume ideal velocity tracking; the robot side smooths (0.22 s) and caps acceleration, so MPPI does
  not score what runs (the old setup had the same mismatch). Options with the planner's author: run the rollout's
  samples through the same filter, or a command-change cost (s7_design.md section 7, Q6).
- If the planner is ever too slow for real time: send the planned horizon instead of one command (Q7).
- The planner world frame `mpc_world` vs the robot's base frame: comes with the perception port (S7 stage 4).
