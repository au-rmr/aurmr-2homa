# bin_clearing demo (MPC + MCTS, S7)

Clear a target cube out of a cluttered bin by pushing: a search (MCTS, or AlphaZero) plans which cubes to push where,
then MPPI in Isaac Sim streams joint velocities to the arm until the target has left the bin. Planner: Paolo Torrado
("Reactive Control for Cluttered-Scene Non-Prehensile Manipulation", under review). Port of the September 2026 ROS 1
demo; plan and decisions in the migration notes (S7), summarized here.

```
cameras host (winthrop)              compute / GUI (inter)        planner host (pinkman, GPU)        control (emmons)
2x RealSense -> apriltag_ros  --->   tag fusion + cube tracker -> /mpc/object_poses -> planner_node -> /mpc/joint_velocity_cmd
  (domain 77, LAN)                     (NOT PORTED YET)            (MCTS/AZ search + MPPI)            -> motion_server stream
                                                                                                        session -> UR16e
```

## Status (2026-11-20; GPU on pinkman busy with another user's jobs, so no planner runs today)
| Part | State |
|---|---|
| Safety layer: streaming controller, motion_server stream sessions, `Robot.streaming` (`robot/`) | done on the mock: stale / bad / over-limit commands stop and latch, force limit, kill and hang of client, planner and motion_server, position limits, accel ramp 0.5 rad/s^2. Test agent mutants: 24 of 34 killed, survivors explained |
| Collision monitor (`tahoma_moveit_config/collision_monitor.py`) | done on the mock: checks q and q + qd*~0.2 s against the workcell's rigid scene; a session needs a fresh clear status and stops on a predicted collision or a silent monitor |
| Switch back from streaming to trajectories | mock's one-cycle jump fixed (`tahoma_control/UrLikeMockSystem`, as ur_robot_driver 3.10.0 resets its position command); motion_server watches 300 ms for a jump. **URSim run still needed** to confirm the real driver |
| Planner port (`policies/bin_clearing`, verbatim + package changes) | done: same search plan bit-for-bit as Paolo's code on a recorded real-robot scene; first MPPI commands identical (MPPI itself is not deterministic run to run, also in the original) |
| Planner node (`policies/bin_clearing_ros`) | done: search -> MPPI streaming; clamp 0.1 rad/s + the old bridge's smoothing (0.22 s); GPU check and rate check (fails the run if commands come slower than the 0.15 s stale limit); no retry on a failed plan. Fake-cell runs pass; offline image (no downloads at run time) |
| AlphaZero planner | option exists (`--planner alphazero`); checkpoint known (`~/aurmr-data/weights/bin_clearing_alphazero/`, sha256 in SHA256SUMS) but not wired into the node or tested |
| Cameras (`docker/realsense`, `perception/mpc_cubes/launch/cameras.launch.py`) | done: both D435i at 15 Hz, tagStandard41h12 detection with the old demo's settings (ids 0-70, 20.2 mm); viewer `tests/perception/mpc_tag_viewer.py` |
| Camera calibration (`sites/uw_lab/calibration/camera1.yaml`, `camera2.yaml`) | done 2026-11-20: touch check 1.4-8.5 mm, mostly height (cameras read 1-8 mm high); `docs/procedures/camera_calibration.md` |
| Tag fusion + cube tracker -> `/mpc/object_poses` | NOT STARTED: port of Paolo's apriltag_fusion.py + aurmr_tasks object_tracker.py; needs which tag ids sit on which cube faces |
| Planner frame `mpc_world` vs `base_link` | NOT DONE: comes with the tracker |
| `tahoma stream` operator CLI | NOT STARTED |
| On the arm (stage 5) | NOT STARTED: kill tests while streaming, first run at 0.05 rad/s with a person at the e-stop, then the full demo |

## Run what exists
Cameras + tag viewer (no robot needed):
```
# cameras host
AURMR_HOST=winthrop CAMERA_LAN_IP=<its robot-LAN IP> docker compose -f compose/compute.yaml run -d --name rs realsense
# GUI host (DISPLAY + XAUTHORITY from sites/uw_lab/hosts/<host>/display.env)
AURMR_HOST=inter docker compose -f compose/compute.yaml run -d --name mpc_viewer mpc_tag_viewer
```
Images are local builds (`docker compose -f compose/compute.yaml build realsense` on both hosts; the planner image
`compose/planner.yaml` on pinkman only, never pushed: it contains Isaac Sim). Planner node + fake cell (needs
pinkman's GPU to itself): `policies/bin_clearing_ros/README.md`.

## Next (in order)
1. URSim streaming run: the switch back to trajectories holds where the stream left the arm; plus the `tahoma stream`
   CLI. Needed before any streaming on the arm.
2. Tag fusion + cube tracker port -> `/mpc/object_poses` (needs the cube tag layout), with the `mpc_world` frame.
3. AlphaZero wired into the node (checkpoint path in compose), checked on the fake cell once the GPU is free.
4. With the GPU: fake-cell run at full rate (rate check passes; acceleration-cap counts in closed loop).
5. Stage 5 on the arm.

## Open questions (for Paolo, after the port matches)
- MPPI's rollouts assume ideal velocity tracking, but the arm gets smoothed (0.22 s) and acceleration-capped commands.
  The old setup had the same mismatch and worked. Options: filter each rollout like the robot side, or a
  command-change cost.
- If the planner is ever slower than real time: stream the planned horizon instead of one command (not needed while
  the planner has the GPU to itself).
- Calibration height bias (1-8 mm): fit the cameras' focal lengths if it matters for the pushes.

## Credits
Planner (MCTS / AlphaZero search, MPPI planner, stacked-bin task): Paolo Torrado, MIT. MPPI library: mppi_torch
(tud-amr). Old ROS 1 bridge, tag fusion and tracker: lab code (aurmr_tahoma, calib).
