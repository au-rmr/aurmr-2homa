# S4: first run on the real arm

For the person running it and the agent helping. Do the steps in order; each step is its own go-ahead.
Stop at the first surprise and write it down before going on.

## Rules
- No motion unless a person is at the e-stop, the area is clear, and the tablet speed slider is low (10 to 20%).
- The tablet's own safety limits stay as they are: they sit under our checks, not instead of them.
- Check once before the first motion that the person is at the e-stop, then keep them there for the whole session. The step script doesn't prompt; it prints what each step will do before it moves.
- Every result goes into the S4 ledger, with the date and the commit.

## Before the session
1. The S3 review is closed, and `scripts/test_mock.sh` and `scripts/test_ursim.sh` pass at the commit you are running.
2. On the control host, the UR adapter and the ejector adapter are still in their usual ports, not swapped. A ping to `robot_ip` in `sites/<site>/arm.yaml` answers in about 1.5 ms.
3. On the tablet: the External Control program is loaded, with its host set to the control host's address on the arm's network (see the comment in `arm.yaml`). Read the robot software version and write it down.
4. The tool weight is set as the payload on the tablet, or noted as unknown.

## Start
On the control host (`roles.control` in `sites/<site>/deployments/lab.yaml`):
```
cd ~/aurmr-2homa
AURMR_HOST=<control host> docker compose -f compose/control.yaml --profile robot up
```
Then press play on the External Control program on the tablet. The log should show "Robot connected to reverse interface".
On the compute host, open a shell with the robot API:
```
AURMR_HOST=<compute host> docker compose -f compose/compute.yaml run --rm shell
```
The step commands below run in that shell, except `payload`.

The UR driver, the controllers and the dashboard are on the control host's private ROS domain, so nothing on the robot network can command the arm except through motion_server. To use them by hand (the payload step, `ros2 control`, dashboard services), open a shell on the control host:
```
AURMR_HOST=<control host> scripts/private_shell.sh
```

## L3: arm idle, nothing commanded
| Step | Command | Pass when |
|---|---|---|
| 1 | `python3 tests/arm/s4_steps.py check` | joint_states and wrench at about 500 Hz, speed scaling equals the slider. Joints match the tablet. The launch log has no "calibration parameters" error. |
| 2 | Leave it running 10 minutes | No controller overrun warnings in the launch log. |
| 3 | `python3 tests/arm/s4_steps.py force` and press the tool by hand | The force rises with the press and falls back. Note the noise at rest: a 10 N stop needs to sit well above it. |

## L4: slow motion, person at the e-stop, scene loaded
| Step | Command | Pass when |
|---|---|---|
| 4 | `... s4_steps.py wrist` | Wrist 3 turns 5 degrees and back. Both RESULT done. |
| 5 | `... s4_steps.py named` | Planned move to pre_bin_3g at 10%. Watch the whole path for anything near the frame, the table or the pod. |
| 6 | `... s4_steps.py straight` | 2 cm out and back along the tool axis. Check the direction is right. |
| 7 | `... s4_steps.py stop` | Software stop after 1 s. Record the kept-moving distance and time. |
| 8 | `... s4_steps.py force_stop`, press the cup with a soft object | Reason force, peak near 10 N. Record the distance and time. |
| 9 | `... s4_steps.py limit`, press harder | ForceLimitExceeded. Our limit (50 N) must trip before the tablet's 150 N. |
| 10 | Start `... s4_steps.py straight`, then kill the shell with Ctrl-C twice, or close the terminal | The arm stops within about 0.6 s (URSim: 0.57 s). |
| 11 | Pull the compute host's network cable during a slow move | The arm stops within about 0.5 s. Plug it back and reconnect. |
| 12 | Stop the robot container on the control host during an idle moment | The driver goes down; note what the tablet shows. Never during motion on the first try. |
| 13 | During a slow move, drag the tablet slider to 0 | The move raises Stalled after 3 s. Slider back up; the next move works. |
| 14 | Protective stop: only if one happens by itself | The move raises; recovery goes through the tablet. No automatic recovery. |

If a straight move fails with `PartialPath: straight path refused: joint N moves more than 0.3 rad per 5 mm` or `the tip leaves the line`, that is our own joint jump check (`robot/tahoma_robot/tahoma_robot/path_check.py`), not MoveIt: nothing moved. Pick a start pose further from the singularity rather than raising the limit.

## After
- Tool payload is in `end_effectors/capcup/tool.yaml` (0.205 kg, CoG estimated at the middle of the cup). Check it matches the tablet; refine with the tablet's payload estimation if force readings drift with wrist rotation.
- Measure the force change while the wrist rotates with no contact (S4 plan, problem 4). It must stay well under the stop forces.
- Write the ledger, then decide S5.

## What was rehearsed without the arm
`scripts/test_ursim.sh` runs this bringup with the real UR driver against URSim (headless mode), including steps 1, 4 to 8 and 13, the slider at 10% (a 5 cm move takes about a minute and must still finish), a killed demo and a stopped program.
Not covered by URSim: real force data (it reads 0), the tablet program flow (URSim runs headless), real stopping distances, real network timing, and protective stops.
