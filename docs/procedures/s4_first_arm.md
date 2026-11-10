# S4: first run on the real arm

For the person running it and the agent helping. Do the steps in order; each step is its own go-ahead.
Stop at the first surprise and write it down before going on. First run: 2026-11-09, all steps passed
(ledger: `~/refresh/phase3/s4_ledger.md` on inter). The commands below are the ones that ran.

## Rules
- No motion unless a person is at the e-stop, the area is clear, and the tablet speed slider is low (10 to 20%).
- The tablet's own safety limits stay as they are: they sit under our checks, not instead of them.
- Check once before the first motion that the person is at the e-stop, then keep them there for the whole session. The step script doesn't prompt; it prints what each step will do before it moves.
- Agent helping: quote the exact command before each step, and wait for the person's "go" in a new message before every motion. Never ask for go and run the command in the same reply.
- Save the full output of every step to a file; don't trim it.
- Every result goes into the S4 ledger, with the date and the commit.

## Before the session
1. `scripts/test_mock.sh` and `scripts/test_ursim.sh` pass at the commit you are running.
2. Deploy that commit to the control host (until an overlay image exists, docs/deferred.md). On the control host: `git clone https://github.com/au-rmr/aurmr-2homa.git ~/aurmr-2homa` the first time, else `cd ~/aurmr-2homa && git pull`, and check `git log -1` is the commit. Copy the image the tests ran on, from the compute host: `docker save ghcr.io/au-rmr/aurmr-2homa/base:dev | ssh <control host> docker load`, then check `docker images` shows the same image ID on both hosts.
3. On the control host, the UR adapter and the ejector adapter are still in their usual ports, not swapped. A ping to `robot_ip` in `sites/<site>/arm.yaml` answers in about 1.5 ms.
4. On the tablet: the External Control program is loaded, with its host set to the control host's address on the arm's network (see the comment in `arm.yaml`) and port 50002.
5. On the tablet, set the payload by hand (Installation, General, Payload): `mass_kg` and `cog_m` from `end_effectors/<tool>/tool.yaml` (CapCup: 0.205 kg at 0, 0, 105.4 mm). Software can't set it on the lab arm's PolyScope 5.11.6 (docs/deferred.md): `s4_steps.py payload` refuses.

## Start
On the control host (`roles.control` in `sites/<site>/deployments/lab.yaml`):
```
cd ~/aurmr-2homa
AURMR_HOST=<control host> docker compose -f compose/control.yaml --profile robot up
```
Wait for "You can start planning now!", then press play on the External Control program on the tablet. The log should show "Robot connected to reverse interface". The driver also logs the robot software version and "Calibration checked successfully".

On the compute host, keep one shell container open and run each step in it:
```
cd ~/aurmr-2homa
AURMR_HOST=<compute host> docker compose -f compose/compute.yaml --profile cli run -d --name s4shell shell
docker exec s4shell bash -c "source /ws/install/setup.bash; cd /repo; python3 tests/arm/s4_steps.py <step>" > <step>.txt 2>&1
```
`...` in the tables below means that `docker exec` line. Remove the shell at the end with `docker rm -f s4shell`.

The UR driver, the controllers and the dashboard are on the control host's private ROS domain, so nothing on the robot network can command the arm except through motion_server. To use them by hand (`ros2 control`, dashboard and driver services), run on the control host:
```
AURMR_HOST=<control host> scripts/private_shell.sh [command]
```
Re-zero the force sensor (cables resting differently showed as about 11 N at S4): `scripts/private_shell.sh ros2 service call /io_and_status_controller/zero_ftsensor std_srvs/srv/Trigger`.

## L3: arm idle, nothing commanded
| Step | Command | Pass when |
|---|---|---|
| 1 | `... s4_steps.py check` | joint_states, wrench and speed scaling at about 500 Hz, speed scaling equals the slider. Joints match the tablet (degrees, base to wrist 3). No "calibration parameters" error in the launch log. |
| 2 | Leave it running 10 minutes | No controller overrun warnings in the launch log (the startup line "Overruns handling is: enabled" is not one). |
| 3 | `... s4_steps.py force`, the person's hand already on the cup, then press and let go a few times | The force rises with the press and falls back. Note the noise at rest (S4: 0.2 N): a 10 N stop needs to sit well above it. A steady offset of a few N is the payload or the cables, not noise. |

## L4: slow motion, person at the e-stop, scene loaded
| Step | Command | Pass when |
|---|---|---|
| 4 | `... s4_steps.py wrist` | Wrist 3 turns 5 degrees and back. Both RESULT done. |
| 5 | `... s4_steps.py named` | Planned move to pre_bin_3g at 10% (wrists 1 and 2 turn about 180 degrees: watch the tool cable). Watch the whole path for anything near the frame, the table or the pod. S4: 49 s, within 0.001 rad. |
| 6 | `... s4_steps.py straight` | 2 cm out and back along the tool axis. Check it goes out (away from the flange) first. |
| 7 | `... s4_steps.py stop` | Software stop after 1 s, reason stopped. Record the distance and settling time (timed from the result). |
| 8 | `... s4_steps.py force_stop`, push the cup's tip back with something stiff held still (a soft object compresses too slowly) | Reason force, peak near 10 N (S4: 10.8 N). Let go when it stops: 3 s later it moves back 1 cm, with a 10 N stop of its own. |
| 9 | `... s4_steps.py limit`, push hard (about 5 kg) | ForceLimitExceeded. Our limit (50 N) must trip before the tablet's 150 N (no protective stop). |
| 10 | Recorder, then the step, killed by its own PID 5 s in (below) | The arm stops within about 0.6 s (S4: cancel 0.50 s, stopped about 0.6 s; URSim 0.57 s). No return move. |
| 11 | Recorder on the control host, the step, then pull the compute host's robot-network cable when the tool starts moving (below) | The arm stops within about 0.6 s (S4: cancel 0.44 s after the link went down). The step raises GuardLost. Plug back in; `check` works again from the same shell. |
| 12 | Stop the robot container on the control host while idle: `AURMR_HOST=<control host> docker compose -f compose/control.yaml --profile robot stop robot` | The tablet program just stops, no message. Must stop within the 30 s grace period, not by SIGKILL (exit 137 at S4, before stop_signal SIGINT). Recover: start the stack first, wait for it, then press play (play first gives "connection refused" on port 50002). |
| 13 | `... s4_steps.py straight`, then press Pause on the tablet as soon as the tool moves | The move raises Stalled about 3 s after the pause. Press play, then `wrist`: both done. The slider can't do this: it stops at 2%, which correctly counts as moving. |
| 14 | Protective stop: only if one happens by itself | The move raises; recovery goes through the tablet. No automatic recovery. |

Step 10, on the compute host. Kill the step's own process: in `a && b & P=$!` the `&` backgrounds the whole chain and `$!` is a wrapper shell, and `pgrep -f` matches the calling shell, so both miss.
```
docker exec -d s4shell bash -c "source /ws/install/setup.bash; python3 /repo/tests/arm/record_joints.py 14 > /tmp/joints.csv"
docker exec s4shell bash -c "source /ws/install/setup.bash; cd /repo; python3 tests/arm/s4_steps.py straight & P=\$!; sleep 5; ps -o pid=,comm= -p \$P; kill -9 \$P; echo KILLED \$(date +%s.%N); sleep 3"
```
`ps` must print `python3` before the kill. The stop time is the first sample in `joints.csv` after the KILLED time with near-zero speed; the controller log shows "Got request to cancel goal".

Step 11. SSH to the control host runs over the cable you pull, so start the recorder there first, detached, on the private domain:
```
ssh <control host> 'cd ~/aurmr-2homa && AURMR_HOST=<control host> docker compose -f compose/control.yaml exec -d -e ROS_DOMAIN_ID=142 -e CYCLONEDDS_URI=file:///repo/robot/tahoma_control/config/cyclonedds_private.xml robot bash -c "source /ws/install/setup.bash && python3 /repo/tests/arm/record_joints.py 40 > /tmp/joints11.csv"'
docker exec s4shell bash -c "source /ws/install/setup.bash; cd /repo; python3 tests/arm/s4_steps.py straight" > step11.txt 2>&1
journalctl -k --since -3min -o short-unix | grep <robot-network interface>   # link down time
```
Both hosts must be NTP-synced (`timedatectl`); compare the link-down time with the cancel time in the control host's log.

If a straight move fails with `PartialPath: straight path refused: joint N moves more than 0.3 rad per 5 mm` or `the tip leaves the line`, that is our own joint jump check (`robot/tahoma_robot/tahoma_robot/path_check.py`), not MoveIt: nothing moved. Pick a start pose further from the singularity rather than raising the limit.

## After
- Shut down: `docker rm -f s4shell` on the compute host, `AURMR_HOST=<control host> docker compose -f compose/control.yaml --profile robot down` on the control host.
- Measure the force change while the wrist rotates with no contact (S4 plan, problem 4). It must stay well under the stop forces.
- Write the ledger.

## What was rehearsed without the arm
`scripts/test_ursim.sh` runs this bringup with the real UR driver against URSim (headless mode), including steps 1, 4 to 8 and 13 (slider at 1%), the slider at 10% (a 5 cm move takes about a minute and must still finish), a killed demo and a stopped program. `URSIM_VERSION` picks the PolyScope version.
Not covered by URSim: real force data (it reads 0), the tablet program flow (URSim runs headless), real stopping distances, real network timing, encoder jitter, and protective stops.
