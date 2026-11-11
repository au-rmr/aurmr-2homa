# S6: the cap_cup demo on the real robot

After S4 has passed. Each level is its own go-ahead; stop at the first surprise and write it down.
Same rules as S4: a person at the e-stop for any motion, slider low, results into the S6 ledger with date and commit.

## What has been checked without the robot
- `scripts/test_tools.sh`: the capcup math gives the same contact, alignment and pose proposals as the old ROS 1 code on 600 recorded-style samples; the ejector gets the old driver's exact bytes; the search's turn levels a tilted cup (8 to 0.1 degrees, both axes, both signs).
- `scripts/test_demo.sh`: the whole demo on mock hardware with the real drivers, a fake ejector and a pretend item tilted by 10 degrees each way, plus a run with nothing to touch.

Not checked, and what this procedure is for: that the electrodes are where the code thinks (channel order and the tool frame), real cap counts per mm of press, the real seal, and the pod model at bin 2H.

## Before
1. `tool.yaml` has the weight and CoG (0.205 kg, 106.7 mm, 2026-11-10), and the same values are set on the tablet (Installation, General, Payload): software payload setting stops the program on PolyScope 5.11 (docs/deferred.md).
2. Pod calibration: DONE 2026-11-10 (docs/procedures/pod_calibration.md; ledger ~/refresh/phase3/pod_calibration_ledger.md); corner checks at 2G/3G/2H/3H and 3H/4H/3I/4I passed by eye. Inside the bin the demo plans against `pod_bins_soft_2h` (`scene_bin` in `sites/<site>/demos/cap_cup.yaml`): bin 2H's canvas walls 2 cm outside the real fabric and no lip, because the canvas is flexible and the search needs to push it (user); the uprights and top strut are rigid and true size. Watch: canvas resistance against the 10 N / 15 N force stops (a stop on the canvas, not the item, means the allowance or the approach height needs changing); the 2H approach puts the cup about 10 mm over the lip; rows are tape-measured (+-1 cm). Also confirm at a stop: s4_steps' watch_stop settling band (S4 follow-up 2).
3. On the compute host: `docker compose -f compose/compute.yaml --profile tool up` (cap board bridge, contact detector, haptic search). The cup must be untouched when it starts: the first sample is the zero point.
4. On the control host: `docker compose -f compose/control.yaml --profile tool up` (ejector), next to the robot service.

## L3: tool only, arm idle
| Step | How | Pass when |
|---|---|---|
| 1 | `ros2 topic hz /cap_data` and `/cap_contact` | About 50 Hz each. |
| 2 | Press one electrode at a time with a finger, going round the cup | The channel that rises matches the drawing: channel 1 at +22.5 degrees from the tool's x axis, counting towards y. Write down the real order. If it differs, the search turns the wrong way: fix the order in the driver before any motion. |
| 3 | Press the cup flat on a box by hand | /cap_contact goes true, and /haptic_search/alignment goes true once pressed evenly. |
| 4 | `ros2 service call /vacuum/set capcup_interfaces/srv/SetVacuum "{mode: 1}"`, cover the cup by hand, then mode 2, then 0 | /vacuum/system_vacuum_mbar rises above 750 when covered and /vacuum_seal goes true; blow-off releases. |

## L4: the demo in pieces, item in bin 2H, low speed
| Step | How | Pass when |
|---|---|---|
| 5 | Run the demo with the item removed | Outcome no_contact; the arm hides again. |
| 6 | Item placed flat | Outcome picked; note the number of search moves. |
| 7 | Item placed tilted (about 10 degrees) | Picked. The search turns towards level, never away. |
| 8 | Item that can't seal (porous) | Search times out (20 s), outcome missed, the arm leaves the bin. |
| 9 | Kill the demo during the search | The arm stops (guard); vacuum stays as it was. Recover by hand. |

Run the demo from the shell: `ros2 run cap_cup_demo run --site-dir /repo/sites/<site>`. The vacuum calls in step 4 go to the tool's own service on the shared domain; driver services (payload, dashboard) are on the control host only: `scripts/private_shell.sh`.

## L5: sign-off
Ten picks in a row at normal speed, with the outcomes recorded. Then the first promotion to the demo branch.

## Known differences from the old demo
See `demos/cap_cup/cap_cup_demo/task.py`: fresh proposals only, no searching without contact, the outcome says whether the item is held, the retreat follows the approach direction and backs out only until 5 cm clear of the pod face, then plans the rest, and inside the bin only the pod-face wall is dropped while the bins are modelled. The search alternates on purpose: a turn, then a move (the old demo took whichever proposal came next).
