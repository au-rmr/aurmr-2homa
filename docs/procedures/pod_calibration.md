# Pod calibration by touch

RUN 2026-11-10 (ledger: `~/refresh/phase3/pod_calibration_ledger.md`). Rewritten from the commands that ran.

Why: `pod_base_link` in `sites/uw_lab/workcell.yaml` was hand-set in ROS 1 (2.35 degree false lean, face 5 to 8 cm off), and the bin model came from a different pod. Both were replaced from this session.

How: the arm's kinematics are calibrated, so the tool is a precise probe. Remove the suction cup and press the rigid aluminium end (38.1 mm diameter) flush on a rigid surface: the probe frame `cup_aluminium_end` (196.4 mm from the flange) then lies on it. Our software sends no motion during the touches: the person moves the arm in freedrive.

## The pod frame
`pod_base_link`: origin on the floor at the left upright's centre (as seen from the arm), at the face; x across the face towards column 4, y into the pod, z up. Surfaces used (`tests/arm/fit_pod.py` FEATURES; lab pod measured 2026-11-10: 930 mm outer width, 30 mm L-channel uprights centred on x = 0 and 0.900):

| Feature name | Surface | Pod-frame value | Cup |
|---|---|---|---|
| `face_*` | Front face of an upright | y = 0 | end flush, pointing into the pod |
| `left_side_cyl_*` / `right_side_cyl_*` | Outer side of the left / right upright, touched with the SIDE of the aluminium end | cup axis at x = -0.015 - 0.01905 / 0.915 + 0.01905 | pointing into the pod, cylinder parallel to the surface, contact up to the end |
| `left_side_*`, `right_side_*`, `base_top_*` | Cup end on an outer side / the base top (z 0.255) | | not reachable on this cell |

The arm reaches about 1.1 to 2.5 m on the face; the base top (0.255 m) is out of reach, so the height comes from `--floor` (pod on the floor, base_link z = 0 is the floor).

## Session
Person in the lab with the e-stop in reach; agent on the compute host.

1. Start the stack on the control host as in `docs/procedures/s4_first_arm.md` (Start) and the shell on the compute host. Do NOT press play: freedrive needs the program stopped. Check `/joint_states` is about 500 Hz with the program stopped.
2. Tape the pod: outer width, upright size, bin columns and depth, divider heights above the floor, flap height; put them in FEATURES and `pods.pod_bins` / `pod_col_*` in `workcell.yaml`. Markers for the touch points on the model: `VIEW_EXTRA="python3 /repo/tests/arm/touch_targets.py" DISPLAY=:1 XAUTHORITY=/run/user/1000/gdm/Xauthority scripts/view_scene.sh scene:=pod_bins`.
3. Remove the suction cup. For each point, freedrive flush, hold still, and the agent runs (the shell's /repo is read-only, so points go to /tmp; copy them out after each):
   ```
   docker exec s4shell bash -c "source /ws/install/setup.bash; cd /repo; python3 tests/arm/touch_point.py <feature> /tmp/points.csv"
   docker cp s4shell:/tmp/points.csv <ledger dir>/points_raw.csv
   ```
   Spread over 0.5 mm means it moved: redo. Points: `face_left_low`, `face_left_high`, `face_right_low`, `face_right_high` (low and high as far apart as the arm reaches), `right_side_cyl_low`, `left_side_cyl_low`. Optional height check to a separate file: the end flat on a surface of taped height (e.g. `table_top` into /tmp/heights.csv).
4. Fit: `docker exec s4shell bash -c "cd /repo; python3 tests/arm/fit_pod.py --floor /tmp/points.csv sites/uw_lab/workcell.yaml"`. Pass: every point within 3 mm of its surface, every cup within 3 degrees; the two side residuals agree (otherwise the taped width is wrong); face touches land on their uprights in pod x.
5. Put the FIT line in `frames.pod_base_link`, old value in a comment with the date.
6. Refit the suction cup. Check by eye, with the program playing and a go per motion (speed = tablet slider):
   ```
   python3 tests/arm/corner_check.py start                 # planned, to pre_bin_3g
   python3 tests/arm/corner_check.py corner [--bin 2g]     # planned, 5 cm in front of where 4 bins meet
   python3 tests/arm/corner_check.py in                    # straight to 1 cm, 10 N stop
   python3 tests/arm/corner_check.py out                   # straight back to 5 cm
   python3 tests/arm/corner_check.py back [--pose drop_hide]
   ```
   (each as `docker exec s4shell bash -c "source /ws/install/setup.bash; cd /repo; python3 ..." > <file> 2>&1`). The cup should point at the corner with the stated air gap.
7. After: `scripts/test_model.sh`, then `scripts/test_demo.sh` once; record everything in the ledger.

## Optional cross-check with the Kinect
The Kinect behind the arm (ROS 1 driver only: the Phase 2 compose file for the compute host in `~/refresh/phase2/compose/`, services roscore + kinect, topics under `/camera_lower_right/`) sees the pod face and the arm. An image grabbed 2026-11-09 is in `~/refresh/phase3/podcal/`. Overlaying the corrected model on it checks the result, but the camera's own pose was set by hand, so the arm in the image has to line up first. Not needed for the calibration.
