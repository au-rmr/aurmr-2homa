# Camera calibration (Azure Kinect, eye-to-hand)

RUN 2026-11-16 (manual freedrive). Result: `sites/uw_lab/calibration/camera_lower_right.yaml` (base_link ->
rgb_camera_link); raw data: `sites/uw_lab/calibration_data/camera_lower_right_2026-11-16/`.

Why: the vision grasp is a point in the camera image; the arm needs it in base_link. The same transform projects the
touch-calibrated pod model into the image (bin crops for the bin tracker).

How: AprilTags (tag36h11, 36 mm black square) stay on the arm: 35 on the wrist-1 cap (`arm_forearm_link`), 43 on the
wrist-3 cap (`arm_wrist_2_link`), 42 on the base-joint cap (`arm_shoulder_link`; on its own it only turns about one
axis, so it helps but cannot calibrate alone; with only one other tag it makes the result worse, so use it with at
least two others). Caps belong to the motor housing, i.e. the link before the joint; the
solver finds the link from the data anyway. Flat surfaces only: a tag wrapped on a tube stops decoding off-axis.
Per pose: the arm's link poses from TF + one camera frame. The solver fits one camera pose and each tag's pose on its
link to all detected corners (camera intrinsics and distortion from the driver). Pod tags would not do: the pod moves.

## Session (manual, as run)
Person at the e-stop. Nothing is commanded: the person moves the arm in freedrive, the program is NOT playing.
1. Control host: start the robot stack as in `docs/procedures/s4_first_arm.md` (Start), do not press play.
   Compute host: `AURMR_HOST=<compute host> docker compose -f compose/compute.yaml --profile cli run -d --name calshell shell`.
2. Compute host: the Kinect on an isolated domain (77), 3072P, and the live tag window
   `tests/perception/tag_viewer.py` (needs DISPLAY) to check which tags are visible. See `docker/kinect/Dockerfile` for
   the container flags (GPU, X display, USB).
3. 15-20 poses over the bins, with the wrist turned 15-40 degrees between poses; each pose: at least one tag in view,
   not covered by the hose. Per pose: `tests/arm/handeye_pose.sh runs/<date>_camera_calibration/poses pNN`
   ("MOVED" = redo). It also records the joint angles.
4. Solve (perception image, scipy):
   `python3 tests/perception/handeye_solve.py runs/<date>_camera_calibration/poses --tags 35 43 42 --tag-size 0.036 --out solve.json`.
   Pass: reprojection rms a few px; solves on the even and the odd poses agree within ~5 mm / 0.3 degrees (2026-11-16:
   2.16 px, 4.4 mm / 0.30 deg).
5. Check: `scripts/view_camera.sh` (RViz: live cloud on the pod's collision boxes) — the pod fabric and dividers sit on
   the box edges. Write the yaml, copy the TF jsons and the solver output into `calibration_data/` (images stay in `runs/`).

## Automated (not written yet)
`sites/uw_lab/calibration/camera_cal_poses.yaml` holds the 15 joint poses of 2026-11-16 (checked by a person:
reachable, clear, a tag in view). A replay through motion_server (capped speed, pod scene) with the same sampling and
solve turns a camera swap into one command; its first run on the arm needs the usual go.
