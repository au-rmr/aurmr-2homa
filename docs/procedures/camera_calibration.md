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

## RealSense pair (MPC/MCTS demo), RUN 2026-11-20
Result: `sites/uw_lab/calibration/camera1.yaml`, `camera2.yaml` (base_link -> `cameraN_color_optical_frame`); raw
data `sites/uw_lab/calibration_data/mpc_cameras_2026-11-20/`. Same method and arm tags as above; differences:
1. Cameras on the cameras host (winthrop), camera domain 77 over the robot LAN, arm tags detected by apriltag_ros:
   `AURMR_HOST=winthrop CAMERA_LAN_IP=<its LAN IP> docker compose -f compose/compute.yaml run -d --name rs realsense
   bash -c "source /opt/ros/jazzy/setup.bash && exec ros2 launch /repo/perception/mpc_cubes/launch/cameras.launch.py
   site_dir:=/repo/sites/uw_lab tags:=view family:=36h11 size:=0.036"`; live view on the GUI host:
   `AURMR_HOST=inter docker compose -f compose/compute.yaml run -d --name mpc_viewer mpc_tag_viewer` (both cameras
   side by side; bottom bar = tags both see).
2. Per pose (both cameras, one arm sample): `CAMERAS="camera1 camera2" CAM_LAN_IP=<compute host LAN IP>
   tests/arm/handeye_pose.sh runs/<date>_mpc_camera_calibration/poses pNN`.
3. Solve each camera directory: `handeye_solve.py runs/.../poses/cameraN --tags 35 43 42 --tag-size 0.036
   --child cameraN_color_optical_frame --out cameraN.json`. Tag 42 is out of these cameras' view.
4. Touch check (the acceptance test; the even/odd split needs ~8 views per tag per camera, which 12 poses did not
   give): lay tagStandard41h12 tags flat in view (cameras back to the site settings), `tests/perception/tag_touch_check.py
   <id> cameraN.json...` per tag on the camera domain, then touch each centre with the cap cup's aluminium end
   (suction cup off, freedrive) and `tests/arm/touch_point.py tagN` in the robot-domain shell.
2026-11-20: 12 poses, rms 2.23 / 1.91 px (floor ~2 px: adding poses did not lower it; not tag size, flatness, edges or
one pose). Touch: camera1 4.3-8.3 mm, camera2 1.4-8.5 mm, mostly height (both read 1-8 mm high). Likely ~1 % focal
error in the factory intrinsics (cameras report zero distortion). If it matters: fit fx, fy, cx, cy with the camera
pose in handeye_solve.py (not written) and re-solve on the same poses + touches.
