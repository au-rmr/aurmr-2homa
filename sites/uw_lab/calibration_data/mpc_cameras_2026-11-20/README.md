# Camera calibration 2026-11-20 (RealSense D435i camera1 + camera2, MPC/MCTS demo)
- p01_tf.json ... p13_tf.json (no p05): per pose, base_link -> each arm link and the joint angles
  (tests/arm/handeye_sample.py). The same arm sample serves both cameras (tests/arm/handeye_pose.sh, CAMERAS=...).
- cameraN_solve_result.json, cameraN_solve_output.txt: tests/perception/handeye_solve.py --tags 35 43 42
  --tag-size 0.036 --child cameraN_color_optical_frame.
- touch_camera_estimates.txt: tests/perception/tag_touch_check.py on 4 tagStandard41h12 tags (ids 0-3, 20.2 mm)
  lying flat on the table, recorded before the touches. touch_points.csv: tests/arm/touch_point.py (cap cup aluminium
  end, suction cup removed, centred on each tag by eye; columns feature, x, y, z, tool axis, spread mm).
- The camera images (cameraN/p*.npz, 2.7 MB each) are kept on the compute host in
  runs/2026-11-20_mpc_camera_calibration/poses/ (local, deletable).
Result: sites/uw_lab/calibration/camera1.yaml, camera2.yaml. Touch check: within 1.4-8.5 mm, mostly height (cameras
read 1-8 mm high); refine intrinsics if that matters (docs/procedures/camera_calibration.md).
