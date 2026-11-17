# Camera calibration 2026-11-16 (Azure Kinect, camera_lower_right)
- p00_tf.json ... p14_tf.json: per pose, base_link -> each arm link and the joint angles (tests/arm/handeye_sample.py).
- solve_result.json, solve_output.txt: tests/perception/handeye_solve.py --tags 35 43 42 --tag-size 0.036.
- The camera images (p*.npz, ~70 MB each, 1.1 GB) are too large for git: kept on the compute host in
  runs/2026-11-16_camera_calibration/poses/ (local, deletable). Tag 38 (curved, on the forearm tube) was removed
  during the session and is not used.
Result: sites/uw_lab/calibration/camera_lower_right.yaml.
