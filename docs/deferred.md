# Deferred on purpose
Things left out for now, with what must happen before they are needed.

| Item | Left out in | Needed by | To do |
|---|---|---|---|
| Camera links + meshes (realsense, azure_kinect, mounts) | S2 | S8, or earlier for any demo that uses a camera | Add them to the robot model with the old calibrated transforms. l515.dae (31.7 MB) and d435.dae (15.1 MB) exceed the 5 MB limit, so simplify them or use boxes. Add the camera links to collision checking, because the arm can hit the mounts |
| `table_up` scene object | S2 | S7 | Measure the raised table, then check the old MPC table bounds against it |
| Pod boxes the old code added only when `pod_bin_4h` was in TF (AddFullPodCollisionGeometry) | S2 | Before pick_stow | Read the old condition, then carry them over as a named scene |
| Acceleration limits (off in the old MoveIt config) | S2 | S4 | Set real values, because time parameterization needs them |
| Pod frame tilt | S2 | S8 | Calibration has 2.35 degrees of roll, but the real pod is vertical, and it puts the pod face 5 to 8 cm from the measured x = 0.95 below 2 m height. Recalibrate before bin moves rely on pod_base_link. pod_partial boxes inherit it |
| Limits as UR safety planes | S2 | S4 | Option: put limits_full on the UR controller too, so they hold even if our software is wrong |
| Overlay image with the repo built in | S4 | before S6 | compose robot and shell services build the repo at start into a volume, which takes a minute and depends on the checkout on that host. Build an image per commit instead (workflow C6). |
| URSim runs headless | S4 | S4 | The lab arm runs the tablet program; URSim tests use headless mode. Check the program flow on the real arm (S4 procedure start). |
