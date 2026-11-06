# Deferred on purpose
Things left out for now, with what must happen before they are needed.

| Item | Left out in | Needed by | To do |
|---|---|---|---|
| Camera links + meshes (realsense, azure_kinect, mounts) | S2 | S8, or earlier for any demo that uses a camera | Add them to the robot model with the old calibrated transforms. l515.dae (31.7 MB) and d435.dae (15.1 MB) exceed the 5 MB limit, so simplify them or use boxes. Add the camera links to collision checking, because the arm can hit the mounts |
| `table_up` scene object | S2 | S7 | Measure the raised table, then check the old MPC table bounds against it |
