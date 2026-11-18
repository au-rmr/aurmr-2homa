# optigrasp demo

Stow items into the pod by hand while perception tracks them, then pick an item by id: a vision grasp (OptiGrasp)
gives the approach, the CapCup's haptic search corrects it (the old ROS 1 demo's flow, user 2026-11-13).

## Status (2026-11-18, tabled by the user; resume from here)
| Part | State |
|---|---|
| Camera (Azure Kinect, 3072P) + calibration | done: compose profile `perception`; `sites/<site>/calibration/camera_lower_right.yaml` |
| Segmentation (`perception/uois`), tracker (`perception/bin_tracker`), grasp model (au-rmr/optigrasp) | done, checked against the old code's recorded outputs |
| Bin rectangles from the pod model (`perception/pod_perception/bin_bounds.py`) | done, checked by eye on a real frame |
| Perception node + services (`perception/pod_perception`, `perception/perception_interfaces`) | done; offline replay + live camera checks (`runs/2026-11-17_pod_perception_offline` on inter) |
| Operator console, stow side (`nodes/cli`) | done, tested against a replayed frame |
| Real stows with items (no arm motion) | NOT DONE: needs ~10 min in the lab, see below |
| Arm pick from a vision grasp | NOT STARTED: design below, needs the user's decisions |

## Run (compute host, no arm motion)
1. Host prerequisites: `docker/host/60-ros2-dds.conf`, someone logged in to the desktop (the Kinect's depth engine needs
   an X display), the weights in `~/aurmr-data/weights` (or `AURMR_WEIGHTS`).
2. Images (local builds): `docker compose -f compose/compute.yaml build kinect perception`.
3. `set -a; . sites/uw_lab/hosts/<host>/display.env; set +a; export XAUTHORITY=$XAUTHORITY_HOST`, then
   `AURMR_HOST=<host> docker compose -f compose/compute.yaml --profile perception up -d kinect perception`.
4. Console: `AURMR_HOST=<host> docker compose -f compose/compute.yaml --profile cli run --rm shell`, then
   `ros2 run optigrasp_demo cli`.
5. In the console: `empty` (the reference, bins as they are now), then per item `stow <bin> <item>` (put it in, stand
   clear, Enter), `list`, `where <item>`, `grasp <item>` (shows the grasp point, no motion), `taken <item>`.
6. Stop: `docker compose -f compose/compute.yaml --profile perception down`.

Network: the camera's ~300 MB/s stays on an isolated loopback domain (77); only the perception services are on the
robot domain (`docker/cyclonedds_perception.xml`; the Kinect container runs with `ROBOT_LAN_IP=127.0.0.1`).

## Next lab session (~10 min, no arm motion)
Stack: steps 1-4 above (robot stack not needed). `empty`; stow 2-3 items one at a time into one bin; `list`, `where`,
`grasp` each; take one out by hand, `taken`; `list`. Checks the tracker and the grasp on real stows (including the
three-item case the old tracker's bug affected). Optional: save the frames (set the node's `state_dir`) to compare
with the old tracker (harness: `~/aurmr-data/reference/bin_tracker/old_vs_new.py`).

## Arm pick: proposed design (not built; the user decides)
Reuse cap_cup's pick (contact, haptic search, lift, retreat, hide), with the approach from vision for any bin:
1. Cup pointing straight into the pod (as cap_cup's approach_quat), at the grasp point's y/z, in front of the bin.
   Joints by IK SEEDED with cap_cup's 2H approach_joints, to stay on the same elbow branch (a pose goal let MoveIt pick
   another branch, whose straight retreat folded the arm: D25, 2026-11-13). Needs a seeded-IK call in tahoma_robot.
2. Move in along the tool axis until contact (force stop as cap_cup), haptic search, lift, retreat, hide; scene and
   held-box size per bin (`pod_bins_soft_<bin>`, `bin_box`).
3. Before the arm: a mock check of which bins (centres and corners) are reachable on that branch with collision-free
   paths, to fix the bins in scope.
Open questions for the user: which bins (start near 2H?); use the model's tilt (roll +-20, pitch +-10 deg; the old
demo tilted the approach by it) or approach straight first and let the search level the cup (dev's proposal).
The old demo also used a hand-set camera offset frame (`rgb_camera_link_offset`, -1 cm in x) to correct its old
calibration; not ported (the 2026-11-16 calibration replaced it). Check the first grasps against that.

## Credits
OptiGrasp: Soofiyan Atar (paper arXiv 2409.19494; empty-mask fix Paolo). Segmentation model: Yi Li with Thomas.
Tracker and the ROS 1 perception: Soofiyan Atar; fixes by Nick Walker, Henri Fung, Markus Grotz. Details in each
package's README.
