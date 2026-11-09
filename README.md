# aurmr-2homa

Robot stack for the AURMR UR16e workcell (ROS 2 Jazzy, Docker). Early construction.

## Setup
The host needs Docker only. All ROS and system packages live in the image, so nothing is installed on the host.
- Package list: `docker/base/apt-packages.txt`
- Build: `scripts/build_image.sh`
The base image is pinned by digest, but apt installs the newest packages at build time. For an exact copy of a working setup, use a published image rather than rebuilding.

## Tests
All run in containers with no network access, and none need a robot.
| Script | What it covers | Time |
|---|---|---|
| `scripts/test_model.sh` | Robot model and workcell: kinematics against the old URDF, collision scenes | seconds |
| `scripts/test_mock.sh` | Planning, motion_server and the robot API on the UR driver's mock hardware | about 6 min |
| `scripts/test_network_cut.sh` | Cutting the network to a demo mid-move stops the arm | about 1 min |
| `scripts/test_ursim.sh` | The real-arm bringup against URSim, UR's controller simulator (needs its image once) | about 3 min |
| `scripts/test_tools.sh` | capcup drivers: same results as the old ROS 1 code, ejector requests, nodes | under 1 min |
| `scripts/test_demo.sh` | The cap_cup demo end to end on mock with a pretend item | about 3 min |

Procedures for the real robot: `docs/procedures/`.

## Check the workcell by eye
`scripts/view_scene.sh` opens RViz with the collision shapes and a picker for pose and scene. The picker shows MoveIt's verdict for that combination.

## Licence
MIT, see `LICENSE`. Third-party files keep their own licences: Robotiq meshes in `end_effectors/capcup/meshes` are BSD. Dependencies are not all open for commercial use: FoundationPose is non-commercial, so check any perception model before commercial use.
