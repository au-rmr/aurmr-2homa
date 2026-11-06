# aurmr-2homa

Robot stack for the AURMR UR16e workcell (ROS 2 Jazzy, Docker). Early construction; licence pending.

## Setup
The host needs Docker only. All ROS and system packages live in the image, so nothing is installed on the host.
- Package list: `docker/base/apt-packages.txt`
- Build: `scripts/build_image.sh`
The base image is pinned by digest, but apt installs the newest packages at build time. For an exact copy of a working setup, use a published image rather than rebuilding.

## Check the workcell by eye
`scripts/view_scene.sh` opens RViz with the collision shapes and a picker for pose and scene. The picker shows MoveIt's verdict for that combination.
