#!/usr/bin/env bash
# Regenerates a tool's collision list. Usage: scripts/gen_collisions.sh capcup
# Output: end_effectors/<tool>/moveit/collisions.srdf. Review the diff before committing:
# every entry switches off a check. "Never" only means sampling found no hit, so for tool
# links only Adjacent pairs are kept: a tool must stay checked against everything else.
set -eu
TOOL=${1:?usage: gen_collisions.sh <tool>}
TRIALS=${TRIALS:-100000}
cd "$(git rev-parse --show-toplevel)"
docker run --rm --network none -u "$(id -u):$(id -g)" -e HOME=/tmp -e ROBOT_LAN_IP=127.0.0.1 -v "$PWD":/repo -w /tmp ghcr.io/au-rmr/aurmr-2homa/base:dev bash -c "
  set -e; source /opt/ros/jazzy/setup.bash
  colcon --log-base /tmp/log build --base-paths /repo/robot /repo/end_effectors --build-base /tmp/build --install-base /tmp/install >/tmp/colcon.txt 2>&1 || { cat /tmp/colcon.txt; exit 1; }
  source /tmp/install/setup.bash
  xacro \$(ros2 pkg prefix --share tahoma_description)/urdf/tahoma.urdf.xacro tool_xacro:=\$(ros2 pkg prefix --share $TOOL)/urdf/tool.xacro > /tmp/m.urdf
  ros2 run moveit_setup_assistant collisions_updater --urdf /tmp/m.urdf --srdf /repo/robot/tahoma_moveit_config/config/base.srdf --output /repo/end_effectors/$TOOL/moveit/collisions.srdf --always --trials $TRIALS
  python3 /repo/scripts/filter_tool_collisions.py /repo/end_effectors/$TOOL/urdf/tool.xacro /repo/end_effectors/$TOOL/moveit/collisions.srdf
"
