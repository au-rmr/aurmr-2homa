#!/usr/bin/env bash
# Opens RViz and a pose/scene picker for checking workcell data. No network, so it cannot reach
# a robot. Usage: scripts/view_scene.sh [pose:=home] [scene:=limits_full]
# VIEW_EXTRA="<command>" runs that command in the same container too (e.g. touch_targets.py markers).
set -eu
cd "$(git rev-parse --show-toplevel)"
SITE=${AURMR_SITE:-uw_lab}
: "${DISPLAY:?needs a desktop session; set DISPLAY}"
docker run --rm --network none -u "$(id -u):$(id -g)" -e HOME=/tmp -e ROBOT_LAN_IP=127.0.0.1 \
  -e DISPLAY -e QT_X11_NO_MITSHM=1 -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e XAUTHORITY=/tmp/.xauth -v "${XAUTHORITY:-$HOME/.Xauthority}":/tmp/.xauth:ro \
  -v /tmp/.X11-unix:/tmp/.X11-unix -v "$PWD":/repo:ro -w /tmp \
  ${VIEW_IMAGE:-ghcr.io/au-rmr/aurmr-2homa/base:dev} bash -c "
    set -e; source /opt/ros/jazzy/setup.bash
    colcon --log-base /tmp/log build --base-paths /repo/robot /repo/end_effectors --build-base /tmp/build --install-base /tmp/install >/tmp/colcon.txt 2>&1 || { cat /tmp/colcon.txt; exit 1; }
    source /tmp/install/setup.bash
    ${VIEW_EXTRA:+( sleep 3; $VIEW_EXTRA ) &}
    ros2 launch tahoma_moveit_config view_scene.launch.py site_dir:=/repo/sites/$SITE $*
  "
