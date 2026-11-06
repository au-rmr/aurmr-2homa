#!/usr/bin/env bash
# L2 tests: MoveIt and ros2_control on mock hardware. No network, no robot.
set -eu
cd "$(git rev-parse --show-toplevel)"
SITE=${AURMR_SITE:-uw_lab}
timeout -s KILL "${MOCK_TIMEOUT:-240}" docker run --rm --network none -e ROBOT_LAN_IP=127.0.0.1 -v "$PWD":/repo:ro -w /tmp \
  ghcr.io/au-rmr/aurmr-2homa/base:dev bash -c "
  source /opt/ros/jazzy/setup.bash
  colcon --log-base /tmp/log build --base-paths /repo/robot /repo/end_effectors --build-base /tmp/build --install-base /tmp/install >/tmp/colcon.txt 2>&1 || { cat /tmp/colcon.txt; exit 1; }
  source /tmp/install/setup.bash
  ros2 launch tahoma_moveit_config mock.launch.py site_dir:=/repo/sites/$SITE >/tmp/mock.txt 2>&1 &
  WORKCELL=/repo/sites/$SITE/workcell.yaml python3 -m pytest -x -q -p no:cacheprovider /repo/tests/mock; rc=\$?
  if [ \$rc != 0 ]; then grep -E 'ERROR|FATAL|error:|Unable|Could not|Failed' /tmp/mock.txt | cut -c1-200 | tail -20; fi
  exit \$rc
"
