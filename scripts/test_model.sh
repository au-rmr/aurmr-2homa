#!/usr/bin/env bash
# L1 model tests in the image. Builds into a throwaway directory; nothing is left in the repo.
set -eu
cd "$(git rev-parse --show-toplevel)"
SITE=${AURMR_SITE:-uw_lab}
docker run --rm --network none -v "$PWD":/repo:ro -w /tmp ghcr.io/au-rmr/aurmr-2homa/base:dev bash -c "
  set -e; source /opt/ros/jazzy/setup.bash
  colcon --log-base /tmp/log build --base-paths /repo/robot /repo/end_effectors --build-base /tmp/build --install-base /tmp/install --event-handlers console_direct- >/tmp/colcon.txt 2>&1 || { cat /tmp/colcon.txt; exit 1; }
  source /tmp/install/setup.bash
  xacro \$(ros2 pkg prefix --share tahoma_description)/urdf/tahoma.urdf.xacro kinematics_params:=/repo/sites/$SITE/calibration/arm_kinematics.yaml > /tmp/new.urdf
  check_urdf /tmp/new.urdf 2>/dev/null | tail -1 || true
  NEW_URDF=/tmp/new.urdf REF_URDF=/repo/sites/$SITE/reference/tahoma_capcup_ros1.urdf python3 -m pytest -q -p no:cacheprovider /repo/tests/model
"
