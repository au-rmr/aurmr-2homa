#!/usr/bin/env bash
# L2 demo test (S6): the cap_cup demo on mock hardware with the real capcup drivers, a fake
# ejector and a pretend item. No network, no devices.
set -eu
cd "$(git rev-parse --show-toplevel)"
SITE=${AURMR_SITE:-uw_lab}
OUT=$(mktemp -d "${DEMO_OUT:-${TMPDIR:-/tmp}}/demo.XXXXXX")
NAME=aurmr-demo-$(basename "$OUT" | tr -d .)
rc=0
timeout -s KILL "${DEMO_TIMEOUT:-600}" docker run --rm --name "$NAME" --network none --user "$(id -u):$(id -g)" \
  -e HOME=/tmp -e ROS_LOG_DIR=/out/ros_log -e ROBOT_LAN_IP=127.0.0.1 -e AURMR_SITE_DIR=/repo/sites/$SITE \
  -e PYTEST_ADDOPTS="${PYTEST_ADDOPTS:-}" -v "$PWD":/repo:ro -v "$OUT":/out -w /tmp \
  ghcr.io/au-rmr/aurmr-2homa/base:dev bash -c "
  source /opt/ros/jazzy/setup.bash
  colcon --log-base /tmp/log build --base-paths /repo/robot /repo/end_effectors /repo/skills /repo/demos --build-base /tmp/build --install-base /tmp/install >/out/colcon.txt 2>&1 || { tail -30 /out/colcon.txt; exit 1; }
  source /tmp/install/setup.bash
  export AURMR_PRIVATE_DOMAIN=142 AURMR_PRIVATE_URI=file:///repo/robot/tahoma_control/config/cyclonedds_private.xml
  export CYCLONEDDS_URI=file:///repo/robot/tahoma_control/config/cyclonedds_shared.xml,\$AURMR_PRIVATE_URI
  ros2 launch tahoma_moveit_config mock.launch.py site_dir:=/repo/sites/$SITE start_pose:=drop_hide >/out/mock.txt 2>&1 &
  ros2 launch capcup_driver capcup.launch.py site_dir:=/repo/sites/$SITE fake_vacuum:=true fake_item_topic:=/fake/item_under_cup >/out/capcup.txt 2>&1 &
  python3 /repo/tests/demo/fake_world.py >/out/world.txt 2>&1 &
  # Mock has no force sensor: a steady zero wrench, on the private domain where the UR driver's
  # would be. Contact and seal come from the real drivers.
  ROS_DOMAIN_ID=\$AURMR_PRIVATE_DOMAIN CYCLONEDDS_URI=\$AURMR_PRIVATE_URI ros2 topic pub -r 200 /wrench geometry_msgs/msg/WrenchStamped '{}' >/dev/null 2>&1 &
  python3 -c 'import time; time.sleep(25)'   # stack up: move_group starts after the controller
  python3 -m pytest -v -s -p no:cacheprovider /repo/tests/demo 2>&1 | tee /out/pytest.txt; exit \${PIPESTATUS[0]}
" || rc=$?
docker rm -f "$NAME" >/dev/null 2>&1 || true
if [ "$rc" = 0 ]; then rm -rf "$OUT"; else echo "test_demo failed (rc=$rc), output kept in $OUT" >&2; fi
exit "$rc"
