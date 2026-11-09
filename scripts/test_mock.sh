#!/usr/bin/env bash
# L2 tests: MoveIt and ros2_control on mock hardware. No network, no robot.
# On failure the full output stays in a directory named on stderr: pytest.txt, mock.txt, ros_log/.
# MOCK_OUT: parent of that directory. MOCK_DOCKER_ARGS: extra docker run args, e.g. --cpuset-cpus=0-3.
set -eu
cd "$(git rev-parse --show-toplevel)"
SITE=${AURMR_SITE:-uw_lab}
OUT=$(mktemp -d "${MOCK_OUT:-${TMPDIR:-/tmp}}/mock.XXXXXX")
NAME=aurmr-mock-$(basename "$OUT" | tr -d .)
rc=0
# timeout only kills the docker client, so the container is removed by name afterwards.
# shellcheck disable=SC2086  # MOCK_DOCKER_ARGS is a list of args
timeout -s KILL "${MOCK_TIMEOUT:-720}" docker run --rm --name "$NAME" --network none \
  --user "$(id -u):$(id -g)" -e HOME=/tmp -e ROS_LOG_DIR=/out/ros_log -e ROBOT_LAN_IP=127.0.0.1 ${MOCK_DOCKER_ARGS:-} \
  -v "$PWD":/repo:ro -v "$OUT":/out -w /tmp ghcr.io/au-rmr/aurmr-2homa/base:dev bash -c "
  source /opt/ros/jazzy/setup.bash
  colcon --log-base /tmp/log build --base-paths /repo/robot /repo/end_effectors --build-base /tmp/build --install-base /tmp/install >/out/colcon.txt 2>&1 || { cat /out/colcon.txt; exit 1; }
  source /tmp/install/setup.bash
  export AURMR_PRIVATE_DOMAIN=142 AURMR_PRIVATE_URI=file:///repo/robot/tahoma_control/config/cyclonedds_private.xml
  export CYCLONEDDS_URI=file:///repo/robot/tahoma_control/config/cyclonedds_shared.xml,\$AURMR_PRIVATE_URI
  ros2 launch tahoma_moveit_config mock.launch.py site_dir:=/repo/sites/$SITE speed_scaling_topic:=/fake/speed_scaling >/out/mock.txt 2>&1 &
  WORKCELL=/repo/sites/$SITE/workcell.yaml python3 -m pytest -v -p no:cacheprovider /repo/tests/mock 2>&1 | tee /out/pytest.txt; rc=\${PIPESTATUS[0]}
  if [ \$rc != 0 ]; then grep -E 'ERROR|FATAL|error:|Unable|Could not|Failed' /out/mock.txt | cut -c1-200 | tail -20; fi
  exit \$rc
" || rc=$?
docker rm -f "$NAME" >/dev/null 2>&1 || true
[ "$rc" = 137 ] && echo "test_mock timed out after ${MOCK_TIMEOUT:-720} s" >&2
if [ "$rc" = 0 ]; then rm -rf "$OUT"; else echo "test_mock failed (rc=$rc), output kept in $OUT" >&2; fi
exit "$rc"
