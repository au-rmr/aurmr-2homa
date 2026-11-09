#!/usr/bin/env bash
# Tool drivers (S5): the capcup math against the old ROS 1 output, and the nodes with UDP and a
# fake ejector. No network, no devices.
set -eu
cd "$(git rev-parse --show-toplevel)"
SITE=${AURMR_SITE:-uw_lab}
OUT=$(mktemp -d "${TOOLS_OUT:-${TMPDIR:-/tmp}}/tools.XXXXXX")
rc=0
timeout -s KILL "${TOOLS_TIMEOUT:-300}" docker run --rm --network none --user "$(id -u):$(id -g)" \
  -e HOME=/tmp -e ROS_LOG_DIR=/out/ros_log -e ROBOT_LAN_IP=127.0.0.1 -e PYTEST_ADDOPTS="${PYTEST_ADDOPTS:-}" \
  -v "$PWD":/repo:ro -v "$OUT":/out -w /tmp \
  ghcr.io/au-rmr/aurmr-2homa/base:dev bash -c "
  source /opt/ros/jazzy/setup.bash
  colcon --log-base /tmp/log build --base-paths /repo/end_effectors /repo/skills --build-base /tmp/build --install-base /tmp/install >/out/colcon.txt 2>&1 || { tail -30 /out/colcon.txt; exit 1; }
  source /tmp/install/setup.bash
  ros2 launch capcup_driver capcup.launch.py site_dir:=/repo/sites/$SITE fake_vacuum:=true >/out/launch.txt 2>&1 &
  python3 -m pytest -v -p no:cacheprovider /repo/tests/capcup /repo/tests/skills 2>&1 | tee /out/pytest.txt; exit \${PIPESTATUS[0]}
" || rc=$?
if [ "$rc" = 0 ]; then rm -rf "$OUT"; else echo "test_tools failed (rc=$rc), output kept in $OUT" >&2; fi
exit "$rc"
