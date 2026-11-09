#!/usr/bin/env bash
# L2.5: the real-arm bringup (hardware:=ur) against URSim, UR's controller simulator. Tests the
# UR driver, the scaled controller, speed scaling, the dashboard and the controller stopper.
# Two containers on an internal Docker network, which cannot reach anything else.
# Needs the URSim image once: docker pull universalrobots/ursim_e-series:$URSIM_VERSION
# Headless mode: the driver sends its own program. The lab arm runs it from the tablet instead.
set -eu
cd "$(git rev-parse --show-toplevel)"
SITE=${AURMR_SITE:-uw_lab}
URSIM_VERSION=${URSIM_VERSION:-5.23.0}
ID=$$
NET=aurmr-ursim-$ID; SIM=aurmr-ursim-sim-$ID; STACK=aurmr-ursim-stack-$ID
SUBNET=${URSIM_SUBNET:-172.31.56}
IMAGE=ghcr.io/au-rmr/aurmr-2homa/base:dev
WS=$(mktemp -d "${URSIM_OUT:-${TMPDIR:-/tmp}}/ursim.XXXXXX")
chmod 777 "$WS"
rc=1
cleanup() {
  docker logs "$SIM" >"$WS/ursim.txt" 2>&1 || true
  docker rm -f "$STACK" "$SIM" >/dev/null 2>&1 || true
  docker network rm "$NET" >/dev/null 2>&1 || true
  if [ "$rc" = 0 ]; then rm -rf "$WS" 2>/dev/null || true; else echo "test_ursim failed, output kept in $WS" >&2; fi
}
trap cleanup EXIT
docker network create --internal --subnet "$SUBNET.0/24" "$NET" >/dev/null
docker run -d --name "$SIM" --network "$NET" --ip "$SUBNET.101" -e ROBOT_MODEL=UR16 \
  "universalrobots/ursim_e-series:$URSIM_VERSION" >/dev/null
docker run --name "$STACK" --network "$NET" --ip "$SUBNET.2" --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -e ROS_LOG_DIR=/ws/ros_log -e ROBOT_LAN_IP="$SUBNET.2" -e WORKCELL="/repo/sites/$SITE/workcell.yaml" \
  -e PYTEST_ADDOPTS="${PYTEST_ADDOPTS:-}" -v "$PWD":/repo:ro -v "$WS":/ws -w /tmp "$IMAGE" bash -c "
  source /opt/ros/jazzy/setup.bash
  colcon --log-base /ws/log build --base-paths /repo/robot /repo/end_effectors --build-base /ws/build --install-base /ws/install >/ws/colcon.txt 2>&1
  source /ws/install/setup.bash
  export AURMR_PRIVATE_DOMAIN=142 AURMR_PRIVATE_URI=file:///repo/robot/tahoma_control/config/cyclonedds_private.xml
  export CYCLONEDDS_URI=file:///repo/robot/tahoma_control/config/cyclonedds_shared.xml,\$AURMR_PRIVATE_URI
  python3 /repo/tests/ursim/wait_dashboard.py $SUBNET.101 || exit 1
  ros2 launch tahoma_moveit_config robot.launch.py site_dir:=/repo/sites/$SITE hardware:=ur \
    robot_ip:=$SUBNET.101 headless_mode:=true >/ws/launch.txt 2>&1 &
  python3 -m pytest -v -p no:cacheprovider /repo/tests/ursim 2>&1 | tee /ws/pytest.txt; exit \${PIPESTATUS[0]}
" && rc=0 || rc=$?
exit "$rc"
