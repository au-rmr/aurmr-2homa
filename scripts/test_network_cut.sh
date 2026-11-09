#!/usr/bin/env bash
# Cuts the network between a demo and the robot stack in the middle of a move, on mock hardware.
# Two containers on an internal Docker network, which cannot reach anything else.
set -eu
cd "$(git rev-parse --show-toplevel)"
SITE=${AURMR_SITE:-uw_lab}
LIMIT_S=${LIMIT_S:-1.0}
ID=$$
NET=aurmr-cut-$ID; STACK=aurmr-cut-stack-$ID; DEMO=aurmr-cut-demo-$ID
SUBNET=172.31.77
IMAGE=ghcr.io/au-rmr/aurmr-2homa/base:dev
WS=$(mktemp -d "${TMPDIR:-/tmp}/cut.XXXXXX")
cleanup() {
  docker rm -f "$STACK" "$DEMO" "$DEMO-2" "$DEMO-stranger" >/dev/null 2>&1 || true
  docker network rm "$NET" >/dev/null 2>&1 || true
}
trap cleanup EXIT
fail() { echo "FAILED: $1, logs in $WS" >&2; docker logs "$STACK" >"$WS/stack.txt" 2>&1 || true; exit 1; }
run() { # name ip command
  docker run -d --name "$1" --network "$NET" --ip "$SUBNET.$2" --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -e ROS_LOG_DIR=/ws/ros_log -e ROBOT_LAN_IP="$SUBNET.$2" -e WORKCELL="/repo/sites/$SITE/workcell.yaml" \
    -v "$PWD":/repo:ro -v "$WS":/ws -w /tmp "$IMAGE" bash -c "source /opt/ros/jazzy/setup.bash; $3" >/dev/null
}
wait_log() { # container text seconds
  local end=$((SECONDS + $3))
  until docker logs "$1" 2>/dev/null | grep -q "$2"; do
    [ "$SECONDS" -lt "$end" ] || fail "$1 never printed $2"
    python3 -c 'import time; time.sleep(0.05)'
  done
}
docker network create --internal --subnet "$SUBNET.0/24" "$NET" >/dev/null
run "$STACK" 2 "
  colcon --log-base /ws/log build --base-paths /repo/robot /repo/end_effectors --build-base /ws/build --install-base /ws/install >/ws/colcon.txt 2>&1
  source /ws/install/setup.bash
  export AURMR_PRIVATE_DOMAIN=142 AURMR_PRIVATE_URI=file:///repo/robot/tahoma_control/config/cyclonedds_private.xml
  export CYCLONEDDS_URI=file:///repo/robot/tahoma_control/config/cyclonedds_shared.xml,\$AURMR_PRIVATE_URI
  ros2 launch tahoma_moveit_config mock.launch.py site_dir:=/repo/sites/$SITE >/ws/mock.txt 2>&1 &
  python3 /repo/tests/mock/fake_sensors.py >/ws/sensors.txt 2>&1 &
  python3 -u /repo/tests/network/watch_arm.py"
wait_log "$STACK" READY 150
# Another host on the robot network sees motion_server, but no controller or driver, on the
# shared domain or on the stack's private one (T12 item 3).
docker run --rm --name "$DEMO-stranger" --network "$NET" --ip "$SUBNET.5" --user "$(id -u):$(id -g)" \
  -e HOME=/tmp -e ROBOT_LAN_IP="$SUBNET.5" -v "$PWD":/repo:ro "$IMAGE" \
  bash -c "source /opt/ros/jazzy/setup.bash; python3 /repo/tests/network/stranger.py" \
  | tee "$WS/stranger.txt" | grep -q '^ISOLATED' || fail "a stranger saw the controllers: $(cat "$WS/stranger.txt")"
echo "another host sees no controller or driver on either domain"
run "$DEMO" 3 "source /ws/install/setup.bash; python3 -u /repo/tests/network/demo.py slow"
wait_log "$DEMO" CONNECTED 60
# CONNECTED prints as the straight move starts.
python3 -c 'import time; time.sleep(1.5)'
BEFORE=$(docker logs "$STACK" 2>/dev/null | grep -c '^STOPPED' || true)
CUT=$(date +%s.%N)
docker network disconnect -f "$NET" "$DEMO"
end=$((SECONDS + 5))
until [ "$(docker logs "$STACK" 2>/dev/null | grep -c '^STOPPED')" -gt "$BEFORE" ]; do
  [ "$SECONDS" -lt "$end" ] || fail "the arm did not stop after the cut"
  python3 -c 'import time; time.sleep(0.05)'
done
AT=$(docker logs "$STACK" 2>/dev/null | grep '^STOPPED' | tail -1 | cut -d' ' -f2)
TOOK=$(python3 -c "print(round($AT - $CUT, 3))")
echo "arm stopped $TOOK s after the cut (limit $LIMIT_S s)"
python3 -c "import sys; sys.exit(0 if 0 < $TOOK < $LIMIT_S else 1)" || fail "stop time $TOOK"
docker rm -f "$DEMO" >/dev/null
run "$DEMO-2" 4 "source /ws/install/setup.bash; python3 -u /repo/tests/network/demo.py quick"
wait_log "$DEMO-2" DONE 60
echo "reconnected and moved after the cut"
rm -rf "$WS" 2>/dev/null || true
echo PASSED
