#!/usr/bin/env bash
# One camera-calibration sample (docs: tests/perception/handeye_solve.py). Run on the compute host with the arm held
# still in freedrive (program NOT playing), the shell container $SHELL_CT up on the robot domain and the camera
# container publishing on domain $CAM_DOMAIN. Sends nothing to the robot.
#   tests/arm/handeye_pose.sh <run_dir> <pose_name>
# Kinect (default): one camera on this host -> <run_dir>/<pose>.npz + <pose>_tf.json.
# RealSense (MPC demo): CAMERAS="camera1 camera2" CAM_LAN_IP=<this host's robot-LAN IP> (the cameras are on another
#   host, domain 77 over the LAN) -> <run_dir>/<camera>/<pose>.npz + <pose>_tf.json, one solve per camera directory.
set -eu
RUN=$1; NAME=$2; SHELL_CT=${SHELL_CT:-calshell}; CAM_DOMAIN=${CAM_DOMAIN:-77}
mkdir -p "$RUN"
docker exec "$SHELL_CT" bash -c "source /ws/install/setup.bash; cd /repo; python3 tests/arm/handeye_sample.py /tmp/cal $NAME"
if [ -z "${CAMERAS:-}" ]; then
  IMAGE=${KINECT_IMAGE:-aurmr-2homa/kinect:draft}
  docker cp "$SHELL_CT:/tmp/cal/${NAME}_tf.json" "$RUN/"
  docker run --rm --net=host -e ROS_DOMAIN_ID=$CAM_DOMAIN -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -e ROBOT_LAN_IP=127.0.0.1 \
    -v "$PWD":/repo:ro -v "$(cd "$RUN" && pwd)":/out "$IMAGE" \
    bash -c "source /opt/ros/jazzy/setup.bash; python3 /repo/tests/perception/capture_frames.py /out/$NAME.npz"
  exit 0
fi
IMAGE=${REALSENSE_IMAGE:-aurmr-2homa/realsense:draft}
: "${CAM_LAN_IP:?set CAM_LAN_IP: this host robot-LAN address, the cameras are on another host}"
for CAM in $CAMERAS; do
  mkdir -p "$RUN/$CAM"
  docker cp "$SHELL_CT:/tmp/cal/${NAME}_tf.json" "$RUN/$CAM/"
done
# One container for all cameras (discovery once); each camera's frame right after the arm sample, arm held still.
docker run --rm --net=host -e ROS_DOMAIN_ID=$CAM_DOMAIN -e ROBOT_LAN_IP="$CAM_LAN_IP" -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -v "$PWD":/repo:ro -v "$(cd "$RUN" && pwd)":/out "$IMAGE" \
  bash -c "source /opt/ros/jazzy/setup.bash; rc=0; for c in $CAMERAS; do
    python3 /repo/tests/perception/capture_frames.py /out/\$c/$NAME.npz --camera /\$c --realsense || rc=1; done; exit \$rc"
