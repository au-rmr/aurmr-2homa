#!/usr/bin/env bash
# One camera-calibration sample (docs: tests/perception/handeye_solve.py). Run on the compute host with the arm held
# still in freedrive (program NOT playing), the shell container $SHELL_CT up on the robot domain and the Kinect
# container publishing on domain $CAM_DOMAIN. Sends nothing to the robot.
#   tests/arm/handeye_pose.sh <run_dir> <pose_name>
set -eu
RUN=$1; NAME=$2; SHELL_CT=${SHELL_CT:-calshell}; CAM_DOMAIN=${CAM_DOMAIN:-77}; IMAGE=${KINECT_IMAGE:-aurmr-2homa/kinect:draft}
mkdir -p "$RUN"
docker exec "$SHELL_CT" bash -c "source /ws/install/setup.bash; cd /repo; python3 tests/arm/handeye_sample.py /tmp/cal $NAME"
docker cp "$SHELL_CT:/tmp/cal/${NAME}_tf.json" "$RUN/"
docker run --rm --net=host -e ROS_DOMAIN_ID=$CAM_DOMAIN -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -e ROBOT_LAN_IP=127.0.0.1 \
  -v "$PWD":/repo:ro -v "$(cd "$RUN" && pwd)":/out "$IMAGE" \
  bash -c "source /opt/ros/jazzy/setup.bash; python3 /repo/tests/perception/capture_frames.py /out/$NAME.npz"
