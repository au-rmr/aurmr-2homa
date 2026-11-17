#!/usr/bin/env bash
# RViz with the robot model, the pod's collision boxes and the live Kinect point cloud, placed by the site's
# camera calibration (sites/<site>/calibration/camera_lower_right.yaml). Read-only, no robot: the arm is drawn in
# the picker's pose, not where it really is. Camera + viewer share an isolated local ROS domain.
# Usage: scripts/view_camera.sh [scene:=pod_bins] [pose:=drop_hide]   (needs DISPLAY + XAUTHORITY, the Kinect, a GPU)
set -eu
cd "$(git rev-parse --show-toplevel)"
SITE=${AURMR_SITE:-uw_lab}
: "${DISPLAY:?needs a desktop session; set DISPLAY}"
DOMAIN=${CAMERA_VIEW_DOMAIN:-77}
NAME=aurmr-view-camera-$$
trap 'docker rm -f "$NAME" >/dev/null 2>&1 || true' EXIT
# 720P colour on 2x2-binned wide depth: a ~260k-point coloured cloud RViz can draw (geometry is the same at any resolution).
docker run -d --name "$NAME" --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all --device-cgroup-rule='c 189:* rmw' \
  -v /dev/bus/usb:/dev/bus/usb --net=host -e ROS_DOMAIN_ID=$DOMAIN -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e ROBOT_LAN_IP=127.0.0.1 -e DISPLAY -e XAUTHORITY=/tmp/.Xauthority -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "${XAUTHORITY:?set XAUTHORITY}":/tmp/.Xauthority:ro ${KINECT_IMAGE:-aurmr-2homa/kinect:draft} \
  bash -c 'source /opt/kinect_ws/install/setup.bash; exec ros2 run azure_kinect_ros_driver node --ros-args \
    -r __ns:=/camera_lower_right -p depth_mode:=WFOV_2X2BINNED -p color_enabled:=true -p color_resolution:=720P \
    -p fps:=5 -p point_cloud:=true -p rgb_point_cloud:=true -p point_cloud_in_depth_frame:=true' >/dev/null
VIEW_NETWORK=host ROS_DOMAIN_ID=$DOMAIN ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  VIEW_EXTRA="python3 /repo/tests/perception/camera_tf.py /repo/sites/$SITE/calibration/camera_lower_right.yaml" \
  scripts/view_scene.sh rviz:=view_scene_camera.rviz scene:=pod_bins "$@"
