# 2x RealSense D435i + AprilTag detection for the MPC/MCTS demo (S7 stage 4), on the cameras host (winthrop).
# realsense2_camera 4.59 (librealsense 2.59, V4L backend) and apriltag_ros 3.4 (christianrauch; families incl. 36h11
# and Standard41h12) from the Jazzy apt repo. Spike 2026-11-20 on winthrop: both cameras enumerate (215222077034
# fw 5.14.0, 909512071469 fw 5.12.5).
# Run: device cgroup rules 'c 81:* rmw' (video) + 'c 189:* rmw' (usb) and /dev mounted (the nodes re-enumerate
# after a camera reset), host net. See compose/compute.yaml services realsense + mpc_tag_viewer (the viewer runs
# from this image on the GUI host: OpenCV + apriltag_msgs).
ARG BASE=ghcr.io/au-rmr/aurmr-2homa/base:dev
FROM ${BASE}
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       ros-jazzy-realsense2-camera ros-jazzy-apriltag-ros ros-jazzy-image-proc \
       ros-jazzy-compressed-image-transport python3-opencv \
    && rm -rf /var/lib/apt/lists/*
