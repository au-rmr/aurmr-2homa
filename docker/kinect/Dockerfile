# Azure Kinect ROS 2 driver on Jazzy (optigrasp step 1, spike passed 2026-11-16 on the compute host).
# libk4a 1.4.1: Microsoft's Ubuntu 18.04 debs (no noble packages exist); they install and run on 24.04.
# Driver: microsoft/Azure_Kinect_ROS_Driver branch humble, pinned, plus docker/kinect/jazzy-includes.patch
# (cv_bridge + tf2_ros headers moved to .hpp on Jazzy).
# Run: GPU + an X display (the depth engine needs a GLX context: without DISPLAY, /tmp/.X11-unix and the
# Xauthority it fails "Depth engine create and initialize failed with error code: 204"; i.e. someone is logged in
# to the host's desktop; the host's DISPLAY and Xauthority path are in sites/<site>/hosts/<host>/display.env),
# device cgroup rule 'c 189:* rmw', /dev/bus/usb,
# host net, and the host kernel buffers from docker/host/60-ros2-dds.conf for 3072P images.
# The libk4a postinst asks for Microsoft's SDK EULA; it is preseeded below (accepted 2026-11-16 by the dev agent for the user).
ARG BASE=ghcr.io/au-rmr/aurmr-2homa/base:dev
FROM ${BASE}

ARG K4A_URL=https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk
ARG DRIVER_COMMIT=6ffb95a
RUN apt-get update \
    && apt-get install -y --no-install-recommends curl ca-certificates git \
       ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-tf2-geometry-msgs ros-jazzy-angles \
       ros-jazzy-joint-state-publisher ros-jazzy-xacro ros-jazzy-robot-state-publisher \
    && cd /tmp \
    && curl -fsSLO ${K4A_URL}/libk4a1.4/libk4a1.4_1.4.1_amd64.deb \
    && curl -fsSLO ${K4A_URL}/libk4a1.4-dev/libk4a1.4-dev_1.4.1_amd64.deb \
    && echo "c1c63f81641eed1326136a44e5a5fd229e1e6315b2b20b2dddb5523a556c9329  libk4a1.4_1.4.1_amd64.deb" | sha256sum -c \
    && echo "08303094b9ad36ea74c19bc8b8950c97055e73dd2e8bd18e2af5e165a2289cd2  libk4a1.4-dev_1.4.1_amd64.deb" | sha256sum -c \
    && echo "libk4a1.4 libk4a1.4/accepted-eula-hash string 0f5d5c5de396e4fee4c0753a21fee0c1ed726cf0316204edda484f08cb266d76" | debconf-set-selections \
    && echo "libk4a1.4 libk4a1.4/accept-eula boolean true" | debconf-set-selections \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y ./libk4a1.4_1.4.1_amd64.deb ./libk4a1.4-dev_1.4.1_amd64.deb \
    && rm -f /tmp/*.deb && rm -rf /var/lib/apt/lists/*

COPY docker/kinect/jazzy-includes.patch /tmp/jazzy-includes.patch
RUN mkdir -p /opt/kinect_ws/src && cd /opt/kinect_ws/src \
    && git clone -b humble https://github.com/microsoft/Azure_Kinect_ROS_Driver.git \
    && cd Azure_Kinect_ROS_Driver && git checkout ${DRIVER_COMMIT} && git apply /tmp/jazzy-includes.patch \
    && . /opt/ros/jazzy/setup.sh && cd /opt/kinect_ws \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log /tmp/jazzy-includes.patch
