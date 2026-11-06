# Base image: ROS 2 Jazzy + Cyclone DDS. TODO(S1): pin the base image by digest.
FROM ros:jazzy-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-demo-nodes-cpp ros-jazzy-demo-nodes-py \
    && rm -rf /var/lib/apt/lists/*
COPY docker/cyclonedds.xml /etc/cyclonedds.xml
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///etc/cyclonedds.xml
