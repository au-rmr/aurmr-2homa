# Base image: ROS 2 Jazzy + Cyclone DDS, pinned by digest.
FROM ros:jazzy-ros-base@sha256:8eed0a4669bbf43ee331878a4c39bd8ebddea8e3312a58b4fae8edb14526b617
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-demo-nodes-cpp ros-jazzy-demo-nodes-py \
    && rm -rf /var/lib/apt/lists/*
COPY docker/cyclonedds.xml /etc/cyclonedds.xml
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///etc/cyclonedds.xml
