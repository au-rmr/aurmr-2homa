# Base image: ROS 2 Jazzy + Cyclone DDS, pinned by digest. apt packages: apt-packages.txt.
FROM ros:jazzy-ros-base@sha256:8eed0a4669bbf43ee331878a4c39bd8ebddea8e3312a58b4fae8edb14526b617
COPY docker/base/apt-packages.txt /tmp/apt-packages.txt
RUN apt-get update \
    && grep -vE '^\s*(#|$)' /tmp/apt-packages.txt | xargs apt-get install -y --no-install-recommends \
    && rm -rf /var/lib/apt/lists/* /tmp/apt-packages.txt
COPY docker/cyclonedds.xml /etc/cyclonedds.xml
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///etc/cyclonedds.xml
