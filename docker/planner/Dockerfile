# Planner image (role: planner, GPU host). Isaac Sim 5.1 + Isaac Lab + MPPI, Python 3.11.
# Versions = the env the MPC + MCTS demo ran in on pinkman (env_isaaclab, 2026-11-18); see docker/planner/README.md.
# ROS 2: Isaac Sim's bundled Jazzy rclpy (built for its Python 3.11), Cyclone DDS like the rest of the stack.
# LOCAL BUILD ONLY: contains NVIDIA Isaac Sim (NVIDIA licence); never push this image to a public registry.
FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive
# Libraries Isaac Sim's Kit needs for headless + GUI rendering (driver libs come from the NVIDIA container toolkit).
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl git build-essential cmake \
        libglu1-mesa libgl1 libegl1 libvulkan1 libxt6 libxrandr2 libxinerama1 libxcursor1 libxi6 \
        libsm6 libice6 libxkbcommon0 libxkbcommon-x11-0 libgomp1 libglib2.0-0 libxext6 libx11-6 libxrender1 \
    && rm -rf /var/lib/apt/lists/*

# Python 3.11 via uv (Ubuntu 22.04 ships 3.10); one venv for everything.
COPY --from=ghcr.io/astral-sh/uv:0.5.11 /uv /usr/local/bin/uv
# Long timeout: the CUDA and Isaac Sim wheels are large (a 30 s default timed out on pinkman).
ENV UV_PYTHON_INSTALL_DIR=/opt/python VIRTUAL_ENV=/opt/venv PATH=/opt/venv/bin:$PATH UV_HTTP_TIMEOUT=600 UV_CONCURRENT_DOWNLOADS=4
RUN uv python install 3.11 && uv venv --python 3.11 /opt/venv && uv pip install pip

# torch first (cu128 for Blackwell / RTX 5090), then Isaac Sim from NVIDIA's index.
RUN uv pip install --index-url https://download.pytorch.org/whl/cu128 torch==2.7.0 torchvision==0.22.0
RUN uv pip install --extra-index-url https://pypi.nvidia.com "isaacsim[all,extscache]==5.1.0.0"

# Isaac Lab, pinned to the upstream commit pinkman ran; only the packages the planner imports.
ARG ISAACLAB_COMMIT=4df6560e187f2cc66685b41b21b259f4485d0c22
RUN git clone https://github.com/isaac-sim/IsaacLab.git /opt/IsaacLab \
    && git -C /opt/IsaacLab checkout ${ISAACLAB_COMMIT} \
    && uv pip install -e /opt/IsaacLab/source/isaaclab -e /opt/IsaacLab/source/isaaclab_assets

# MPPI (third-party, tud-amr), pinned. --no-deps: its declared deps (urdfenvs, gym) are for its examples
# and pin a gymnasium pre-release no longer on PyPI; the library imports only torch, numpy, scipy, ghalton.
RUN uv pip install --no-deps "mppi_torch @ git+https://github.com/tud-amr/mppi_torch.git@75e17e87fd42a803d3227258d14c4231cbcbc0ec" \
    && uv pip install numpy==1.23.5 scipy==1.15.3 ghalton==0.6.1 hydra-core==1.3.2 omegaconf==2.3.0 pyyaml h5py

# UR16e meshes for policies/bin_clearing (its URDF is in the repo; meshes identical to upstream, BSD-3-Clause).
ARG UR_DESCRIPTION_COMMIT=6507bb6756d0065d49b76230ff55f875f099a827
RUN git clone --filter=blob:none --sparse https://github.com/ros-industrial/universal_robot.git /tmp/ur \
    && git -C /tmp/ur checkout ${UR_DESCRIPTION_COMMIT} \
    && git -C /tmp/ur sparse-checkout set ur_description/meshes/ur10e ur_description/meshes/ur16e \
    && mkdir -p /opt/ur_description && cp -r /tmp/ur/ur_description/meshes /opt/ur_description/ \
    && rm -rf /tmp/ur
ENV UR_DESCRIPTION_MESHES=/opt/ur_description/meshes

# Bundled ROS 2 Jazzy (rclpy, Cyclone) from the isaacsim pip package.
ENV OMNI_KIT_ACCEPT_EULA=YES \
    ROS_DISTRO=jazzy \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=file:///etc/cyclonedds.xml \
    ISAAC_ROS2=/opt/venv/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/jazzy
ENV LD_LIBRARY_PATH=${ISAAC_ROS2}/lib \
    PYTHONPATH=${ISAAC_ROS2}/rclpy:/repo/policies/bin_clearing/src:/repo/policies/bin_clearing_ros
COPY docker/cyclonedds.xml /etc/cyclonedds.xml
# Vulkan + EGL loaders need NVIDIA's ICD files; the container toolkit mounts the driver libraries
# (NVIDIA_DRIVER_CAPABILITIES=all) but not these. Without them Kit has no GPU device (no viewer, no GPU PhysX).
RUN mkdir -p /etc/vulkan/icd.d /usr/share/glvnd/egl_vendor.d \
    && printf '{"file_format_version":"1.0.1","ICD":{"library_path":"libGLX_nvidia.so.0","api_version":"1.3"}}\n' \
       > /etc/vulkan/icd.d/nvidia_icd.json \
    && printf '{"file_format_version":"1.0.0","ICD":{"library_path":"libEGL_nvidia.so.0"}}\n' \
       > /usr/share/glvnd/egl_vendor.d/10_nvidia.json
WORKDIR /repo
