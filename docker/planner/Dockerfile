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

# Isaac's default ground plane (Isaac Lab GroundPlaneCfg) + its textures, so the MPPI scene build needs no internet
# (it downloaded these from NVIDIA's S3 at run time; test agent T33). Pinned by sha256 (files of 2025-10-14).
# OmniPBR.mdl, also referenced, ships with Isaac Sim.
ARG ISAAC_ASSETS=https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Environments/Grid
RUN set -e; d=/opt/isaac_assets/Environments/Grid; mkdir -p $d/Materials/Textures; cd $d \
    && curl -fsSL -o default_environment.usd ${ISAAC_ASSETS}/default_environment.usd \
    && for t in WireframeBlur_basecolor WireframeBlur_blue Wireframe_blue; do \
           curl -fsSL -o Materials/Textures/$t.png ${ISAAC_ASSETS}/Materials/Textures/$t.png; done \
    && printf '%s\n' \
       "8a21c317d638d33a4e6c20c958a7ed8f4d8c5195efd7e20f1b85178b4a15638a  default_environment.usd" \
       "deab39088eaf39ed7be951966c5e6bd5e58b90ab64743b0cdc8e5579fd5a99d9  Materials/Textures/WireframeBlur_basecolor.png" \
       "44fc0e82007bb0bd94631cd7ec45986a71ed74b9413899fbffac13efe151be4b  Materials/Textures/WireframeBlur_blue.png" \
       "940b96ee8c8e53bd1acbc770d4ca0ac98bed9c96439ab386d1b27dbc9790cc4e  Materials/Textures/Wireframe_blue.png" \
       > SHA256SUMS && sha256sum -c SHA256SUMS
ENV BIN_CLEARING_GROUND_USD=/opt/isaac_assets/Environments/Grid/default_environment.usd

# Kit extensions Isaac Lab @ ISAACLAB_COMMIT needs but isaacsim 5.1.0.0 does not ship (it has urdf importer 2.4.30,
# Isaac Lab pins =2.4.31). Kit fetched them from NVIDIA's registry at EVERY run into the container (lost on exit) and
# fails offline. Pinned by sha256; unpacked where Kit's registry puts them (run logs 2026-11-19).
ARG KIT_REGISTRY=https://d4i3qtqj3r0z5.cloudfront.net
RUN set -e; d=/opt/venv/lib/python3.11/site-packages/isaacsim/kit/data/Kit/Isaac-Sim/5.1/exts/3; mkdir -p $d; cd /tmp \
    && curl -fsSL -o urdf.zip "${KIT_REGISTRY}/isaacsim.asset.importer.urdf-2.4.31%2B107.3.3.lx64.r.cp311.zip" \
    && curl -fsSL -o pip.zip "${KIT_REGISTRY}/omni.kit.pip_archive-d38fa9ecd1fb6df4.zip" \
    && printf '%s\n' "665cc13d48842ac6d0c203359819c1d5c767a1d920fb8a00179e147ec2fb5201  urdf.zip" \
                     "3a0d0623be2349a2861f566dc8d5bdb5040c11b0977bb255c6d522b4be7bd682  pip.zip" | sha256sum -c - \
    && python -c "import zipfile,sys; zipfile.ZipFile(sys.argv[1]).extractall(sys.argv[2])" urdf.zip \
           $d/isaacsim.asset.importer.urdf-2.4.31+107.3.3.lx64.r.cp311 \
    && python -c "import zipfile,sys; zipfile.ZipFile(sys.argv[1]).extractall(sys.argv[2])" pip.zip \
           $d/omni.kit.pip_archive-0.0.0+69cbf6ad.lx64.cp311 \
    && rm urdf.zip pip.zip

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
