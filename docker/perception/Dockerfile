# Perception inference image (DRAFT): Jazzy py3.12 + torch 2.x for bin segmentation (perception/uois: multi-frame
# Mask2Former on upstream detectron2 + Mask2Former, pure Python) and OptiGrasp. Build from the repo root:
#   docker build -f docker/perception/Dockerfile -t aurmr-2homa/perception:draft .
# Inference only: no training stack. Weights are bind-mounted read-only from the host (~/aurmr-data/weights), never
# baked in.
# numpy stays at Ubuntu's 1.26.4 (cv_bridge and the apt scientific stack are built against it).
ARG BASE=ghcr.io/au-rmr/aurmr-2homa/base:dev
FROM ${BASE}
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-opencv python3-scipy python3-matplotlib python3-sklearn python3-pandas python3-skimage \
      python3-pil python3-h5py python3-yaml python3-tabulate python3-termcolor python3-cloudpickle \
      ros-jazzy-cv-bridge \
    && rm -rf /var/lib/apt/lists/*
RUN echo "numpy==1.26.4" > /etc/pip-constraints.txt \
    && pip3 install --no-cache-dir --break-system-packages -c /etc/pip-constraints.txt \
       --index-url https://download.pytorch.org/whl/cu124 torch==2.5.1 torchvision==0.20.1 \
    && pip3 install --no-cache-dir --break-system-packages -c /etc/pip-constraints.txt \
       fvcore==0.1.5.post20221221 iopath==0.1.9 yacs==0.1.8 omegaconf==2.3.0 pycocotools==2.0.8 \
       timm==0.9.16 huggingface_hub==0.23.1 safetensors==0.4.5 tqdm==4.66.5
# Mask2Former imports the compiled MultiScaleDeformableAttention op at import time and hard-fails without it,
# but its forward falls back to the pure-PyTorch ms_deform_attn_core_pytorch on any exception. This stub makes
# the import succeed and the CUDA call raise, so the fallback runs. Replace with a compiled op if too slow.
RUN pip3 install --no-cache-dir --break-system-packages --no-deps fairscale==0.4.13
RUN printf '# stub: see docker/perception/Dockerfile\n' \
    > /usr/local/lib/python3.12/dist-packages/MultiScaleDeformableAttention.py
# OptiGrasp inference (au-rmr/optigrasp, pinned by commit; its tests: OPTIGRASP_WEIGHTS=... pytest in a clone).
RUN pip3 install --no-cache-dir --break-system-packages --no-deps \
      "optigrasp @ git+https://github.com/au-rmr/optigrasp.git@e2de5327ebc235c265e880dea290f3489fc40664"
# Upstream detectron2 and Mask2Former, pinned, used from source (no compiled ops; see the stub above).
# detectron2 717ab9f = the base of the lab's copy; Mask2Former 9b0651c = its last commit, the base of the lab's fork.
# perception/uois/patches: the lab's inference-relevant fixes (numpy >= 1.24, Pillow >= 10, box index device; no
# evaluation import, it needs the compiled ops).
COPY perception/uois/patches /tmp/uois-patches
RUN git clone -q https://github.com/facebookresearch/detectron2.git /opt/detectron2 \
    && git -C /opt/detectron2 checkout -q 717ab9f0aeca216a2f800e43d705766251ba3a55 \
    && git -C /opt/detectron2 apply /tmp/uois-patches/detectron2-717ab9f.patch \
    && git clone -q https://github.com/facebookresearch/Mask2Former.git /opt/Mask2Former \
    && git -C /opt/Mask2Former checkout -q 9b0651c6c1d5b3af2e6da0589b719c514ec0d69a \
    && git -C /opt/Mask2Former apply /tmp/uois-patches/mask2former-9b0651c.patch \
    && rm -rf /opt/detectron2/.git /opt/Mask2Former/.git /tmp/uois-patches
ENV PYTHONPATH=/opt/detectron2:/opt/Mask2Former
COPY perception/uois /tmp/uois
RUN pip3 install --no-cache-dir --break-system-packages --no-deps --no-build-isolation /tmp/uois && rm -rf /tmp/uois
