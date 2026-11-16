# Perception inference image (optigrasp step 2 DRAFT, 2026-11-16): Jazzy py3.12 + torch 2.x for
# Mask2Former (UOIS_multi_frame, lab-modified detectron2 0.6, pure Python) and Depth-Anything (optigrasp).
# Inference only: no training stack. Lab model code is NOT in this image (D6 open); weights are bind-mounted
# read-only from the host (~/aurmr-data/weights), never baked in.
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
