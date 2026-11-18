#!/usr/bin/env bash
# Perception tests in the perception image on a GPU: segmentation (perception/uois) against recorded outputs, the bin
# tracker (perception/bin_tracker) on a synthetic pod, and the
# camera-calibration solver on synthetic data. Weights from AURMR_WEIGHTS (default ~/aurmr-data/weights), read-only.
# Tests the package copy INSTALLED in the image: rebuild it after changes to perception/uois:
#   docker build -f docker/perception/Dockerfile -t aurmr-2homa/perception:draft .
set -eu
cd "$(git rev-parse --show-toplevel)"
W=${AURMR_WEIGHTS:-$HOME/aurmr-data/weights}
IMAGE=${PERCEPTION_IMAGE:-aurmr-2homa/perception:draft}
docker run --rm --gpus all --network none -v "$PWD":/ws:ro -v "$W":/weights:ro \
  -e UOIS_WEIGHTS=/weights/uois_mask2former/model_final_stack.pth -e PYTHONDONTWRITEBYTECODE=1 -w /ws "$IMAGE" \
  python3 -m pytest -q -p no:cacheprovider -W ignore perception/uois/tests perception/bin_tracker/tests tests/perception "$@"
