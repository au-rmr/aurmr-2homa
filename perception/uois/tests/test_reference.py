"""The port reproduces the workcell's segmentation service on recorded bin crops.

tests/data/<bin>/color_NNNN.npy: the crops the workcell sent (2G, 2H 2026-03-24; 1H 2026-02-12), oldest first.
expected_nK.npz: the original code's output on the first K crops (torch 1.13.1, RTX 3090): full_mask, embeddings.
Needs a CUDA GPU and the weights: UOIS_WEIGHTS=<model_final_stack.pth> pytest perception/uois/tests
(in the perception image, which has detectron2 + Mask2Former). Tolerances fixed before the first comparison
(2026-11-16): masks are not bit-identical across torch versions.
"""
import os
from pathlib import Path

import numpy as np
import pytest
import torch

DATA = Path(__file__).parent / "data"
WEIGHTS = os.environ.get("UOIS_WEIGHTS")
MASK_IOU, EMBED_COS = 0.95, 0.99
CASES = sorted((b.name, int(f.stem[len("expected_n"):])) for b in DATA.iterdir() for f in b.glob("expected_n*.npz"))

pytestmark = pytest.mark.skipif(not (WEIGHTS and torch.cuda.is_available()),
                                reason="set UOIS_WEIGHTS and run on a CUDA GPU")


@pytest.fixture(scope="module")
def segmenter():
    from uois import Segmenter
    return Segmenter(WEIGHTS)


@pytest.mark.parametrize("bin_name,n", CASES)
def test_matches_recorded_output(segmenter, bin_name, n):
    frames = [np.load(f) for f in sorted((DATA / bin_name).glob("color_*.npy"))[:n]]
    want = np.load(DATA / bin_name / f"expected_n{n}.npz")
    labels, emb = segmenter.segment(frames)
    mo, mn = want["full_mask"], labels
    lo, ln = [l for l in np.unique(mo) if l], [l for l in np.unique(mn) if l]
    assert len(lo) == len(ln), (lo, ln)
    for l in lo:                                  # best-matching new label for each recorded object
        a = mo == l
        iou = max(((a & (mn == m)).sum() / (a | (mn == m)).sum() for m in ln), default=0.0)
        assert iou >= MASK_IOU, (l, iou)
    eo, en = want["embeddings"].reshape(len(want["embeddings"]), -1), emb.reshape(len(emb), -1)
    assert eo.shape == en.shape
    for a, b in zip(eo, en):
        assert a @ b / (np.linalg.norm(a) * np.linalg.norm(b)) >= EMBED_COS
