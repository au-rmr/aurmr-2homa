"""Bin item segmentation with per-item embeddings (multi-frame Mask2Former). See README.md."""
from .segment import CONFIG, Segmenter, make_config

__all__ = ["CONFIG", "Segmenter", "make_config"]
