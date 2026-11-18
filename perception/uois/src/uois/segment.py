"""Bin item segmentation: colour crops of one bin, oldest first, in; a label mask and one embedding per item out.

Port of the workcell's segmentation service (uois_service, ROS 1; Soofiyan Atar, 2023) around Yi Li's multi-frame
Mask2Former (uois.frame). Behaviour kept from the service, including what is surprising:
- one frame: only the model's top-scoring item comes back (label 1), with its embedding; nothing found gives an
  all-zero mask and an all-ones embedding;
- several frames (the same bin after each stow): every item found in the LAST frame, labelled 1..n in model order,
  a later item wins where masks overlap; items whose last-frame embedding is missing are skipped in the embeddings
  but still get a label.
Frames are BGR uint8 (OpenCV order): what the workcell sent. The Kinect publishes bgra8; the tracker cropped its first
three channels and the service passed them on (labelled rgb8, but the data was BGR, as the recorded crops show).
"""
import os

import numpy as np
import torch
from detectron2.config import get_cfg
from detectron2.data import transforms as T
from detectron2.modeling import build_model
from detectron2.projects.deeplab import add_deeplab_config
from mask2former import add_maskformer2_config

from .frame.config import add_maskformer2_frame_config
from .frame import frame_maskformer_model_v5  # noqa: F401  registers FrameMaskFormerV5
from .frame.modeling.meta_arch import frame_mask_former_head  # noqa: F401  registers FrameMaskFormerHead
from .frame.modeling.transformer_decoder import sequenceformer_transformer_decoder  # noqa: F401

CONFIG = os.path.join(os.path.dirname(__file__), "configs",
                      "256x256_synbin_v0p3_stackXYZ011010_optRPY110100_randXYZ110_2s12345_v6_v76_contra_1p0_"
                      "softmax_0p1_21k24k.yaml")
# The service's overrides (uois_service uois_multi_frame.py).
SERVICE_OPTS = ["TEST.DETECTIONS_PER_IMAGE", "100", "DATALOADER.NUM_WORKERS", "0", "INPUT.SAMPLING_FRAME_NUM", "10",
                "MODEL.REID.TEST_MATCH_THRESHOLD", "0.2", "MODEL.MASK_FORMER.TEST.INFERENCE_THRESHOLD", "0.6"]
EMBED_DIM = 256


def make_config(weights, config=CONFIG, opts=SERVICE_OPTS, device="cuda"):
    cfg = get_cfg()
    add_deeplab_config(cfg)
    add_maskformer2_config(cfg)
    add_maskformer2_frame_config(cfg)
    cfg.LOAD_DATASET_INTO_MEMORY = False      # key added by the lab's Mask2Former copy; unused at inference
    cfg.merge_from_file(config)
    cfg.merge_from_list(list(opts) + ["MODEL.WEIGHTS", weights, "MODEL.DEVICE", device])
    cfg.freeze()
    return cfg


class Segmenter:
    def __init__(self, weights, device="cuda", config=CONFIG, opts=SERVICE_OPTS):
        self.cfg = make_config(weights, config, opts, device)
        self.model = build_model(self.cfg).eval()
        # Exact names, strict. Not detectron2's DetectionCheckpointer: for .pth files it renames weights by
        # name-matching heuristics, which put this model's nested layers in the wrong places (the lab's detectron2
        # copy had the heuristics commented out for that reason).
        state = torch.load(weights, map_location="cpu", weights_only=True)["model"]
        self.model.load_state_dict(state, strict=True)
        self.aug = T.ResizeShortestEdge([self.cfg.INPUT.MIN_SIZE_TEST] * 2, self.cfg.INPUT.MAX_SIZE_TEST)
        self.input_format = self.cfg.INPUT.FORMAT
        self.device = device

    @torch.no_grad()
    def predict(self, frames):
        """The model's raw output for a list of H x W x 3 uint8 frames (detectron2 DefaultPredictor preprocessing)."""
        images = []
        for img in frames:
            if self.input_format == "RGB":
                img = img[:, :, ::-1]
            height, width = img.shape[:2]
            img = self.aug.get_transform(img).apply_image(img)
            images.append(torch.as_tensor(img.astype("float32").transpose(2, 0, 1)))
        return self.model([{"image": images, "height": height, "width": width}])

    def segment(self, frames):
        """Returns (labels H x W uint8, embeddings N x 256 float). See the module docstring for the rules."""
        if len(frames) == 1:
            return self._single(frames)
        return self._sequence(frames)

    def _single(self, frames):
        inst = self.predict(frames)[0]["instances"]
        h, w = frames[0].shape[:2]
        if len(inst.scores) == 0:
            return np.zeros((h, w), np.uint8), np.ones((1, EMBED_DIM))
        mask = inst.pred_masks[0].detach().cpu().numpy().astype(np.uint8)
        return mask, inst.reid_embed[0].detach().cpu().numpy().reshape(1, EMBED_DIM)

    def _sequence(self, frames):
        p = self.predict(frames)
        h, w = frames[0].shape[:2]
        last = len(frames) - 1
        labels = np.zeros((h, w), np.uint8)
        n = 1
        for obj_masks in p["pred_masks"]:             # per object: one mask per frame
            m = obj_masks[last].detach().cpu().numpy().astype(np.uint8)
            if m.any():
                labels += n * m
                labels[labels > n] = n
                n += 1
        emb = [e[last] for e in p["reid_embed"] if e[last] is not None]
        if not emb:
            return np.zeros((h, w), np.uint8), np.zeros((0, EMBED_DIM))
        return labels, np.stack([e.detach().cpu().numpy() if torch.is_tensor(e) else np.asarray(e) for e in emb])
