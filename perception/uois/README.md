# uois: bin item segmentation

Colour crops of one bin in, a label mask of the items and one 256-number embedding per item out. The embeddings are
trained so the same item looks alike across stows, which is what the bin tracker uses to tell items apart. Plain
Python, no ROS (the ROS 2 node lives elsewhere in perception/); kept self-contained so it can move to its own repo
(docs/how_to/standalone_packages.md).

```python
from uois import Segmenter
seg = Segmenter("model_final_stack.pth")                 # CUDA
labels, embeddings = seg.segment([crop_before, crop_after])   # oldest first; rules in src/uois/segment.py
```

## What it is
- Model: multi-frame Mask2Former with re-identification embeddings (`FrameMaskFormerV5`, sequence transformer
  decoder), written by Yi Li with Thomas (full name to be added), 2022-2023. Initialised from Meta's COCO panoptic
  Mask2Former R50 and trained by the lab on synthetic bin images (256 x 256). Weights: `model_final_stack.pth`
  (lab-trained; not published yet, kept with the lab's weights).
- Robot integration (the segmentation service this ports, its single-frame and multi-mask behaviour): Soofiyan Atar,
  2023.
- Port (2026): only the inference path, unchanged except imports; our own small predictor (`segment.py`).

## Where it came from
- `src/uois/frame/`: 16 files of `mask2former_frame` from the version the workcell ran: au-rmr/aurmr_perception
  branch `UOIS_multi_frame` @ 253c502 (2023-12-01), as checked out in the `aurmr_opti_grasp_rebased` workspace.
  Its origin is Yi Li's fork of Mask2Former (`liyi14/UIE`, no longer public; copy on the old workcell drive).
  Only change: `from mask2former_frame.` imports became `from uois.frame.`.
- `src/uois/configs/`: the service's config and its base, unchanged.
- Upstream, pinned in `docker/perception/Dockerfile`: facebookresearch/detectron2 @ 717ab9f (the base of the lab's
  copy) and facebookresearch/Mask2Former @ 9b0651c (the base of Yi's fork), with `patches/`:
  - detectron2: `np.bool` -> `bool` (numpy >= 1.24), `Image.LINEAR` -> `Image.BILINEAR` (Pillow >= 10),
    `Boxes` indexing moved to the boxes' device (the lab's fix);
  - Mask2Former: no `evaluation` import at package load (it needs detectron2's compiled ops; training only).
- Weights load by exact name (strict), not through detectron2's checkpointer: its name-matching heuristics put this
  model's nested layers in the wrong places (the lab had switched them off in its copy).

## Check
`UOIS_WEIGHTS=<model_final_stack.pth> pytest perception/uois/tests` in the perception image (`scripts/test_perception.sh`).
Five recorded service calls (bins 1H, 2G, 2H; 1 and 2 frames): same items, mask IoU >= 0.9997, embeddings identical
(cosine 1.0000) to the original code on torch 1.13.

## Licence
MIT. `src/uois/frame/` derives from Mask2Former (MIT, Meta); detectron2 is Apache-2.0 (not copied, fetched at build).
