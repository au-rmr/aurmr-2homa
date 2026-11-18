# bin_tracker: which item is where in the pod

Keeps per-bin item masks up to date across stows, picks and re-captures, so the robot can ask for "item X" and get
its mask in the camera image. Plain Python: no ROS, the segmenter is passed in (on the robot: `uois.Segmenter`).
Kept self-contained so it can move to its own repo (docs/how_to/standalone_packages.md).

```python
from bin_tracker import BinTracker, Status
tracker = BinTracker(bounds, segmenter.segment, empty_pod_depth)   # bounds: {bin: [r1, r2, c1, c2]} in the image
tracker.stow("2H", "item-42", bgr, depth)      # -> Status.OK / NO_OBJ_STORED / UNDERSEGMENTATION / ...
mask = tracker.get_obj_mask("item-42")         # full-image uint8, None in a bad bin
tracker.pick("item-42", bgr, depth)
```
How it works and what it keeps or changes from the original: `src/bin_tracker/tracker.py` docstring.

## Credits and origin
Port of the workcell's SegNet tracker (au-rmr/aurmr_tahoma `aurmr_unseen_object_clustering/tools/segmentation_net.py`
@ 118051e, branch opti_grasp_integration, as the `aurmr_opti_grasp_rebased` workspace ran it): Soofiyan Atar
(2022-2023), with fixes by Nick Walker, Henri Fung and Markus Grotz (2024). Its bookkeeping in the ROS 1 node
(`aurmr_perception/pod_model.py`) moves to the ROS 2 node.

## Checks
- `tests/test_tracker.py` (no GPU): a synthetic pod with coloured blocks and a fake segmenter. Includes the 3-item case
  that the original's update_masks bug failed (below).
- Old vs port: the original file (ROS imports stubbed) and the port, run side by side on 300 random sequences of 12
  stows / picks / re-captures / unseen-item stows with the same fake segmenter, gave identical status, item table,
  masks and embeddings at every step (2026-11-17, before the fix below; harness kept with the lab's reference data, not in this repo).

## Fixed after the comparison (2026-11-17)
`update_masks` stored the embeddings by the inverse of the label mapping. With two items in a bin that is harmless;
with three or more, labels can come back from the model in a cycle, and the next capture then swapped identities
(synthetic: 21 of 40 label orders after three re-captures; 0 of 40 with the fix). Now `embeddings_new[i] =
embeddings[recs[i] - 1]`. To compare with the original again, apply the same line to the old file.
