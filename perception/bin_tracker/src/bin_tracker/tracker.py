"""Which item is where in the pod: per-bin item masks kept up to date across stows and picks.

Port of the workcell's SegNet tracker (aurmr_tahoma aurmr_unseen_object_clustering/tools/segmentation_net.py @ 118051e,
as the aurmr_opti_grasp_rebased workspace ran it; Soofiyan Atar 2022-2023, later fixes by Nick Walker, Henri Fung and
Markus Grotz, 2024). Plain Python: no ROS, no files unless `state_dir` is given; the segmenter is passed in.

How it works, per bin (a pixel rectangle of the camera image):
- An empty-pod depth image is the reference. A stow counts only if the bin's depth changed by > 30 mm somewhere
  (after a 9 x 9 erosion, twice); otherwise NO_OBJ_STORED.
- Every stow / pick / update crops the bin, appends the crop to the bin's history and segments the whole history
  (the multi-frame model looks at the bin's past views). Masks are then forced to the expected item count
  (refine_masks: drop small or merge contained extra masks; too few = UNDERSEGMENTATION, the bin becomes "bad").
- Items keep their label across captures by matching embeddings (dot product, Hungarian assignment) between the
  previous and the new masks; a stowed item takes the label left unmatched.

Kept from the original on purpose (so old and new can be compared on the same captures); see README.md:
- the SIFT fallback is unreachable: matching "fails" only if the best score is < -0.2, but the best starts at 0;
- the whole history goes to the segmenter; the model samples at most 10 frames (more than 10 captures of one bin
  without a reset was reported to fail);
- relabelling assumes background is present: a mask with no 0 pixels shifts every label down by one.
Changed: update_masks stores each item's embedding under its new label (the original used the inverse mapping, which
swaps identities with 3+ items); picking an unknown item returns OBJ_NOT_FOUND (it crashed); a bad bin's masks are not available (the original
asked for a mouse selection, disabled); reset_bin also clears the bin's history and does not fail for a good bin;
debug image writes, prints and the unused point-cloud standardisation are gone.
"""
import enum
import logging
import os

import cv2
import numpy as np
import scipy.ndimage as spy
from scipy.optimize import linear_sum_assignment

log = logging.getLogger("bin_tracker")
EMBED_DIM = 256


class Status(enum.IntEnum):
    OK = 0
    NO_OBJ_STORED = 1
    UNDERSEGMENTATION = 2
    OBJ_NOT_FOUND = 3
    MATCH_FAILED = 4
    IN_BAD_BINS = 5


DEFAULTS = {
    "min_pixel_threshold": 30,     # depth change (camera units, mm) that counts as a new item
    "min_match_count": 3,          # SIFT good matches per item
    "min_area": 100,               # px; extra masks smaller than this are dropped
}


class Bin:
    def __init__(self, bin_id, bounds, init_depth, config):
        self.bin = bin_id
        self.n = 0
        self.config = config
        self.bounds = bounds
        r1, r2, c1, c2 = bounds
        self.current = {"rgb": np.zeros((r2 - r1, c2 - c1, 3), np.uint8), "depth": None, "mask": None,
                        "embeddings": np.array([])}
        self.last = {"rgb": None, "depth": None, "mask": None, "embeddings": np.array([])}
        self.init_depth = init_depth[r1:r2, c1:c2]
        self.prev = {}
        self.history = []          # bin crops sent to the segmenter, oldest first

    def crop(self, img):
        r1, r2, c1, c2 = self.bounds
        return img[r1:r2, c1:c2]

    def update_current(self, rgb, depth):
        self.prev["last"] = self.last.copy()
        self.prev["current"] = self.current.copy()
        self.last = self.current.copy()
        self.current["rgb"] = self.crop(rgb)
        self.current["depth"] = self.crop(depth)
        self.current["embeddings"] = None
        self.current["mask"] = None

    def abort_update(self):
        self.last = self.prev["last"]
        self.current = self.prev["current"]

    def new_object_detected(self, depth):
        depth_now = self.crop(depth)
        depth_bin = self.init_depth if self.current["depth"] is None else self.current["depth"]
        changed = np.abs(depth_bin - depth_now) > self.config["min_pixel_threshold"]
        changed = spy.binary_erosion(changed, structure=np.ones((9, 9), np.uint8), iterations=2)
        return bool(np.sum(changed) > 0)


class BinTracker:
    """bounds: {bin_id: [row_start, row_end, col_start, col_end]} in the colour image.
    segment_fn(list of BGR crops, oldest first) -> (labels H x W, embeddings N x 256), e.g. uois.Segmenter.segment.
    empty_depth: depth image (mm, aligned to colour) of the empty pod."""

    def __init__(self, bounds, segment_fn, empty_depth, config=None, state_dir=None):
        self.config = dict(DEFAULTS, **(config or {}))
        self.segment_fn = segment_fn
        self.state_dir = state_dir
        empty_depth = np.asarray(empty_depth, np.float32)
        self.bins = {b: Bin(b, list(v), empty_depth, self.config) for b, v in bounds.items()}
        self.H, self.W = empty_depth.shape[:2]
        self.items = {}            # item id -> [bin id, label in that bin]
        self.n = 0
        self.bad_bins = []
        self.current = {"rgb": None, "depth": None}
        self.last = {"rgb": None, "depth": None}

    # ---- capture -> masks -------------------------------------------------------------------------------------
    def _capture(self, rgb, depth):
        self.last = self.current.copy()
        self.current = {"rgb": np.asarray(rgb)[..., :3].astype(np.uint8), "depth": np.asarray(depth, np.float32)}

    def segment(self, bin_id):
        """Crop the bin from the current capture, add it to the bin's history, segment the history."""
        b = self.bins[bin_id]
        crop = b.crop(self.current["rgb"]).copy()
        b.history.append(crop)
        if self.state_dir:
            d = os.path.join(self.state_dir, bin_id)
            os.makedirs(d, exist_ok=True)
            np.save(os.path.join(d, f"color_{len(b.history) - 1:04d}.npy"), crop)
        mask, embed = self.segment_fn(list(b.history))
        h, w = crop.shape[:2]
        try:
            mask = np.asarray(mask).astype(np.uint8).reshape(h, w)
        except ValueError:
            mask = np.zeros((h, w), np.uint8)
        embed = np.asarray(embed, dtype=np.float64)
        embed = embed.reshape(-1, EMBED_DIM) if mask.max() > 0 else embed.reshape(-1)
        out = np.zeros(mask.shape, np.uint8)          # labels 0..k in order of value (see module docstring)
        for i, v in enumerate(np.unique(mask)):
            out[mask == v] = i
        return out, embed

    # ---- label bookkeeping ------------------------------------------------------------------------------------
    def match_masks_using_embeddings(self, mask1, mask2, embed1, embed2):
        recs = np.zeros((np.max(mask1), np.max(mask2)))
        match_failed = False
        for i in range(np.max(mask1)):
            max_score = 0
            for j in range(np.max(mask2)):
                if np.count_nonzero(embed2[j]) == 0 or np.count_nonzero(embed1[i]) == 0:
                    score = -1
                else:
                    score = np.dot(embed2[j], embed1[i])
                recs[i, j] = score
                max_score = max(max_score, score)
            if max_score < -0.2:                       # unreachable, as in the original
                match_failed = True
        rows, cols = linear_sum_assignment(-recs)
        return cols + 1, match_failed, rows + 1

    def match_masks_using_sift(self, im1, im2, mask1, mask2):
        sift_failed = False
        im1 = cv2.cvtColor(im1, cv2.COLOR_RGB2GRAY)
        im2 = cv2.cvtColor(im2, cv2.COLOR_RGB2GRAY)
        recs = np.zeros((np.max(mask1), np.max(mask2)))
        sift = cv2.SIFT_create(nfeatures=150, contrastThreshold=0.04, edgeThreshold=40)
        k2, d2 = sift.detectAndCompute(im2, None)
        flann = cv2.FlannBasedMatcher(dict(algorithm=1, trees=5), dict(checks=50))
        for i in range(1, np.max(mask1) + 1):
            k1, d1 = sift.detectAndCompute(im1 * (mask1 == i), None)
            good = [m for m, n in flann.knnMatch(d1, d2, 2) if m.distance < 0.6 * n.distance]
            if len(good) < self.config["min_match_count"]:
                sift_failed = True
            dst = np.round(np.float32([k2[m.trainIdx].pt for m in good]).reshape(-1, 2)).astype(int)
            for j in range(1, np.max(mask2) + 1):
                recs[i - 1, j - 1] = np.sum(mask2[dst[:, 1], dst[:, 0]] == j)
        rows, cols = linear_sum_assignment(-recs)
        return cols + 1, sift_failed, rows + 1

    @staticmethod
    def update_masks(mask2, recs, embeddings):
        mask2_new = np.zeros(mask2.shape, np.uint8)
        embeddings_new = np.empty([np.max(mask2), EMBED_DIM], np.float64)
        for i in range(len(recs)):
            mask2_new[mask2 == recs[i]] = i + 1
            # FIXED 2026-11-17: the original stored embeddings_new[recs[i] - 1] = embeddings[i] (inverse mapping),
            # which swaps identities with 3+ items once labels come back in a cycle.
            embeddings_new[i] = embeddings[recs[i] - 1]
        return mask2_new, embeddings_new

    def refine_masks(self, mask, n, embed):
        """Force exactly n masks: drop or merge extras (smallest first); None, None if fewer than n."""
        embeddings = np.copy(embed)
        if n == 0:
            return np.zeros(mask.shape, np.uint8), np.array([])
        mask = mask.copy()
        if np.max(mask) > n:
            for _ in range(np.max(mask) - n):
                areas = np.array([np.sum(mask == i) for i in range(1, np.max(mask) + 1)])
                small = np.argmin(areas) + 1
                if areas[small - 1] < self.config["min_area"]:
                    target = 0
                else:
                    tl_s, br_s = _extent(mask == small)
                    target, tl_b, br_b = 0, None, None
                    for j in range(1, np.max(mask) + 1):
                        if j == small:
                            continue
                        tl, br = _extent(mask == j)
                        if tl[0] <= tl_s[0] + 10 and br[0] >= br_s[0] - 10 and tl[1] <= tl_s[1] + 10 \
                                and br[1] >= br_s[1] - 10:
                            if target == 0 or (tl[0] >= tl_b[0] and br[0] <= br_b[0] and tl[1] >= tl_b[1]
                                               and br[1] <= br_b[1]):
                                target, tl_b, br_b = j, tl, br
                mask[mask == small] = target
                mask[mask > small] -= 1
                embeddings = np.delete(embeddings, small - 1, 0)
        if np.max(mask) < n:
            log.info("only %d of %d masks", np.max(mask), n)
            return None, None
        return mask, embeddings

    def _match(self, b, old_mask, new_mask, old_emb, new_emb):
        """(recs, sift_failed). recs[k] = new label of old item k + 1."""
        recs, failed, _ = self.match_masks_using_embeddings(old_mask, new_mask, old_emb, new_emb)
        if failed:                                     # unreachable (see module docstring); kept for the record
            recs, sift_failed, _ = self.match_masks_using_sift(b.last["rgb"], b.current["rgb"], old_mask, new_mask)
            return recs, sift_failed
        return recs, False

    # ---- operations -------------------------------------------------------------------------------------------
    def stow(self, bin_id, obj_id, rgb, depth):
        """After item obj_id was put into bin_id. rgb: H x W x 3 (or 4) uint8, BGR(A); depth: H x W, mm."""
        b = self.bins[bin_id]
        if not b.new_object_detected(np.asarray(depth, np.float32)):
            return Status.NO_OBJ_STORED
        self._capture(rgb, depth)
        b.update_current(self.current["rgb"], self.current["depth"])
        if bin_id in self.bad_bins:
            b.n += 1
            self.n += 1
            self.items[obj_id] = [bin_id, b.n]
            return Status.IN_BAD_BINS
        b.n += 1
        self.n += 1
        mask, emb = self.segment(bin_id)
        mask, emb = self.refine_masks(mask, b.n, emb)
        if mask is None:
            self.items[obj_id] = [bin_id, b.n]
            self.bad_bins.append(bin_id)
            return Status.UNDERSEGMENTATION
        if b.last["mask"] is not None:
            recs, sift_failed = self._match(b, b.last["mask"], mask, b.last["embeddings"], emb)
            if sift_failed:
                self.bad_bins.append(bin_id)
            for i in range(1, b.n + 1):                 # the stowed item takes the unmatched label
                if i not in recs:
                    recs = np.append(recs, i)
                    break
            b.current["mask"], b.current["embeddings"] = self.update_masks(mask, recs, emb)
        else:
            if b.n != 1:
                raise RuntimeError(f"bin {bin_id}: no previous masks but {b.n} items")
            b.current["mask"], b.current["embeddings"] = mask.copy(), emb
        self.items[obj_id] = [bin_id, b.n]
        return Status.OK

    def pick(self, obj_id, rgb, depth):
        """After item obj_id was taken out of its bin."""
        if obj_id not in self.items:
            return Status.OBJ_NOT_FOUND
        bin_id, obj_n = self.items[obj_id]
        b = self.bins[bin_id]
        self._capture(rgb, depth)
        b.update_current(self.current["rgb"], self.current["depth"])
        if bin_id in self.bad_bins:
            b.n -= 1
            self.n -= 1
            del self.items[obj_id]
            return Status.IN_BAD_BINS
        mask, emb = self.segment(bin_id)
        b.n -= 1
        self.n -= 1
        mask, emb = self.refine_masks(mask, b.n, emb)
        if mask is None:
            self.bad_bins.append(bin_id)
            del self.items[obj_id]
            return Status.UNDERSEGMENTATION
        try:
            old_mask = b.last["mask"].copy()
            old_emb = b.last["embeddings"].copy()
            old_mask[old_mask == obj_n] = 0
            old_mask[old_mask > obj_n] -= 1
            old_emb = np.delete(old_emb, obj_n - 1, 0)
        except (AttributeError, IndexError, ValueError):   # as in the original: no usable previous state
            old_emb = np.ones((1, EMBED_DIM), np.float64)
            old_mask = np.zeros((256, 256), np.uint8)
        recs, sift_failed = self._match(b, old_mask, mask, old_emb, emb)
        if sift_failed:
            del self.items[obj_id]
            self.bad_bins.append(bin_id)
            return Status.MATCH_FAILED
        b.current["mask"], b.current["embeddings"] = self.update_masks(mask, recs, emb)
        for other, (ob, ol) in self.items.items():
            if ob == bin_id and ol > obj_n:
                self.items[other] = [ob, ol - 1]
        del self.items[obj_id]
        return Status.OK

    def update(self, bin_id, rgb, depth):
        """Re-capture a bin whose contents did not change on purpose (e.g. after a failed grasp)."""
        b = self.bins[bin_id]
        self._capture(rgb, depth)
        b.update_current(self.current["rgb"], self.current["depth"])
        mask, emb = self.segment(bin_id)
        if bin_id in self.bad_bins:
            return Status.IN_BAD_BINS
        mask, emb = self.refine_masks(mask, b.n, emb)
        if mask is None:
            b.abort_update()
            self.current = self.last
            return Status.UNDERSEGMENTATION
        if b.last["mask"] is None:
            raise RuntimeError(f"bin {bin_id}: update without previous masks")
        recs, sift_failed = self._match(b, b.last["mask"], mask, b.last["embeddings"], emb)
        if sift_failed:
            self.bad_bins.append(bin_id)
        b.current["mask"], b.current["embeddings"] = self.update_masks(mask, recs, emb)
        return Status.OK

    # ---- queries ----------------------------------------------------------------------------------------------
    def get_obj_mask(self, obj_id):
        """Full-image uint8 mask (1 = the item), or None for a bad bin (no reliable masks there)."""
        bin_id, label = self.items[obj_id]
        if bin_id in self.bad_bins:
            return None
        b = self.bins[bin_id]
        full = np.zeros((self.H, self.W), np.uint8)
        r1, r2, c1, c2 = b.bounds
        full[r1:r2, c1:c2] = (b.current["mask"] == label).astype(np.uint8)
        return full

    def get_masks(self):
        """All bins' labels in one full image (bin labels offset so they do not collide)."""
        full = np.zeros((self.H, self.W))
        offset = 0
        for b in self.bins.values():
            if b.current["mask"] is not None:
                m = b.current["mask"].copy()
                m[m > 0] += offset
                r1, r2, c1, c2 = b.bounds
                full[r1:r2, c1:c2] = m
                offset += np.max(m)
        return full

    def reset_bin(self, bin_id):
        """Forget a bin (after emptying it by hand): items, masks, history, bad-bin flag."""
        b = self.bins[bin_id]
        b.n = 0
        b.current = {"rgb": None, "depth": None, "mask": None, "embeddings": None}
        b.last = {"rgb": None, "depth": None, "mask": None, "embeddings": None}
        b.history = []
        for other, (ob, _) in list(self.items.items()):
            if ob == bin_id:
                del self.items[other]
        if bin_id in self.bad_bins:
            self.bad_bins.remove(bin_id)
        return Status.OK


def _extent(region):
    """(top-left [x, y], bottom-right [x, y]) of the largest outer contour of a boolean region."""
    cnts = cv2.findContours(region.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    c = max(cnts, key=cv2.contourArea)
    return [c[:, :, 0].min(), c[:, :, 1].min()], [c[:, :, 0].max(), c[:, :, 1].max()]
