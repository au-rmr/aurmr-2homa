"""A synthetic pod and a fake segmenter, for testing the tracker's bookkeeping without a GPU or a model.

Items are flat coloured blocks; the fake segmenter finds them by colour in the LAST crop (like the real one), returns
labels in a shuffled order (the real model's order is arbitrary too) and a unit embedding per colour, so the same item
always has the same embedding."""
import numpy as np

H, W = 120, 200
BOUNDS = {"1A": [10, 60, 10, 90], "1B": [10, 60, 110, 190]}
BG = (40, 200, 200)                     # BGR yellow-ish bin fabric
EMPTY_MM = 1000.0
COLORS = {"red": (0, 0, 220), "green": (0, 200, 0), "blue": (210, 0, 0), "white": (230, 230, 230)}


class Pod:
    """Mutable synthetic scene: place(bin, colour, rect in bin coords, height mm) / remove(colour)."""

    def __init__(self):
        self.items = {}

    def place(self, bin_id, color, rect, height=60):
        self.items[color] = (bin_id, rect, height)

    def remove(self, color):
        del self.items[color]

    def capture(self):
        bgr = np.zeros((H, W, 3), np.uint8)
        depth = np.full((H, W), EMPTY_MM, np.float32)
        for b, (r1, r2, c1, c2) in BOUNDS.items():
            bgr[r1:r2, c1:c2] = BG
        for color, (b, (y1, y2, x1, x2), h) in self.items.items():
            r1, _, c1, _ = BOUNDS[b]
            bgr[r1 + y1:r1 + y2, c1 + x1:c1 + x2] = COLORS[color]
            depth[r1 + y1:r1 + y2, c1 + x1:c1 + x2] = EMPTY_MM - h
        return bgr, depth


def embedding(color):
    e = np.zeros(256)
    e[list(COLORS).index(color) * 7] = 1.0
    return e


class FakeSegmenter:
    def __init__(self, seed=0):
        self.rng = np.random.default_rng(seed)
        self.calls = []

    def __call__(self, frames):
        self.calls.append(len(frames))
        crop = frames[-1]
        found = [c for c, v in COLORS.items() if np.all(crop == v, axis=-1).any()]
        order = self.rng.permutation(len(found))
        labels = np.zeros(crop.shape[:2], np.uint8)
        emb = []
        for k, i in enumerate(order):
            labels[np.all(crop == COLORS[found[i]], axis=-1)] = k + 1
            emb.append(embedding(found[i]))
        return labels, np.array(emb).reshape(-1, 256)


def color_at(tracker, obj_id, bgr):
    """The colour under an item's tracked mask (None if the mask is empty or mixed)."""
    m = tracker.get_obj_mask(obj_id).astype(bool)
    if not m.any():
        return None
    px = {tuple(p) for p in bgr[m]}
    hits = [c for c, v in COLORS.items() if tuple(v) in px]
    return hits[0] if len(hits) == 1 and len(px) == 1 else None
