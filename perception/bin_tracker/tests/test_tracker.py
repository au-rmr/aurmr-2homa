"""BinTracker bookkeeping on a synthetic pod with a fake segmenter (no GPU, no model). Run: pytest perception/bin_tracker"""
import numpy as np
import pytest

from bin_tracker import BinTracker, Status
from fakes import BOUNDS, FakeSegmenter, Pod, color_at

RECTS = {"red": (5, 40, 3, 20), "green": (5, 40, 25, 45), "blue": (5, 40, 50, 75)}


def make(seed=0):
    pod, seg = Pod(), FakeSegmenter(seed)
    tracker = BinTracker(BOUNDS, seg, pod.capture()[1])
    return pod, seg, tracker


def stow(pod, tracker, bin_id, color):
    pod.place(bin_id, color, RECTS[color])
    return tracker.stow(bin_id, color, *pod.capture())


def test_first_stow_tracks_the_item():
    pod, seg, t = make()
    assert stow(pod, t, "1A", "red") == Status.OK
    assert color_at(t, "red", pod.capture()[0]) == "red"
    assert t.items == {"red": ["1A", 1]}


def test_stow_without_a_depth_change_is_rejected():
    pod, seg, t = make()
    assert t.stow("1A", "ghost", *pod.capture()) == Status.NO_OBJ_STORED
    assert t.items == {} and seg.calls == []


@pytest.mark.parametrize("seed", range(8))
def test_two_items_keep_their_identity(seed):
    pod, seg, t = make(seed)
    for c in ("red", "green"):
        assert stow(pod, t, "1A", c) == Status.OK
    bgr = pod.capture()[0]
    assert [color_at(t, c, bgr) for c in ("red", "green")] == ["red", "green"]


@pytest.mark.parametrize("seed", range(8))
def test_three_items_keep_their_identity(seed):
    pod, seg, t = make(seed)
    for c in ("red", "green", "blue"):
        assert stow(pod, t, "1A", c) == Status.OK
    bgr = pod.capture()[0]
    assert [color_at(t, c, bgr) for c in ("red", "green", "blue")] == ["red", "green", "blue"]


@pytest.mark.parametrize("seed", range(12))
def test_three_items_survive_later_captures(seed):
    """The original's update_masks bug (inverse mapping) lost identities here for about half the seeds (seed 1: the
    labels come back in a 3-cycle)."""
    pod, seg, t = make(seed)
    for c in ("red", "green", "blue"):
        stow(pod, t, "1A", c)
    for _ in range(3):
        assert t.update("1A", *pod.capture()) == Status.OK
    bgr = pod.capture()[0]
    assert [color_at(t, c, bgr) for c in ("red", "green", "blue")] == ["red", "green", "blue"]


def test_history_grows_per_bin_and_reset_clears_it():
    pod, seg, t = make()
    stow(pod, t, "1A", "red")
    stow(pod, t, "1B", "blue")
    stow(pod, t, "1A", "green")
    assert seg.calls == [1, 1, 2]
    t.reset_bin("1A")
    assert len(t.bins["1A"].history) == 0 and "red" not in t.items and t.items["blue"] == ["1B", 1]


@pytest.mark.parametrize("seed", range(4))
def test_pick_keeps_the_rest(seed):
    pod, seg, t = make(seed)
    for c in ("red", "green"):
        stow(pod, t, "1A", c)
    pod.remove("red")
    assert t.pick("red", *pod.capture()) == Status.OK
    assert "red" not in t.items and t.items["green"] == ["1A", 1]
    assert color_at(t, "green", pod.capture()[0]) == "green"


def test_pick_unknown_item():
    pod, seg, t = make()
    assert t.pick("nothing", *pod.capture()) == Status.OBJ_NOT_FOUND


def test_update_after_a_failed_grasp_keeps_identities():
    pod, seg, t = make(3)
    for c in ("red", "green"):
        stow(pod, t, "1A", c)
    assert t.update("1A", *pod.capture()) == Status.OK
    bgr = pod.capture()[0]
    assert [color_at(t, c, bgr) for c in ("red", "green")] == ["red", "green"]


def test_unseen_item_makes_the_bin_bad_until_reset():
    pod, seg, t = make()
    stow(pod, t, "1A", "red")
    pod.place("1A", "green", RECTS["green"])
    bgr, depth = pod.capture()
    bgr[np.all(bgr == (0, 200, 0), axis=-1)] = (40, 200, 200)     # green item invisible to the segmenter
    assert t.stow("1A", "green", bgr, depth) == Status.UNDERSEGMENTATION
    assert "1A" in t.bad_bins and t.get_obj_mask("red") is None
    assert t.stow("1B", "blue", *(pod.place("1B", "blue", RECTS["blue"]) or pod.capture())) == Status.OK
    t.reset_bin("1A")
    assert "1A" not in t.bad_bins and t.items == {"blue": ["1B", 1]}
