"""The new capcup code against what the old ROS 1 code produced (golden_ros1.json, made by
make_golden_ros1.py in the old image) and against the old driver's exact ejector requests."""
import json
import math
import os

import pytest
from capcup_driver import ejector as E
from capcup_driver.fake import FakeCIP
from capcup_driver.haptic import HapticSearch
from capcup_driver.touch import TouchDetector
from sequence import samples

HERE = os.path.dirname(__file__)
GOLDEN = json.load(open(os.path.join(HERE, "golden_ros1.json")))["rows"]


def run_new():
    touch, search, out = TouchDetector(time_constant=10), HapticSearch(), []
    for ms, values in samples():
        t = ms * 1e-3
        contact = touch.update(t, values)
        search.update(t, values)
        out.append((contact, search.step(), search.aligned()))
    return out


NEW = run_new()


def test_contact_matches_ros1():
    assert [c for c, _, _ in NEW] == [g["contact"] for g in GOLDEN]


def test_alignment_matches_ros1():
    assert [a for _, _, a in NEW] == [g["aligned"] for g in GOLDEN]


def test_proposals_match_ros1():
    compared = dropped = 0
    for i, ((_, proposal, _), g) in enumerate(zip(NEW, GOLDEN)):
        old = g["pose"]
        if old is None or any(math.isnan(v) for v in old):
            assert proposal is None, f"sample {i}: old published nothing or NaN, new published"
            dropped += old is not None
            continue
        assert proposal is not None, f"sample {i}: old published a pose, new did not"
        t, q = proposal
        assert list(t) + list(q) == pytest.approx(old, abs=1e-9), f"sample {i}"
        compared += 1
    print(f"{compared} proposals equal, {dropped} NaN proposals dropped")
    assert compared > 500


def test_sequence_covers_the_cases():
    """Guards the test itself: each behaviour must appear, or the comparison proves little."""
    poses = [g["pose"] for g in GOLDEN if g["pose"] and not any(math.isnan(v) for v in g["pose"])]
    assert any(p[2] > 0 for p in poses) and any(p[2] < 0 for p in poses) and any(p[2] == 0 for p in poses)
    assert any(abs(p[0]) > 1e-4 for p in poses), "no sideways proposal"
    assert any(abs(p[6]) < 0.9999 for p in poses), "no tilt proposal"
    assert any(g["contact"] for g in GOLDEN) and not all(g["contact"] for g in GOLDEN)
    assert any(g["aligned"] for g in GOLDEN) and not all(g["aligned"] for g in GOLDEN)


# Byte patterns copied from the old VacuumGripperSocketNode.py.
OLD_BYTES = {
    E.OFF: bytearray([0b00000000] + [0b00000000] + [0b00000000] + [0b00000000] + [0b00000000] * 12),
    E.SUCK: bytearray([0b00000000] + [0b00000000] + [0b00000001] + [0b00000001] + [0b00000000] * 12),
    E.BLOW: bytearray([0b00000000] + [0b00000000] + [0b00000010] + [0b00000010] + [0b00000000] * 12),
}


@pytest.mark.parametrize("mode", [E.OFF, E.SUCK, E.BLOW])
def test_ejector_sends_the_old_requests(mode):
    from pycomm3 import Services
    cip = FakeCIP()
    E.Ejector("x", driver_factory=lambda _: cip).set_mode(mode)
    assert cip.requests == [(bytes(Services.set_attribute_single), 0xA2, 13, 5,
                             bytes(OLD_BYTES[mode]))]


def test_ejector_reads_vacuum_as_the_old_driver():
    from pycomm3 import Services
    cip = FakeCIP(rise_s=0.0)
    e = E.Ejector("x", driver_factory=lambda _: cip)
    e.set_mode(E.SUCK)
    assert e.system_vacuum_mbar() == 820
    service, cls, instance, attribute, _ = cip.requests[-1]
    assert (service, cls, instance, attribute) == (
        bytes(Services.get_attribute_single), 0xA2, 515, 5)


def test_ejector_bad_reading_raises_instead_of_zero():
    """The old driver returned 0 on an empty reply, which reads as 'no item'."""
    class Empty:
        def generic_message(self, **_):
            from types import SimpleNamespace
            return SimpleNamespace(value=b"", error=None)

        def close(self):
            pass

    with pytest.raises(ConnectionError):
        E.Ejector("x", driver_factory=lambda _: Empty()).system_vacuum_mbar()


def test_ejector_reconnects_once_then_raises():
    calls = []

    def factory(_):
        calls.append(1)
        raise ConnectionError("no answer")

    with pytest.raises(ConnectionError):
        E.Ejector("x", driver_factory=factory).system_vacuum_mbar()
    assert len(calls) == 2


@pytest.mark.parametrize("tilt_deg, axis", [(8, "x"), (-8, "x"), (8, "y"), (-8, "y")])
def test_turn_proposal_levels_the_cup(tilt_deg, axis):
    """The search's turn, applied as Robot applies a tool-frame rotation (current * q), must
    reduce the angle between the cup axis and a tilted surface. Electrodes on a 2 cm rim read
    10000 counts per mm pressed, the scale the search's plane fit assumes."""
    import numpy as np
    from capcup_driver.haptic import HapticSearch
    t = math.radians(tilt_deg)
    normal = np.array([0.0, -math.sin(t), math.cos(t)]) if axis == "x" else \
        np.array([math.sin(t), 0.0, math.cos(t)])
    angles = np.linspace(0, 2 * np.pi, 8, endpoint=False) + np.pi / 8
    rim = np.stack([0.02 * np.cos(angles), 0.02 * np.sin(angles), np.zeros(8)], axis=1)
    depth = 0.004 + rim @ normal   # 4 mm in at the centre
    search = HapticSearch()
    search.update(0.0, [3_000_000] * 8)
    search.update(0.02, [int(3_000_000 + 1e7 * max(0.0, d)) for d in depth])
    for _ in range(2):   # one of two consecutive proposals is the turn
        _, q = search.step()
        if abs(q[3]) < 0.99999:
            break
    x, y, z, w = q
    turned_axis = np.array([2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)])
    before = math.degrees(math.acos(normal[2]))
    after = math.degrees(math.acos(min(1.0, abs(turned_axis @ normal))))
    print(f"tilt {tilt_deg} about {axis}: {before:.1f} -> {after:.1f} deg")
    assert after < 0.5 * before
