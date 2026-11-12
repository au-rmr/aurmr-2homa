"""capcup_driver.calibration: the fit recovers known gains and curve from synthetic presses, a
tilt cancels between runs rolled 180 deg, and the estimator inverts the curve."""
import csv

import numpy as np
import pytest

from capcup_driver import calibration as C

GAINS = np.array([0.9, 0.6, 0.5, 0.65, 1.0, 1.4, 1.55, 1.4])
GAINS = GAINS / GAINS.mean()


def curve(d_mm):
    return 16000 * d_mm + 2000 * np.maximum(d_mm - 4, 0) ** 2


def write_run(path, tilt=0.0, seed=0):
    """Tip from 30 mm above contact to 6 mm in at 0.2 mm per sample, in-air values ~9.7e6.
    tilt adds a cos pattern over the channels (a surface not square to the cup)."""
    rng = np.random.default_rng(seed)
    air = 9.7e6 + rng.normal(0, 3e4, 8)
    angles = np.radians(22.5 + 45 * np.arange(8))
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "tcp_z", "df_n", "dfz_n", "contact"] + C.CHANNELS + ["qx", "qy", "qz", "qw", "phase"])
        for i, d_mm in enumerate(np.arange(-30, 6.01, 0.2)):
            d = max(d_mm, 0.0)
            load = curve(d) * GAINS * (1 + tilt * np.cos(angles))
            vals = air + load + rng.normal(0, 600, 8)
            force = 2.5 * d if d_mm > 0 else rng.normal(0, 0.3)
            # cup straight down: q = 180 deg about x
            w.writerow([i * 0.17, 1.0 - d_mm / 1000, force, -force, d > 0, *vals, 1, 0, 0, 0, "in"])


def test_fit_recovers_gains_and_curve(tmp_path):
    p = tmp_path / "a.csv"
    write_run(p)
    cal = C.fit([str(p)])
    assert np.allclose(cal["gains"], GAINS, atol=0.02)
    est = C.Estimator(cal)
    # contact = the sum 20k over its low: ~0.2 mm in (16k/mm x 8 channels), so readings run ~0.2 mm short
    for d in (2.0, 4.0):
        got = est.deflection_mm(curve(d) * GAINS)
        assert np.allclose(got, d - 0.2, atol=0.25), (d, got)
    assert cal["residual_pct"] < 5


def test_tilt_cancels_between_rolled_runs(tmp_path):
    a, b = tmp_path / "a.csv", tmp_path / "b.csv"
    write_run(a, tilt=0.3, seed=1)
    write_run(b, tilt=-0.3, seed=2)   # rolled 180 deg: the tilt pattern changes sign per channel
    cal = C.fit([str(a), str(b)])
    one = C.fit([str(a)])
    assert np.abs(np.array(cal["gains"]) - GAINS).max() < 0.03
    assert np.abs(np.array(one["gains"]) - GAINS).max() > 0.1   # a single tilted run is biased


def test_no_contact_is_refused(tmp_path):
    p = tmp_path / "a.csv"
    write_run(p)
    rows = list(csv.reader(open(p)))
    with open(p, "w", newline="") as f:   # never touches: channels stay at their in-air values
        w = csv.writer(f)
        w.writerow(rows[0])
        for r in rows[1:]:
            w.writerow(r[:5] + rows[1][5:13] + r[13:])
    with pytest.raises(ValueError, match="no contact"):
        C.fit([str(p)])


def test_estimator_extends_past_both_ends(tmp_path):
    """Not clamped: below contact reads negative (the lifted side of a tilted touch), above the
    calibrated range keeps rising (user 2026-11-12)."""
    est = C.Estimator({"gains": [1.0] * 8,
                       "curve": {"deflection_mm": [0.0, 1.0, 2.0, 3.0],
                                 "counts": [0, 10000, 20000, 40000]}})
    got = est.deflection_mm([-5000, 0, 15000, 50000, 0, 0, 0, 0])
    assert np.allclose(got[:4], [-0.5, 0.0, 1.5, 3.5])
    assert list(est.in_range([-5000, 0, 15000, 50000, 0, 0, 0, 0])[:4]) == [False, True, True, False]

