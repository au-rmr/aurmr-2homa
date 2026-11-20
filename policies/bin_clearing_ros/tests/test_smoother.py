"""Smoother = the old mppi_bridge exponential smoothing, per command. No ROS or Isaac needed:

    PYTHONPATH=policies/bin_clearing_ros python3 -m pytest policies/bin_clearing_ros/tests/test_smoother.py -q
"""
import math

import pytest

from bin_clearing_ros.planner_node import Smoother


def run(s, u, times):
    return [s(u, t)[0] for t in times]


def test_first_command_is_one_alpha_step():
    s = Smoother(1, 0.2, 20.0, 0.15)
    assert s([0.1], 0.0)[0] == pytest.approx(0.02)


def test_time_constant_independent_of_rate():
    # Bridge: alpha 0.2 per 1/20 s -> after 0.5 s of a constant 0.1 command the output is the same at any rate.
    expect = 0.1 * (1 - 0.8 ** (0.5 * 20 + 1))   # +1: the first command is one 1/20 s step
    for hz in (10.0, 20.0, 50.0, 100.0):           # gaps under the 0.15 s reset (5 Hz would restart every time)
        s = Smoother(1, 0.2, 20.0, 0.15)
        times = [i / hz for i in range(int(0.5 * hz) + 1)]
        assert run(s, [0.1], times)[-1] == pytest.approx(expect, rel=1e-9), hz
    assert -0.05 / math.log(0.8) == pytest.approx(0.224, abs=1e-3)   # ~0.22 s time constant


def test_stays_within_clamp():
    s = Smoother(2, 0.2, 20.0, 0.15)
    for i in range(200):
        y = s([0.1 if i % 3 else -0.1, 0.1], i * 0.04)
        assert all(abs(v) <= 0.1 + 1e-12 for v in y)


def test_gap_over_stale_limit_restarts_from_zero():
    s = Smoother(1, 0.2, 20.0, 0.15)
    for i in range(50):
        s([0.1], i * 0.04)                      # converged near 0.1
    assert s([0.1], 49 * 0.04 + 0.2)[0] == pytest.approx(0.02)   # 0.2 s gap: arm has ramped to zero


def test_alpha_one_is_off():
    s = Smoother(1, 1.0, 20.0, 0.15)
    assert run(s, [0.07], [0.0, 0.04]) == [0.07, 0.07]


@pytest.mark.parametrize("alpha", [0.0, -0.1, 1.5])
def test_bad_alpha_refused(alpha):
    with pytest.raises(ValueError):
        Smoother(1, alpha, 20.0, 0.15)
