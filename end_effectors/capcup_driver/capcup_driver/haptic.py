"""Haptic search, ported from the old ROS 1 cap_haptic_search.py.

Proposes a small change of tool pose from the cap board's 8 electrodes: sideways towards the
loaded side, along the axis to reach a target load, and a tilt to level the contact. Coordinates:
x between channels 1 and 8, y between 2 and 3, z from the cup's base to its tip.

Same numbers as the old code (tests/capcup compares against its recorded output), except that a
proposal with a non-finite value is dropped instead of published: the old code published NaN
when the contact was already level.
"""
import math

import numpy as np

from capcup_driver import processing as proc


def _expm_skew(axis, angle):
    """exp(angle * K) for the old code's skew matrix K, by Rodrigues' formula (was scipy expm)."""
    k = np.array([[0, 0, axis[1]], [0, 0, -axis[0]], [-axis[1], axis[0], 0]], dtype=float)
    # K is the cross-product matrix of w = (axis[0], axis[1], 0), which has unit length.
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * (k @ k)


def quaternion_from_matrix(matrix):
    """(x, y, z, w), the same algorithm and sign as ROS 1 tf.transformations."""
    m = np.array(matrix, dtype=np.float64)[:4, :4]
    q = np.empty((4,))
    t = np.trace(m)
    if t > m[3, 3]:
        q[3] = t
        q[2] = m[1, 0] - m[0, 1]
        q[1] = m[0, 2] - m[2, 0]
        q[0] = m[2, 1] - m[1, 2]
    else:
        i, j, k = 0, 1, 2
        if m[1, 1] > m[0, 0]:
            i, j, k = 1, 2, 0
        if m[2, 2] > m[i, i]:
            i, j, k = 2, 0, 1
        t = m[i, i] - (m[j, j] + m[k, k]) + m[3, 3]
        q[i] = t
        q[j] = m[i, j] + m[j, i]
        q[k] = m[k, i] + m[i, k]
        q[3] = m[k, j] - m[j, k]
    q *= 0.5 / math.sqrt(t * m[3, 3])
    return q


class HapticSearch:
    def __init__(self, num_sensors=8, points=None, thresh_hi=10e4 * 10, thresh_lo=2e4 * 10,
                 lat_step=0.02, lon_step=0.005, alpha=0.7, thresh_alignment=1e4):
        angles = np.linspace(0, 2 * np.pi, num_sensors, endpoint=False) + np.pi / num_sensors
        self._coords = (np.column_stack([np.cos(angles), np.sin(angles)])
                        if points is None else points)
        self._thresh_hi = thresh_hi
        self._thresh_lo = thresh_lo
        self._lat_step = lat_step * (1 - alpha)
        self._lon_step = lon_step
        self._thresh_alignment = thresh_alignment
        self._alpha = alpha
        self._cap = None
        self._var = None
        self._last_was_orientation = False
        self._stack = [proc.Normalize(), proc.NormalizeTime()]
        self._variance = proc.ExpWeightVariance(time_constant=1)

    def update(self, t, values):
        data = proc.apply_process_stack((t, np.array(values, dtype=float)), self._stack)
        self._cap = data[1]
        _, self._var = self._variance(data)

    def _lateral(self):
        if np.sum(self._cap) == 0:
            return np.zeros(2)
        direction = np.sum(self._cap[:, None] * self._coords, axis=0)
        return direction / np.sum(self._cap)

    def _rotation(self):
        p = np.column_stack((self._coords, np.ones(len(self._coords))))
        params = np.linalg.inv(p.T @ p) @ p.T @ self._cap
        plane = np.r_[params[:2], -200000]   # the old calibration of cap counts to mm
        plane = -plane / np.linalg.norm(plane)
        rot = np.cross(plane, np.array([0, 0, 1]))
        angle = np.arcsin(np.linalg.norm(rot))
        with np.errstate(invalid="ignore", divide="ignore"):
            rot = rot / np.linalg.norm(rot)
        return rot, angle

    def _longitudinal(self):
        if np.any(self._cap > self._thresh_hi):
            return -1.0
        if np.sum(self._cap) < self._thresh_lo:
            return 1.0
        return 0.0

    def step(self):
        """One proposal, as the old 50 Hz timer made: (translation xyz, quaternion xyzw) in the
        tool frame, or None. Alternates orientation and translation like the old code."""
        if self._cap is None or np.any(np.isclose(self._cap, np.zeros_like(self._cap), atol=1e-6)):
            return None
        self._alpha = (self._last_was_orientation is False)
        self._last_was_orientation = not self._last_was_orientation
        lateral = self._lateral() * (1 - self._alpha)
        max_translation = 0.01
        if np.linalg.norm(lateral) > max_translation:
            lateral = lateral / np.linalg.norm(lateral) * max_translation
        translation = np.r_[lateral, self._lon_step * self._longitudinal()]
        axis, angle = self._rotation()
        angle = np.min([angle * self._alpha, np.pi / 16])
        transform = np.eye(4)
        transform[:3, :3] = _expm_skew(axis, angle)
        transform[:3, 3] = translation
        q = quaternion_from_matrix(transform)
        if not (np.all(np.isfinite(translation)) and np.all(np.isfinite(q))):
            return None
        return translation, q

    @property
    def last_was_turn(self):
        """Whether the last step() was an orientation step (lateral zeroed, tilt applied), as
        opposed to a translation step. Set even when step() dropped a non-finite proposal."""
        return self._last_was_orientation

    def aligned(self):
        if self._cap is None:
            return None
        return bool(np.all(self._cap > self._thresh_lo / len(self._cap)))
