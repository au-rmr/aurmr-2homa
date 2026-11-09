"""Signal processing for the cap board, unchanged from the old ROS 1 capacitive_cup package
(src/dashboard/processing.py). Its own notes follow.

some things need state (forward derivative, linear filters)
some things just need initial params (normalize, rescale)

This has been nifty for things other than just plotting because it automatically
takes care of state stuff. Might move this somewhere more general because of how
nice it has been to use.
"""
from abc import ABC, abstractmethod
from typing import Tuple, List

import numpy as np


class AbstractProcess(ABC):
    @abstractmethod
    def __call__(self, data: Tuple[float, np.ndarray]) -> np.ndarray:
        """Transform the latest data input"""
        raise NotImplementedError

class Normalize(AbstractProcess):
    def __init__(self, offsets: np.ndarray = None):
        self._offsets = offsets

    def __call__(self, data):
        time, vals = data
        if self._offsets is None:
            self._offsets = vals
        return time, vals - self._offsets

class Scale(AbstractProcess):
    def __init__(self, scales: np.ndarray):
        self._scales = scales

    def __call__(self, data):
        time, vals = data
        return time, vals * self._scales

class NormalizeTime(AbstractProcess):
    def __init__(self, time_offset: float = None):
        self._time_offset = time_offset

    def __call__(self, data):
        time, val = data
        if self._time_offset is None:
            self._time_offset = time
        return time - self._time_offset, val

class ScaleTime(AbstractProcess):
    def __init__(self, time_scale: float = 1e-3):
        self._time_scale = time_scale
    def __call__(self, data):
        time, val = data
        return time * self._time_scale, val

class BackwardsDerivative(AbstractProcess):
    def __init__(self, init_val: np.ndarray = None, init_time: float = -1):
        self._prev_val = init_val
        self._prev_time = init_time

    def __call__(self, data):
        time, vals = data
        if self._prev_val is None:
            self._prev_val = vals
        if time == self._prev_time:
            raise ValueError("Timestamp has not changed")
        deriv = (vals - self._prev_val) / (time - self._prev_time)
        self._prev_time, self._prev_val = data
        return time, deriv

class HighPassFilter(AbstractProcess):
    def __init__(self, time_constant: float = 100):
        self._time_constant = time_constant
        self._filtered_state = None
        self._raw_state = None
        self._time = None

    def __call__(self, data):
        time, vals = data
        if self._raw_state is None:
            self._raw_state = vals
            self._filtered_state = vals
            self._time = time
            return time, vals

        dt = time - self._time
        alpha = self._time_constant / (self._time_constant + dt)
        self._filtered_state = alpha * self._filtered_state + alpha * (vals - self._raw_state)
        self._time = time
        self._raw_state = vals
        return time, self._filtered_state

class LowPassFilter(AbstractProcess):
    def __init__(self, time_constant: float = 100):
        self._time_constant = time_constant
        self._filtered_state = None
        self._time = None

    def __call__(self, data):
        time, vals = data
        if self._filtered_state is None:
            self._filtered_state = vals
            self._time = time
            return time, vals

        dt = time - self._time
        alpha = dt / (self._time_constant + dt)
        self._filtered_state = alpha * vals + (1-alpha) *  self._filtered_state
        self._time = time
        return time, self._filtered_state

class SumChannels(AbstractProcess):
    def __call__(self, data):
        time, vals = data
        return time, np.array([np.sum(vals)])

class ExpWeightVariance(AbstractProcess):
    def __init__(self, time_constant: float = 100):
        self._time_constant = time_constant
        self._moving_average = None
        self._moving_variance = None
        self._time = None

    def __call__(self, data):
        time, vals = data
        if self._time is None:
            self._time = time
            self._moving_average = vals
            self._moving_variance = np.zeros_like(vals)
            return self._time, self._moving_variance

        dt = time - self._time
        alpha = 1 - np.exp(-dt / self._time_constant)
        self._moving_average = (1-alpha) * self._moving_average + alpha * vals
        self._moving_variance = (1-alpha) * self._moving_variance + alpha * (vals - self._moving_average)**2
        self._time = time
        return self._time, self._moving_variance


def apply_process_stack(data, stack: List[AbstractProcess]):
    for process in stack:
        data = process(data)
    return data