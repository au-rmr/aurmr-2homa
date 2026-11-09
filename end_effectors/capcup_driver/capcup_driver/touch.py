"""Contact from the cap board, ported from the old ROS 1 touch_detector.py.

Its note: the cup is viscoelastic, so contact can leave a DC offset that makes thresholds
brittle. A high-pass filter with a long time constant decays offsets to zero, which also means
a long contact eventually reads as no contact. The first sample is the zero point, so the cup
must be untouched when the detector starts.
"""
import numpy as np

from capcup_driver import processing as proc


class TouchDetector:
    def __init__(self, time_constant=10.0, threshold=10000.0):
        self.threshold = threshold
        self.contact = False
        self._stack = [proc.NormalizeTime(), proc.Normalize(),
                       proc.HighPassFilter(time_constant), proc.SumChannels()]

    def update(self, t, values):
        """t in seconds. Returns contact after this sample."""
        data = (t, np.array(values, dtype=float))
        for process in self._stack:
            data = process(data)
        self.contact = bool(data[1][0] > self.threshold)
        return self.contact
