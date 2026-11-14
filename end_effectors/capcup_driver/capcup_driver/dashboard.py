"""CapCup dashboard, ported from the old ROS 1 capacitive_cup dashboard (cap_dashboard_node.py):
the 8 channels over time, all channels on one plot with the search's thresholds, their sum, and
a ring showing each channel round the cup. Values are changes from each channel's in-air value,
taken from the first samples and again with "Re-zero" (the old "Reset"). With a calibration
(capcup_calibration.yaml) a switch shows them gain-corrected or as deflection in mm; the
deflection is not clamped (below contact reads negative), dotted lines mark 0 mm and the top of
the calibrated range.

Force (2026-11-13): the arm's wrist force (/force_torque_sensor_broadcaster/wrench, the copy
motion_server publishes on the shared domain), as the change from its value at the last re-zero:
x, y, z and the size of the change, with the cap_cup touch stop (30 N) dashed. motion_server
measures its stops from each move's start instead, so this shows drift that a stop would not see
until the next move. Thinned to 50 Hz.
"""
import collections
import math
import time

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets   # before pyqtgraph: it uses the binding already loaded
import pyqtgraph as pg  # noqa: E402  (the image also has PyQt6, which pyqtgraph would pick first)

from capcup_driver import calibration as C

WINDOW_S = 30.0
ZERO_SAMPLES = 10
# The haptic search's thresholds (haptic.py defaults): per channel for alignment, on the sum
# for "too little" and on any channel for "too much".
THRESH_LO, THRESH_HI = 2e4 * 10, 10e4 * 10
MODES = ("raw counts", "gain-corrected counts", "deflection (mm)")
FORCE_STOP_N = 30.0     # cap_cup touch_force_n (sites/uw_lab/demos/cap_cup.yaml)
FORCE_EVERY_S = 0.02    # keep one wrench sample per 20 ms (it comes at 500 Hz)


class Ring(QtWidgets.QWidget):
    """Channel k drawn at 22.5 + 45 (k - 1) deg from the tool's x axis, counting towards y, seen
    from the cup's tip (the old radial plot; "Flip" mirrors it to the view from the flange)."""

    def __init__(self):
        super().__init__()
        self.values, self.scale, self.flip = np.zeros(8), 1.0, False
        self.setMinimumSize(260, 260)

    def set(self, values, scale):
        self.values, self.scale = np.asarray(values, dtype=float), scale
        self.update()

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        side = min(self.width(), self.height())
        r = side * 0.32
        cx, cy = self.width() / 2, self.height() / 2
        rect = QtCore.QRectF(cx - r, cy - r, 2 * r, 2 * r)
        sign = -1 if self.flip else 1
        for k in range(8):
            v = float(np.clip(self.values[k] / self.scale, -1, 1)) if self.scale > 0 else 0.0
            if v >= 0:
                col = QtGui.QColor(int(230 - 10 * v), int(230 - 170 * v), int(230 - 170 * v))
            else:
                col = QtGui.QColor(int(230 + 170 * v), int(230 + 110 * v), 230 + int(25 * -v))
            p.setPen(QtGui.QPen(col, side * 0.08))
            centre = 22.5 + 45 * k                      # deg from tool x, towards y
            p.drawArc(rect, int(16 * sign * (centre - 18)), int(16 * sign * 36))
            a = math.radians(sign * centre)
            p.setPen(QtGui.QPen(QtGui.QColor(60, 60, 60)))
            p.drawText(QtCore.QPointF(cx + 1.32 * r * math.cos(a) - 14, cy - 1.32 * r * math.sin(a) + 5),
                       f"ch{k + 1}")
        p.setPen(QtGui.QPen(QtGui.QColor(120, 120, 120), 1, QtCore.Qt.DashLine))
        p.drawLine(QtCore.QPointF(cx, cy), QtCore.QPointF(cx + r * 0.8, cy))
        p.drawText(QtCore.QPointF(cx + r * 0.82, cy - 4), "x")
        a = math.radians(sign * 90)
        p.drawLine(QtCore.QPointF(cx, cy), QtCore.QPointF(cx + r * 0.8 * math.cos(a), cy - r * 0.8 * math.sin(a)))
        p.drawText(QtCore.QPointF(cx + r * 0.85 * math.cos(a) + 4, cy - r * 0.85 * math.sin(a)), "y")
        p.drawText(QtCore.QPointF(6, 16), "seen from the flange" if self.flip else "seen from the tip")


class Dashboard(QtWidgets.QWidget):
    def __init__(self, cal=None, cal_path=""):
        super().__init__()
        self.setWindowTitle("CapCup dashboard")
        self.est = C.Estimator(cal) if cal else None
        self.t, self.y = collections.deque(), collections.deque()
        self.zero, self.zero_buf, self.t0 = None, [], time.monotonic()
        self.contact, self.last_rx = None, None
        self.ft, self.f = collections.deque(), collections.deque()
        self.f_zero, self.f_last = None, 0.0

        self.mode = QtWidgets.QComboBox()
        self.mode.addItems(MODES if self.est else MODES[:1])
        rezero = QtWidgets.QPushButton("Re-zero (cup in air)")
        rezero.clicked.connect(self.start_zero)
        flip = QtWidgets.QPushButton("Flip ring")
        self.ring = Ring()
        flip.clicked.connect(lambda: (setattr(self.ring, "flip", not self.ring.flip), self.ring.update()))
        self.status = QtWidgets.QLabel()
        top = QtWidgets.QHBoxLayout()
        top.addWidget(QtWidgets.QLabel("Show:"))
        top.addWidget(self.mode)
        top.addWidget(rezero)
        top.addWidget(flip)
        top.addStretch()
        top.addWidget(self.status)
        if cal_path:
            top.addWidget(QtWidgets.QLabel(f"calibration: {cal_path}"))

        pg.setConfigOptions(antialias=True, background="w", foreground="k")
        stacked = pg.GraphicsLayoutWidget()
        self.ch_plots, self.ch_curves = [], []
        for k in range(8):
            pl = stacked.addPlot(row=k, col=0)
            pl.showGrid(x=True, y=True)
            pl.setLabel("left", f"ch{k + 1}")
            if k:
                pl.setXLink(self.ch_plots[0])
            self.ch_plots.append(pl)
            self.ch_curves.append(pl.plot(pen=pg.mkPen(pg.intColor(k, hues=8), width=2)))
        self.ch_plots[-1].setLabel("bottom", "time", units="s")
        combined = pg.PlotWidget(title="All channels")
        combined.showGrid(x=True, y=True)
        combined.addLegend(offset=(5, 5))
        self.all_curves = [combined.plot(pen=pg.mkPen(pg.intColor(k, hues=8), width=2), name=f"ch{k + 1}")
                           for k in range(8)]
        self.thresh_lines = [pg.InfiniteLine(v, angle=0, pen=pg.mkPen("r", style=QtCore.Qt.DashLine))
                             for v in (THRESH_LO / 8, THRESH_HI)]
        for line in self.thresh_lines:
            combined.addItem(line)
        total = pg.PlotWidget(title="Sum of channels")
        total.showGrid(x=True, y=True)
        self.sum_curve = total.plot(pen=pg.mkPen("k", width=2))
        self.sum_line = pg.InfiniteLine(THRESH_LO, angle=0, pen=pg.mkPen("r", style=QtCore.Qt.DashLine))
        total.addItem(self.sum_line)
        # Deflection view: 0 mm (contact) and the top of the calibrated range; outside them the
        # values are a straight extension of the curve, not measured.
        self.range_lines = []
        if self.est:
            for plot in [combined] + self.ch_plots:
                for v in (0.0, float(self.est.d[-1])):
                    line = pg.InfiniteLine(v, angle=0, pen=pg.mkPen((120, 120, 120), style=QtCore.Qt.DotLine))
                    plot.addItem(line)
                    self.range_lines.append(line)
        self.combined, self.total = combined, total
        force = pg.PlotWidget(title="Wrist force, change since re-zero (N)")
        force.showGrid(x=True, y=True)
        force.addLegend(offset=(5, 5))
        self.force_curves = [force.plot(pen=pg.mkPen(c, width=w), name=n) for c, w, n in
                             (("r", 1, "x"), ("g", 1, "y"), ("b", 1, "z"), ("k", 2, "|change|"))]
        force.addItem(pg.InfiniteLine(FORCE_STOP_N, angle=0, pen=pg.mkPen("r", style=QtCore.Qt.DashLine)))
        force.setXLink(self.ch_plots[0])
        self.force = force

        grid = QtWidgets.QGridLayout()
        grid.addWidget(stacked, 0, 0, 2, 1)
        grid.addWidget(combined, 0, 1)
        grid.addWidget(total, 0, 2)
        grid.addWidget(self.ring, 1, 1)
        grid.addWidget(self.force, 1, 2)
        grid.setColumnStretch(0, 2)
        grid.setColumnStretch(1, 2)
        grid.setColumnStretch(2, 2)
        outer = QtWidgets.QVBoxLayout(self)
        outer.addLayout(top)
        outer.addLayout(grid)
        self.mode.currentIndexChanged.connect(self.redraw)
        self.resize(1400, 850)

    def start_zero(self):
        self.zero, self.zero_buf = None, []
        self.t.clear()
        self.y.clear()
        self.f_zero = None
        self.ft.clear()
        self.f.clear()

    def on_wrench(self, xyz):
        now = time.monotonic()
        if now - self.f_last < FORCE_EVERY_S:
            return
        self.f_last = now
        v = np.asarray(xyz, dtype=float)
        if not np.all(np.isfinite(v)):
            return
        if self.f_zero is None:
            self.f_zero = v
        self.ft.append(now - self.t0)
        self.f.append(v - self.f_zero)
        while self.ft and self.ft[-1] - self.ft[0] > WINDOW_S:
            self.ft.popleft()
            self.f.popleft()

    def on_cap(self, values):
        v = np.asarray(values, dtype=float)
        self.last_rx = time.monotonic()
        if self.zero is None:
            self.zero_buf.append(v)
            if len(self.zero_buf) >= ZERO_SAMPLES:
                self.zero = np.mean(self.zero_buf, axis=0)
            return
        self.t.append(self.last_rx - self.t0)
        self.y.append(v - self.zero)
        while self.t and self.t[-1] - self.t[0] > WINDOW_S:
            self.t.popleft()
            self.y.popleft()

    def shown(self, y):
        m = self.mode.currentIndex()
        if m == 1:
            return self.est.normalised(y)
        if m == 2:
            return np.array([self.est.deflection_mm(row) for row in np.atleast_2d(y)]).reshape(np.shape(y))
        return y

    def redraw(self, *_):
        age = None if self.last_rx is None else time.monotonic() - self.last_rx
        zeroing = "" if self.zero is not None else f"  zeroing {len(self.zero_buf)}/{ZERO_SAMPLES}"
        self.status.setText(("no data yet" if age is None else
                             f"last sample {age:.1f} s ago" + ("  STALE" if age > 1.0 else ""))
                            + f"  contact: {self.contact}" + zeroing)
        if self.f:
            ft, f = np.asarray(self.ft), np.asarray(self.f)
            for k in range(3):
                self.force_curves[k].setData(ft, f[:, k])
            self.force_curves[3].setData(ft, np.linalg.norm(f, axis=1))
        if not self.y:
            return
        t = np.asarray(self.t)
        y = self.shown(np.asarray(self.y))
        mm = self.mode.currentIndex() == 2
        for k in range(8):
            self.ch_curves[k].setData(t, y[:, k])
            self.all_curves[k].setData(t, y[:, k])
        lo, hi = float(y.min()), float(y.max())
        pad = 0.05 * max(hi - lo, 1e-9)
        for pl in self.ch_plots:
            pl.setYRange(lo - pad, hi + pad, padding=0)
        self.sum_curve.setData(t, y.sum(axis=1) if not mm else y.mean(axis=1))
        self.total.setTitle("Mean deflection (mm)" if mm else "Sum of channels")
        for line in self.thresh_lines + [self.sum_line]:
            line.setVisible(self.mode.currentIndex() == 0)
        for line in self.range_lines:
            line.setVisible(mm)
        scale = (self.est.d[-1] if mm else max(abs(lo), abs(hi), 1.0))
        self.ring.set(y[-1], scale)


def main():
    import argparse
    import os
    import sys

    import rclpy
    from capcup_interfaces.msg import CapData
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import WrenchStamped
    from std_msgs.msg import Bool

    ap = argparse.ArgumentParser()
    ap.add_argument("--calibration", default=os.environ.get("CAPCUP_CALIBRATION", ""),
                    help="capcup_calibration.yaml (optional)")
    args, ros_args = ap.parse_known_args()
    cal = C.load(args.calibration) if args.calibration and os.path.exists(args.calibration) else None
    rclpy.init(args=ros_args)
    node = rclpy.create_node("cap_dashboard")
    app = QtWidgets.QApplication(sys.argv[:1])
    win = Dashboard(cal, args.calibration if cal else "")
    node.create_subscription(CapData, "/cap_data", lambda m: win.on_cap(m.values), qos_profile_sensor_data)
    node.create_subscription(Bool, "/cap_contact", lambda m: setattr(win, "contact", m.data),
                             qos_profile_sensor_data)
    node.create_subscription(
        WrenchStamped, "/force_torque_sensor_broadcaster/wrench",
        lambda m: win.on_wrench((m.wrench.force.x, m.wrench.force.y, m.wrench.force.z)),
        qos_profile_sensor_data)
    spin = QtCore.QTimer()
    spin.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    spin.start(10)
    draw = QtCore.QTimer()
    draw.timeout.connect(win.redraw)
    draw.start(100)
    win.show()
    rc = app.exec_()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
