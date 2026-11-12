"""CapCup channel calibration from presses on a flat surface (S6, 2026-11-12).

Model, deliberately simple: pressed square onto a flat surface, channel k's change from its own
in-air value is gain_k * curve(deflection). One curve shared by all channels, one gain per
channel (mean gain = 1). The cup has hysteresis and is not linear; this is a first map from
deflection to counts and back, fitted on the loading side only. The in-air values drift, so
they are taken at run time and never stored.

Deflection is how far the tip has travelled along the cup since first contact. First contact is
where the channel sum first jumps CONTACT_COUNTS above its lowest value so far (the cup senses
the surface a few mm out and sags slightly, then jumps on touch). Not the force: it creeps 1-2 N
while the arm moves through air (run A, 2026-11-12). Not the model's tip: press 1 and run A both
touched ~3 mm before the model said (the suction cup is viscoelastic; its length varies).
"""
import csv

import numpy as np
import yaml

CHANNELS = [f"ch{i}" for i in range(1, 9)]
CONTACT_COUNTS = 20000   # sum jump that counts as touching; the sum's noise is ~+-3k
AIR_FROM_CONTACT = 0.015   # in-air samples: at least this far (m) before contact (the cup
                           # senses the surface ~5 mm out, so the values sag before touching)
STEP_MM = 0.5


def roll_deg(path):
    """The tool x axis' angle from base x seen from above (deg, median over the run), or None for
    a CSV without orientation."""
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows or "qw" not in rows[0]:
        return None
    angles = []
    for r in rows:
        x, y, z, w = (float(r[k]) for k in ("qx", "qy", "qz", "qw"))
        angles.append(np.degrees(np.arctan2(2 * (x * y + z * w), 1 - 2 * (y * y + z * z))))
    return float(np.median(angles))


def read_run(path):
    """Rows of a cap_press CSV as arrays: tip z, force change, channels (n x 8), cup axis z, phase."""
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"{path}: no samples")
    z = np.array([float(r["tcp_z"]) for r in rows])
    df = np.array([float(r["df_n"]) for r in rows])
    ch = np.array([[float(r[c]) for c in CHANNELS] for r in rows])
    if "qw" in rows[0]:
        axis_z = []
        for r in rows:
            x, y = float(r["qx"]), float(r["qy"])
            axis_z.append(1 - 2 * (x * x + y * y))   # z component of the tool z axis
        axis_z = -np.array(axis_z)                   # the cup points down: make it positive
    else:
        axis_z = np.ones(len(rows))   # press 1 had no orientation: cup straight down
    phase = np.array([r.get("phase", "in") for r in rows])
    return z, df, ch, axis_z, phase


def deflection(path):
    """Per sample: deflection along the cup (m, + = pressed in), change from in-air values
    (n x 8), phase. Raises if the run never touched or has no in-air samples."""
    z, df, ch, axis_z, phase = read_run(path)
    loading = np.flatnonzero(phase == "in")
    total = ch.sum(axis=1)
    low, hits = np.inf, []
    for a, b in zip(loading[:-1], loading[1:]):
        low = min(low, total[a])
        if total[a] - low >= CONTACT_COUNTS and total[b] - low >= CONTACT_COUNTS:
            hits.append(a)
            break
    if not hits:
        raise ValueError(f"{path}: the channel sum never jumped {CONTACT_COUNTS} above its low (no contact)")
    i0 = hits[0]
    along = z / np.maximum(axis_z, 0.5)          # position along the cup axis (cos <= 60 deg)
    d = along[i0] - along
    air = [i for i in loading if i < i0 and d[i] <= -AIR_FROM_CONTACT]
    if len(air) < 5:
        raise ValueError(f"{path}: fewer than 5 samples at least {AIR_FROM_CONTACT * 1000:.0f} mm "
                         "before contact; start the press further out")
    return d, ch - ch[air].mean(axis=0), phase


def fit(paths):
    """Gains and curve from one or more runs. Gains are fitted per run, averaged within each roll
    (runs within 10 deg), then across rolls, so a tilt fixed in the room cancels between rolls 180
    deg apart however many repeats each roll has. The per-run gains are returned too."""
    per_run, d_all, y_all = [], [], []
    for p in paths:
        d, y, phase = deflection(p)
        m = (phase == "in") & (d >= 0)
        if m.sum() < 10:
            raise ValueError(f"{p}: fewer than 10 loading samples after contact")
        # Rank-1 fit: y ~ curve(d) * gain, by the first singular vectors.
        u, s, vt = np.linalg.svd(y[m], full_matrices=False)
        g = vt[0] * np.sign(vt[0].sum())
        per_run.append(g / g.mean())
        d_all.append(d[m])
        y_all.append(y[m])
    groups = {}
    for p, g in zip(paths, per_run):
        r = roll_deg(p)
        key = None if r is None else int(round(r / 10.0)) % 36
        groups.setdefault(key, []).append(g)
    gains = np.mean([np.mean(gs, axis=0) for gs in groups.values()], axis=0)
    gains /= gains.mean()
    d, y = np.concatenate(d_all), np.concatenate(y_all)
    level = y @ gains / (gains @ gains)           # each sample's curve value, gain removed
    resid = y - np.outer(level, gains)
    edges = np.arange(0, d.max() * 1000 + STEP_MM, STEP_MM)
    curve_d, curve_c = [], []
    for lo in edges:
        sel = (d * 1000 >= lo) & (d * 1000 < lo + STEP_MM)
        if sel.sum() >= 2:
            curve_d.append(round(float(np.median(d[sel] * 1000)), 2))
            curve_c.append(round(float(np.median(level[sel]))))
    return {
        "gains": [round(float(v), 4) for v in gains],
        "per_run_gains": [[round(float(v), 4) for v in g] for g in per_run],
        "curve": {"deflection_mm": curve_d, "counts": curve_c},
        "residual_pct": round(float(100 * np.linalg.norm(resid) / np.linalg.norm(y)), 1),
        "samples": int(len(d)),
    }


def hysteresis(path, cal):
    """Unloading vs loading at the same deflection: (deflection_mm, counts in, counts out)."""
    d, y, phase = deflection(path)
    g = np.asarray(cal["gains"])
    level = y @ g / (g @ g)
    out = []
    for lo in np.arange(0, d.max() * 1000, 1.0):
        a = (phase == "in") & (d * 1000 >= lo) & (d * 1000 < lo + 1)
        b = (phase == "out") & (d * 1000 >= lo) & (d * 1000 < lo + 1)
        if a.sum() and b.sum():
            out.append((lo + 0.5, float(np.median(level[a])), float(np.median(level[b]))))
    return out


def load(path):
    with open(path) as f:
        cal = yaml.safe_load(f)
    if len(cal["gains"]) != 8 or min(cal["gains"]) <= 0:
        raise ValueError(f"{path}: needs 8 positive gains")
    c = cal["curve"]
    if len(c["deflection_mm"]) < 2 or np.any(np.diff(c["counts"]) <= 0):
        raise ValueError(f"{path}: the curve must rise with deflection")
    return cal


class Estimator:
    """Per-channel deflection (mm) from changes against in-air values, by the inverse curve."""

    def __init__(self, cal):
        self.gains = np.asarray(cal["gains"], dtype=float)
        self.d = np.asarray(cal["curve"]["deflection_mm"], dtype=float)
        self.c = np.asarray(cal["curve"]["counts"], dtype=float)

    def normalised(self, change):
        return np.asarray(change, dtype=float) / self.gains

    def deflection_mm(self, change):
        """Through the curve, and past its ends along the slope of the end segments: below
        contact (e.g. the lifted side of a tilted touch, or the sag as the cup nears a surface)
        comes out negative, a harder press keeps rising. Not clamped (user 2026-11-12): a
        negative side is real information. Outside the calibrated range it is an extension, not
        a measurement; in_range() says which."""
        n = np.asarray(self.normalised(change), dtype=float)
        out = np.interp(n, self.c, self.d)
        lo_slope = (self.d[1] - self.d[0]) / (self.c[1] - self.c[0])
        hi_slope = (self.d[-1] - self.d[-2]) / (self.c[-1] - self.c[-2])
        out = np.where(n < self.c[0], self.d[0] + (n - self.c[0]) * lo_slope, out)
        return np.where(n > self.c[-1], self.d[-1] + (n - self.c[-1]) * hi_slope, out)

    def in_range(self, change):
        n = np.asarray(self.normalised(change), dtype=float)
        return (n >= self.c[0]) & (n <= self.c[-1])


def dump(cal, meta):
    """YAML text for sites/<site>/tools/capcup_calibration.yaml."""
    head = "".join(f"# {line}\n" for line in meta)
    body = yaml.safe_dump({k: cal[k] for k in ("gains", "curve", "residual_pct", "samples",
                                               "per_run_gains")},
                          default_flow_style=None, sort_keys=False, width=100)
    return head + body
