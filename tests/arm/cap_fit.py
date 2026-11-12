"""Fits the CapCup channel calibration from cap_press CSVs (capcup_driver/calibration.py).

    python3 tests/arm/cap_fit.py run1.csv [run2.csv ...] [--out sites/uw_lab/tools/capcup_calibration.yaml]

Prints per-run gains (they should agree between repeats; a tilt shows as a difference between
runs rolled 180 deg apart, which the average cancels), the shared curve, the rank-1 residual and,
for runs pressed with --back, the hysteresis. Writes the YAML only with --out.
"""
import argparse
import os
import time

from capcup_driver import calibration as C


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--out")
    a = ap.parse_args()
    cal = C.fit(a.runs)
    for path, g in zip(a.runs, cal["per_run_gains"]):
        print(f"gains {os.path.basename(path)}: {g}")
    print(f"gains (mean): {cal['gains']}")
    print(f"rank-1 residual {cal['residual_pct']} % of the signal, {cal['samples']} samples")
    print("curve (deflection mm -> counts at mean gain):")
    for d, c in zip(cal["curve"]["deflection_mm"], cal["curve"]["counts"]):
        print(f"  {d:5.2f} mm  {c:9d}")
    for path in a.runs:
        h = C.hysteresis(path, cal)
        if h:
            print(f"hysteresis {os.path.basename(path)} (mm: in -> out counts):")
            for d, cin, cout in h:
                print(f"  {d:4.1f} mm  {cin:9.0f} -> {cout:9.0f}  ({100 * (cout - cin) / max(cin, 1):+.0f} %)")
    if a.out:
        meta = [f"CapCup channel calibration, {time.strftime('%Y-%m-%d')}, tests/arm/cap_fit.py from:",
                *[f"  {os.path.basename(p)}" for p in a.runs],
                "Channel k's change from its in-air value ~ gains[k] * curve(deflection), loading side.",
                "In-air values are taken at run time (they drift), never from this file."]
        with open(a.out, "w") as f:
            f.write(C.dump(cal, meta))
        print(f"wrote {a.out}")


if __name__ == "__main__":
    main()
