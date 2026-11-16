# Raw calibration data (uw_lab)
The measurements behind the fitted values in this site's yaml files, kept so a value can be checked or
refit without going back to the arm. Small text only (repo limit 5 MB per file).

| Folder | What | Fitted into | Tool |
|---|---|---|---|
| `pod_2026-11-10/` | touch points on the pod (`points_raw.csv`, as tests/arm/touch_point.py writes it: name, tip x y z, tool axis x y z, spread mm) and `heights_raw.csv`, a height check (table_down's top touched at 0.714 vs tape 0.722) | `workcell.yaml` frames.pod_base_link (`fit_pod.py --floor points_raw.csv` gives it exactly, checked 2026-11-16) | `tests/arm/fit_pod.py`, docs/procedures/pod_calibration.md |
| `table_up_2026-11-12/` | 4 touches on the raised desk (aluminium end) | `workcell.yaml` objects.table_up | plane fit, comment in workcell.yaml |
| `capcup_2026-11-12/` | cap press runs A, B (roll 0) and C (roll 180) + the fit's output | `tools/capcup_calibration.yaml` | `tests/arm/cap_fit.py`, docs/procedures/s6_cap_cup.md |
