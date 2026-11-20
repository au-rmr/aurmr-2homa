# bin_clearing: retrieve a target cube from a cluttered, stacked bin (MCTS / AlphaZero search + MPPI)

Runtime code of the MPC + MCTS demo: a search over pushes/pulls finds a plan that gets the target cube out of the
bin, then an MPPI controller (Isaac Lab parallel physics) turns each plan step into joint velocity commands.
Inference only; training, benchmarks and paper scripts stay in the author's research repos.

## Credits and origin
Author: **Paolo Torrado** (University of Washington). Paper: Paolo Torrado et al., *Reactive Control for
Cluttered-Scene Non-Prehensile Manipulation* (under review), listed at https://paolotorrado.com/#pubs.
Licence: MIT (the repository's LICENSE), with the author's agreement (2026-11-19).

Copied from his research repos at the commits the demo ran:
- `src/bin_clearing/search/` from puzzle (github.com/patorrad/puzzles) @ 60e5fc6, branch MORE: the MCTS search,
  the Isaac Lab bin simulator it searches with, the real-robot scene conversion and solution format.
- `src/bin_clearing/mpc/` from isaaclabmpc (github.com/patorrad/isaaclabmpc) @ 52c09b4: the MPPI planner on Isaac
  Lab, its cost terms, the stacked-bin task (`examples/ur16e_stacked_robot`) and the UR16e model with his tool.
- Third-party, pinned in the planner image: mppi_torch (tud-amr/mppi_torch @ 75e17e8), Isaac Lab, Isaac Sim, and
  the UR16e meshes from ros-industrial/universal_robot @ 6507bb6 (BSD-3-Clause).

`ORIGIN.txt` lists every copied file with its source and checksum. The first commit is the verbatim copy; every
change after it is in git history.

## Changes from the original (second commit; `git log -p` has the details)
Only what it takes to run as a package inside the planner image; no algorithm or parameter changed.
- Imports made package-relative; `sys.path` hacks removed. MORE raises NotImplementedError (not used by the demo).
- `search/real_robot.py` (was puzzle `pipeline.py`): the real-robot pose -> bin-frame conversion, planning +
  verification and the scenario YAML, code unchanged; the zerorpc bridge server, subprocess orchestration, wandb,
  CSV, video and benchmark metrics are gone (the ROS 2 node replaces them).
- `search/solution.py` (was puzzle `main.py`): only `save_solution` and its helpers.
- `mpc/stacked_bin/planner.py`: the module-level AppLauncher + argparse and the zerorpc server are replaced by
  `build_planner(scenario_path, solution_path)`; the caller launches Isaac first. `config.yaml`: absolute
  `/home/paolo` paths set to null (solution given by the caller; the stand URDF was unused).
- `mpc/assets/robots/ur16e.py`: uses the meshes at `$UR_DESCRIPTION_MESHES` (set in the planner image).

## Running
Inside the planner image (`compose/planner.yaml`; `PYTHONPATH` includes `policies/bin_clearing/src`). Isaac allows
one app per process, so the search and the MPPI planner run in separate processes, as in the original pipeline.
```
python -m bin_clearing.search --scenario <scenario.yaml> --out <dir>    # -> <dir>/solutions/server_scenario.json
```
```python
from isaaclab.app import AppLauncher; app = AppLauncher(headless=True).app      # first
from bin_clearing.mpc.stacked_bin.planner import build_planner
planner = build_planner("<scenario.yaml>", "<dir>/solutions/server_scenario.json")
u = planner.compute_action_tensor(state_bytes, root_bytes)   # torch.save bytes, as the old bridge sent
```

## Checks (2026-11-19, against the original at the pinned commits, on the lab's reference runs)
- Search, run `5obs_stacked2_mcts_05`, seed 42: same plan bit for bit as the original's usual result (4 of 5
  original runs; the 5th moved the second push point 4.7 mm: GPU physics). The solution file is equal except the
  replayed step poses, which vary by ~6 mm between original runs too.
- MPPI, same scene + solution: first command identical, second within 1.4e-7; after that runs diverge, as two
  runs of the original do (not repeatable run to run).
Harnesses and data: the lab's reference set `mpc_mcts_real_robot_2026-09` (not in this repo).
