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
