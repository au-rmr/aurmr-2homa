"""Plan one scene from a scenario YAML (runs in its own process: the search starts an Isaac app).

    python -m bin_clearing.search --scenario <scenario.yaml> --out <dir> [--planner mcts] [--override k=v ...]

Writes <out>/solutions/server_scenario.json (for the MPPI planner) and <out>/plan.json (plan + result summary).
Exit code 0 only if a plan was found AND passed verification (the pipeline's condition for running MPC).
"""
import argparse
import json
import os
import sys
from dataclasses import asdict


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--planner", default="mcts")
    ap.add_argument("--name", default="server_scenario")
    ap.add_argument("--override", nargs="*", default=[])
    args = ap.parse_args()

    from omegaconf import open_dict
    from .real_robot import build_search_env, load_scenario_yaml, plan_scene, prepare_config, search_config

    cfg = search_config(args.planner, args.override)
    initial_state, _, bin_size = load_scenario_yaml(args.scenario)
    with open_dict(cfg):
        if bin_size is not None:
            cfg.bin_size = bin_size
        if not any(o.startswith("viewer=") for o in args.override):
            cfg.viewer = "headless"  # pipeline_real_robot says replay (viewer window); headless unless asked
    prepare_config(cfg, initial_state)
    env = build_search_env(cfg)
    result, plan, solution_path = plan_scene(env, cfg, initial_state, args.out, args.name)

    def plain(v):
        return v.tolist() if hasattr(v, "tolist") else v

    summary = {"result": asdict(result), "solution_path": str(solution_path) if solution_path else None,
               "plan": [{k: plain(v) for k, v in a.items()} for a in (plan or [])]}
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, "plan.json"), "w") as f:
        json.dump(summary, f, indent=1)
    ok = bool(result.success and result.verify_passed)
    print(f"SEARCH {'OK' if ok else 'FAILED'} plan_len={result.plan_length} verify={result.verify_rate:.0%} "
          f"time={result.plan_time_s:.1f}s -> {args.out}", flush=True)
    sys.stdout.flush()
    os._exit(0 if ok else 1)  # Isaac Lab teardown can hang (the pipeline killed its worker for the same reason)


if __name__ == "__main__":
    main()
