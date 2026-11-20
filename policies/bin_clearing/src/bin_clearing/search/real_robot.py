"""Real-robot search: tracked poses -> bin-frame scene -> MCTS / AlphaZero plan -> solution file for the MPPI planner.

From puzzle @ 60e5fc6 pipeline.py (Paolo Torrado), real-robot mode (`--config-name=pipeline_real_robot`,
scenario_source=server). Kept unchanged: the pose -> bin-frame conversion (_run_real_robot_scenario), planning +
verification (_plan_scenario), the solution file (solution.save_solution) and the scenario YAML. Dropped: the
zerorpc bridge_server and subprocess orchestration, wandb, CSV, video and benchmark metrics (the ROS 2 node owns
processes and transport now). Run planning in its own process: the bin simulator starts an Isaac app on import
(one per process), set ISAACLAB_HEADLESS=1 before importing for no viewer.
"""
import os
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch
import yaml
from hydra import compose, initialize_config_dir
from omegaconf import DictConfig, open_dict

CONF_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "conf")


@dataclass
class PuzzleResult:
    scenario_idx: int
    scenario_name: str
    seed: int
    success: bool
    plan_length: int
    batch_calls: int
    total_pairs: int
    plan_time_s: float
    verify_rate: float = 0.0
    verify_successes: int | None = None
    verify_passed: bool | None = None
    verify_std: float | None = None
    network_forward_passes: int | None = None
    budget_cap: int | None = None


def search_config(planner: str = "mcts", overrides: list[str] | None = None) -> DictConfig:
    """The pipeline's config: pipeline_real_robot + planner=<planner> (+ hydra-style overrides)."""
    with initialize_config_dir(config_dir=CONF_DIR, version_base=None):
        return compose(config_name="pipeline_real_robot", overrides=[f"planner={planner}", *(overrides or [])])


def scene_from_tracked_poses(raw_poses: torch.Tensor, cfg: DictConfig) -> tuple[dict, list]:
    """Tracked poses (MPPI world frame) -> (initial_state in the bin frame, bin_center).

    raw_poses: flat [pos(3), quat wxyz(4)] per object in isaac_index order: target, obstacles..., then the two bin
    trackers (back wall, side). Unchanged from pipeline._run_real_robot_scenario.
    """
    n_objects = raw_poses.numel() // 7

    # Print all received MPPI positions so we can verify the tracker.
    print(f'[real-robot] {n_objects} total objects from server:')
    for i in range(n_objects):
        p = raw_poses[i * 7: i * 7 + 3].tolist()
        label = 'bin-tracker' if i >= n_objects - 2 else f'object[{i}]'
        print(f'  {label}: MPPI=[{p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f}]')

    # Last 2 objects are bin trackers (back-wall, side); back-wall is at n_objects-2.
    bin_tracker_pos = raw_poses[(n_objects - 2) * 7: (n_objects - 2) * 7 + 3].tolist()
    n_puzzle_objects = n_objects - 2  # cube objects only, excluding both bin trackers

    bin_s = cfg.get('bin_size', 0.4)
    # Back wall center: MPPI x = x0 + bin_s, MPPI y = y0 + bin_s/2
    x0 = bin_tracker_pos[0] - bin_s        # MPPI x at exit side (EW=0 in bin frame)
    y0 = bin_tracker_pos[1] - bin_s / 2    # MPPI y at NS=0 in bin frame
    bin_center = [x0 + bin_s / 2, y0 + bin_s / 2]
    print(f'[real-robot] bin_s={bin_s}  tracker=[{bin_tracker_pos[0]:.4f}, {bin_tracker_pos[1]:.4f}]'
          f'  x0={x0:.4f}  y0={y0:.4f}  bin_center={[round(v,4) for v in bin_center]}')

    def _world_to_bin(wp):
        # Pure translation — matches simulators/isaaclab_env.py's own bin-local
        # convention ("Bin placed with NS along world x, EW along world y.
        # No axis swap."), which is what the puzzle solver's _is_goal() check
        # actually depends on. Do not add an axis swap or quaternion rotation
        # here without re-deriving against that reference.
        return [wp[0] - x0, wp[1] - y0, wp[2] - 1.225]

    positions_bin = [_world_to_bin(raw_poses[i * 7: i * 7 + 3].tolist()) for i in range(n_puzzle_objects)]
    quats         = [raw_poses[i * 7 + 3: i * 7 + 7].tolist() for i in range(n_puzzle_objects)]
    n_obs = n_puzzle_objects - 1

    print(f'[real-robot] {n_puzzle_objects} puzzle objects — bin-frame positions (should be in [0, {bin_s}]):')
    for i, pos in enumerate(positions_bin):
        ok = all(0 <= pos[j] <= bin_s for j in range(2))
        label = 'target' if i == 0 else f'obstacle[{i-1}]'
        flag = '' if ok else '  *** OUTSIDE BIN ***'
        print(f'  {label}: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]{flag}')

    # Clamp x/y to [0, bin_s]: objects within noise-tolerance of a wall are snapped inside.
    # Real-robot mode uses actual bin-frame positions — no centre-shift (which would
    # push interior obstacles outside the opposite wall when the target is near the exit).
    eps = 1e-3
    for pos in positions_bin:
        pos[0] = float(max(eps, min(bin_s - eps, pos[0])))
        pos[1] = float(max(eps, min(bin_s - eps, pos[1])))

    initial_state = {
        'target_pos':    torch.tensor(positions_bin[0]),
        'target_quat':   torch.tensor(quats[0]),
        'obstacle_pos':  torch.tensor(positions_bin[1:]),
        'obstacle_quat': torch.tensor(quats[1:]),
    }

    obj_s    = cfg.get('obj_size', 0.05)
    bottom_z = obj_s / 2  # centre z of a cube resting on the floor
    all_z    = [float(initial_state['target_pos'][2])]
    if initial_state['obstacle_pos'].numel() > 0:
        all_z += initial_state['obstacle_pos'][:, 2].tolist()
    shift_z = bottom_z - min(all_z)
    initial_state['target_pos'][2] += shift_z
    if initial_state['obstacle_pos'].numel() > 0:
        initial_state['obstacle_pos'][:, 2] += shift_z
    print(f'[real-robot] z-shifted scene by {shift_z:+.4f} m (lowest object now at z={bottom_z:.4f})')

    # Snap all block z-coords to the nearest discrete level (obj_s/2, 3*obj_s/2, …).
    # Pose estimation noise puts blocks 5-20 mm above their true level; AZ's network
    # was trained on exact levels and degrades when z is out-of-distribution.
    _n_levels = cfg.get('max_stack_height', 3) + 1
    _z_levels = [obj_s / 2 + k * obj_s for k in range(_n_levels)]
    def _snap_z(z: float) -> float:
        return min(_z_levels, key=lambda zl: abs(zl - z))
    initial_state['target_pos'][2] = _snap_z(float(initial_state['target_pos'][2]))
    if initial_state['obstacle_pos'].numel() > 0:
        for _i in range(len(initial_state['obstacle_pos'])):
            initial_state['obstacle_pos'][_i][2] = _snap_z(float(initial_state['obstacle_pos'][_i][2]))

    return initial_state, bin_center


def prepare_config(cfg: DictConfig, initial_state: dict) -> None:
    """Per-scene config, as the pipeline set it before planning (n_obstacles; AlphaZero: n_z_levels from its checkpoint)."""
    with open_dict(cfg):
        cfg.n_obstacles = len(initial_state['obstacle_pos'])
        # If using AlphaZero, propagate n_z_levels from the checkpoint spec so
        # the planning env and SolverGame action space match the trained network.
        if cfg.planner.name == 'alphazero':
            import torch as _torch
            _ckpt = _torch.load(cfg.planner.checkpoint, map_location='cpu', weights_only=False)
            _spec_z = _ckpt.get('spec', {}).get('Z', None)
            if _spec_z and _spec_z != cfg.n_z_levels:
                print(f'[real-robot] AlphaZero checkpoint requires n_z_levels={_spec_z} '
                      f'(config has {cfg.n_z_levels}) — overriding.')
                cfg.n_z_levels = _spec_z


def build_search_env(cfg: DictConfig):
    """The bin simulator the search plans with (cfg.parallel_envs parallel Isaac Lab envs). Starts Isaac on first import."""
    from .simulators import build_env
    viewer = cfg.get("viewer", "headless")
    if viewer == 'headless':
        os.environ['ISAACLAB_HEADLESS'] = '1'
    return build_env(cfg, n_envs=cfg.parallel_envs, viewer_mode=viewer)


def _plan_scenario(env, cfg: DictConfig, scenario_idx: int, scenario_name: str,
                   initial_state: dict, seed: int) -> tuple[PuzzleResult, list | None]:
    """Run one planning attempt. Returns (PuzzleResult, plan_or_None)."""
    print(f'\n[Scenario {scenario_idx + 1}/{cfg.n_scenarios}] {scenario_name} seed={seed}')

    env.reset_sim_counters()
    from .planner import _PlannerBase
    planner = _PlannerBase.from_cfg(env, cfg, seed)

    t0 = time.time()
    plan = planner.plan(initial_state, verbose=cfg.debug)
    plan_time = time.time() - t0

    success = plan is not None and len(plan) > 0
    result = PuzzleResult(
        scenario_idx=scenario_idx,
        scenario_name=scenario_name,
        seed=seed,
        success=success,
        plan_length=len(plan) if success else 0,
        batch_calls=env.batch_calls,
        total_pairs=env.total_pairs,
        plan_time_s=plan_time,
    )

    status = 'SUCCESS' if success else 'FAILED'
    print(f'  {status} | plan_len={result.plan_length} | '
          f'batch_calls={result.batch_calls} | time={plan_time:.1f}s')

    if not success:
        return result, None

    verify_successes, _, verify_rate, verify_passed, verify_flags = planner.verify(plan, initial_state)
    result.verify_rate = verify_rate
    result.verify_successes = verify_successes
    result.verify_passed = verify_passed
    result.verify_std = float(np.std(verify_flags)) if verify_flags else None
    result.network_forward_passes = getattr(planner, 'network_forward_passes', None)
    result.budget_cap = getattr(cfg.planner, 'n_simulations', None)
    print(f'  Verify: {verify_successes}/{env.n_envs} ({verify_rate:.0%}) — '
          f'{"PASS" if verify_passed else "FAIL"}')

    return result, plan


def plan_scene(env, cfg: DictConfig, initial_state: dict, out_dir: str | Path,
               scenario_name: str = 'server_scenario') -> tuple[PuzzleResult, list | None, Path | None]:
    """Plan + verify one scene, as the pipeline's worker did (seed = cfg.seed + 0). On success writes
    <out_dir>/solutions/<scenario_name>.json (the MPPI planner's input) and returns its path.
    The pipeline launched MPC only if result.success and result.verify_passed: the caller must check both."""
    seed = cfg.seed if cfg.seed is not None else 0
    result, plan = _plan_scenario(env, cfg, 0, scenario_name, initial_state, seed)
    if plan is None:
        return result, None, None
    from .solution import save_solution
    solution_path = Path(out_dir) / 'solutions' / f'{scenario_name}.json'
    solution_path.parent.mkdir(parents=True, exist_ok=True)
    save_solution(str(solution_path), plan, initial_state, cfg, env)
    return result, plan, solution_path


def write_scenario_yaml(path: str | Path, initial_state: dict, bin_center: list, bin_size: float) -> Path:
    """The scenario YAML the MPPI planner reads (tracked positions in the bin frame + the tracked bin centre)."""
    n_obs = len(initial_state['obstacle_pos'])
    obstacle_pos = initial_state['obstacle_pos']
    obstacle_quat = initial_state['obstacle_quat']
    scenario_data = {
        'bin_size': float(bin_size),
        'bin_center': [round(v, 6) for v in bin_center],
        'initial_state': {
            'target_pos':  [round(v, 6) for v in initial_state['target_pos'].tolist()],
            'target_quat': [round(v, 6) for v in initial_state['target_quat'].tolist()],
            'obstacles': [
                {
                    'pos':  [round(v, 6) for v in obstacle_pos[i].tolist()],
                    'quat': [round(v, 6) for v in obstacle_quat[i].tolist()],
                }
                for i in range(n_obs)
            ],
        },
    }
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'w') as f:
        yaml.dump(scenario_data, f, default_flow_style=False, sort_keys=False)
    return path


def load_scenario_yaml(path: str | Path) -> tuple[dict, list | None, float | None]:
    """A scenario YAML -> (initial_state, bin_center, bin_size) (pipeline._load_initial_state_from_yaml + metadata)."""
    with open(path) as f:
        sc = yaml.safe_load(f)
    ist = sc['initial_state']
    return ({
        'target_pos':    torch.tensor(ist['target_pos']),
        'target_quat':   torch.tensor(ist['target_quat']),
        'obstacle_pos':  torch.tensor([o['pos']  for o in ist['obstacles']]),
        'obstacle_quat': torch.tensor([o['quat'] for o in ist['obstacles']]),
    }, sc.get('bin_center'), sc.get('bin_size'))
