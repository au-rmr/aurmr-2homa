"""Solution file written by the search and read by the MPPI planner (format: 'steps' in the target frame).

From puzzle @ 60e5fc6 main.py (Paolo Torrado): save_solution and its helpers, unchanged. The rest of main.py (the
standalone solve/replay CLI) is not needed by the demo and was dropped.
"""
import json
import logging

from omegaconf import DictConfig

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Replay helpers
# ---------------------------------------------------------------------------

def _bin_to_target_frame(pos: list, target_pos: list) -> list:
    """Convert a bin-frame position to target-relative frame.

    Both puzzles and MPC now use x=north-south/forward, y=east-west/lateral,
    so this is a pure subtraction — no axis swap needed.

    At deploy time: abs_mpc_pos = observed_target_pos + offset.
    """
    bx, by, bz = pos
    tx, ty, tz = target_pos
    return [bx - tx, by - ty, bz - tz]


def _simulate_plan_steps(env, plan: list[dict], initial_state: dict) -> list[dict]:
    steps = []
    state = initial_state
    env.set_state(state)
    target_initial = initial_state['target_pos'].tolist()

    for action in plan:
        obj_idx  = int(action['obj_idx'])
        obj_name = 'target' if obj_idx == 0 else f'obstacle_{obj_idx - 1}'

        if obj_idx == 0:
            start_pos  = state['target_pos'].tolist()
            start_quat = state['target_quat'].tolist()
        else:
            start_pos  = state['obstacle_pos'][obj_idx - 1].tolist()
            start_quat = state['obstacle_quat'][obj_idx - 1].tolist()
        target_start_pos  = state['target_pos'].tolist()
        target_start_quat = state['target_quat'].tolist()

        (new_state, _, _), = env.batch_evaluate([(state, action)])
        state = new_state

        if obj_idx == 0:
            end_pos  = state['target_pos'].tolist()
            end_quat = state['target_quat'].tolist()
        else:
            end_pos  = state['obstacle_pos'][obj_idx - 1].tolist()
            end_quat = state['obstacle_quat'][obj_idx - 1].tolist()
        target_end_pos  = state['target_pos'].tolist()
        target_end_quat = state['target_quat'].tolist()

        steps.append({
            'obj_idx':           obj_idx,
            'obj_name':          obj_name,
            'coordinate_frame':  'target',
            'start_pos':         _bin_to_target_frame(start_pos, target_initial),
            'start_quat':        start_quat,
            'end_pos':           _bin_to_target_frame(end_pos, target_initial),
            'end_quat':          end_quat,
            'target_start_pos':  _bin_to_target_frame(target_start_pos, target_initial),
            'target_start_quat': target_start_quat,
            'target_end_pos':    _bin_to_target_frame(target_end_pos, target_initial),
            'target_end_quat':   target_end_quat,
        })

    return steps


def _serialise_action(a: dict) -> dict:
    """Serialise one plan action to a JSON-safe dict regardless of planner type."""
    out = {'action_type': a['action_type'], 'obj_idx': int(a['obj_idx']),
           'push_z': float(a['push_z'])}
    if 'push_pos' in a:
        out['push_pos'] = [float(v) for v in a['push_pos']]
    if 'push_start_xy' in a:
        out['push_start_xy'] = [float(v) for v in a['push_start_xy']]
        out['push_end_xy']   = [float(v) for v in a['push_end_xy']]
    return out


def save_solution(path: str, plan: list[dict], initial_state: dict,
                  cfg: DictConfig, env):
    BIN_W    = env.bin_w
    BIN_D    = env.bin_d
    BIN_H    = 0.5
    OBJ_SIZE = env._OBJ_SIZE
    OBJ_H    = OBJ_SIZE / 2
    EXIT_Y   = -0.05
    PUSHER_T = 0.012
    PUSHER_W = OBJ_SIZE * 0.88
    wt       = cfg.wall_thickness

    def make_actor(name, size, pos, color, fixed=False, rho=500.0):
        return {
            'type': 'Box', 'name': name,
            'init_pos': [float(v) for v in pos],
            'init_ori': [0.0, 0.0, 0.0, 1.0],
            'size': [float(v) for v in size],
            'rho': rho, 'friction': float(cfg.friction),
            'fixed': fixed, 'color': color,
        }

    wall_color  = [0.5, 0.5, 0.8]
    floor_color = [0.7, 0.6, 0.5]
    actors = [
        make_actor('target', [OBJ_SIZE]*3, initial_state['target_pos'].tolist(),
                   [0.9, 0.2, 0.2], rho=50.0),
        *[make_actor(f'obstacle_{i}', [OBJ_SIZE]*3, pos.tolist(), [0.3, 0.5, 0.9])
          for i, pos in enumerate(initial_state['obstacle_pos'])],
        # Actor positions in bin-local frame: x=NS/forward, y=EW/lateral
        make_actor('floor', [BIN_D+2*wt, BIN_W+2*wt, wt],
                   [BIN_D/2, BIN_W/2, -wt/2], floor_color, fixed=True),
        make_actor('wall_north', [wt, BIN_W+2*wt, BIN_H],
                   [BIN_D+wt/2, BIN_W/2, BIN_H/2], wall_color, fixed=True),
        make_actor('wall_west',  [BIN_D, wt, BIN_H],
                   [BIN_D/2, -wt/2, BIN_H/2], wall_color, fixed=True),
        make_actor('wall_east',  [BIN_D, wt, BIN_H],
                   [BIN_D/2, BIN_W+wt/2, BIN_H/2], wall_color, fixed=True),
    ]

    data = {
        # steps use "target" frame (x/y-swapped, target-relative offsets).
        # initial_state and plan remain in bin frame for _do_replay().
        'coordinate_frame': 'target',
        'env_config': {
            'BIN_W': BIN_W, 'BIN_D': BIN_D, 'BIN_H': BIN_H,
            'WALL_T': wt, 'OBJ_SIZE': OBJ_SIZE, 'OBJ_H': OBJ_H,
            'EXIT_Y': EXIT_Y, 'PUSHER_T': PUSHER_T, 'PUSHER_W': PUSHER_W,
            'friction': float(cfg.friction), 'n_obstacles': cfg.n_obstacles,
        },
        'initial_state': {
            'target_pos':    initial_state['target_pos'].tolist(),
            'target_quat':   initial_state['target_quat'].tolist(),
            'obstacle_pos':  initial_state['obstacle_pos'].tolist(),
            'obstacle_quat': initial_state['obstacle_quat'].tolist(),
        },
        'actors': actors,
        'plan': [_serialise_action(a) for a in plan],
        'steps': _simulate_plan_steps(env, plan, initial_state),
    }

    with open(path, 'w') as f:
        json.dump(data, f, indent=2)
    logger.info('Solution saved to %s', path)
