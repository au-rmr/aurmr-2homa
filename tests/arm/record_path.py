"""Records a candidate path for cap_cup: drop_hide -> pre_approach, planned by OMPL, run with the
person at the e-stop watching the hose, then run back reversed (user 2026-11-13: plan a few, keep
a good one, replay it every pick).

    python3 tests/arm/record_path.py OUT.yaml [--site-dir DIR] [--no-return]

Two motions under one go: the planned path to pre_approach (from drop_hide; first a planned move
to drop_hide if the arm isn't there), then the same path reversed back to drop_hide. The path is
written to OUT.yaml before it runs. To keep it, copy it to sites/<site>/demos/<path_file>.
"""
import argparse
import os

import yaml
from tahoma_robot import Robot
from tahoma_robot.robot import path_from_dict, path_to_dict

from cap_cup_demo.task import Config, held_item_size


def main():
    p = argparse.ArgumentParser()
    p.add_argument("out")
    p.add_argument("--site-dir", default=os.environ.get("AURMR_SITE_DIR", "/repo/sites/uw_lab"))
    p.add_argument("--no-return", action="store_true")
    args = p.parse_args()
    with open(os.path.join(args.site_dir, "demos", "cap_cup.yaml")) as f:
        cfg = yaml.safe_load(f)
    speed = cfg.get("planned_speed", 0.2)
    demo = Config(**cfg, site_dir=args.site_dir)
    with Robot.connect("capcup", 50.0, os.path.join(args.site_dir, "workcell.yaml")) as robot:
        robot.scene.release()
        robot.scene.use(*cfg["scene_hide"])
        print("to drop_hide:", robot.go_named("drop_hide", speed=speed).reason, flush=True)
        # Both scenes at once (the work scene's objects and the hide scene's narrower side walls):
        # the path runs out in the work scene and back in the hide scene.
        robot.scene.use(*cfg["scene_work"], *cfg["scene_hide"])
        # Planned with the worst-case held item on the cup (a box as large as the bin), so the same
        # path is clear both ways, with or without an item (the demo runs it back holding one).
        robot.scene.hold(held_item_size(robot, demo), start=demo.held_start_m)
        trajectory = robot.plan_joints(cfg["pre_approach_joints"], speed=speed)
        data = path_to_dict(trajectory)
        with open(args.out, "w") as f:
            yaml.safe_dump(data, f, default_flow_style=None, width=200)
        print(f"planned: {len(data['points'])} points, {data['points'][-1]['t']:.1f} s; saved {args.out}",
              flush=True)
        print("path out:", robot.run_path(trajectory).reason, flush=True)
        if not args.no_return:
            robot.scene.use(*cfg["scene_hide"])
            print("path back (reversed):", robot.run_path(path_from_dict(data, reverse=True)).reason,
                  flush=True)
        robot.scene.release()


if __name__ == "__main__":
    main()
