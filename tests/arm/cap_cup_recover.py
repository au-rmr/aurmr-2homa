"""Gets the arm out of the cap_cup bin after a run stopped part way (killed, e-stop, error).

    python3 tests/arm/cap_cup_recover.py [--site-dir DIR] [--keep-vacuum]

Vacuum off first (unless --keep-vacuum: an item that may be held stays held), then the demo's
own retreat: straight back out along the approach direction until an item as deep as the bin
would clear the pod face, with the bin scene (pod face open, bins modelled). If the vacuum
holds, the bin-sized box is attached as in the demo. Then the planned move to drop_hide, and
the box is released. Two motions: run it only with the person at the e-stop.
"""
import argparse
import os

import yaml
from aurmr_skills import load_tool
from tahoma_robot import Robot
from tahoma_robot.robot import _rotate

from cap_cup_demo.task import Config, held_item_size, hide, retreat


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--site-dir", default=os.environ.get("AURMR_SITE_DIR", "/repo/sites/uw_lab"))
    p.add_argument("--keep-vacuum", action="store_true")
    args = p.parse_args()
    with open(os.path.join(args.site_dir, "demos", "cap_cup.yaml")) as f:
        cfg = Config(**yaml.safe_load(f), site_dir=args.site_dir)
    with Robot.connect("capcup", 50.0, os.path.join(args.site_dir, "workcell.yaml")) as robot, \
            load_tool(robot, needs=["suction"]) as cup:
        if not args.keep_vacuum:
            cup.off()
            print("vacuum off", flush=True)
        robot.scene.release()
        robot.scene.use(*cfg.scene_bin)
        way_in = _rotate(cfg.approach_quat, (0.0, 0.0, 1.0))
        item = held_item_size(robot, cfg)
        print(f"start: tip {tuple(round(v, 3) for v in robot.tcp_pose()[0])}", flush=True)
        retreat(robot, cfg, way_in, item)
        print(f"retreat ended: tip x {robot.tcp_pose()[0][0]:.3f}", flush=True)
        if args.keep_vacuum and cup.sealed():
            robot.scene.hold(item, start=cfg.held_start_m)
            print("holding the bin-sized box", flush=True)
        hide(robot, cfg, print)
        robot.scene.release()
        print("RESULT at drop_hide", flush=True)


if __name__ == "__main__":
    main()
