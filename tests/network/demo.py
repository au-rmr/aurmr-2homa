"""Stands in for a demo on another host. slow: a long move to be cut. quick: a short full move."""
import os
import sys

from tahoma_robot import Robot

with Robot.connect("capcup", 50.0, os.environ["WORKCELL"]) as robot:
    robot.go_named("pre_bin_3g", speed=0.8)
    print("CONNECTED", flush=True)
    if sys.argv[1] == "slow":
        robot.move_straight((0, 0, -0.16), speed=0.02)
    else:
        print(robot.move_straight((0, 0, -0.03), speed=0.1).reason.upper(), flush=True)
