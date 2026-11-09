"""A demo that gets killed part way through a move."""
import os

from tahoma_robot import Robot

robot = Robot.connect("capcup", 50.0, os.environ["WORKCELL"])
robot.go_named("pre_bin_3g", speed=0.8)
print("moving", flush=True)
robot.move_straight((0, 0, -0.16), speed=0.02)
