"""A demo that starts a slow straight move and prints 'moving'; the test kills it."""
import os
import threading
import time

from tahoma_robot import Robot

robot = Robot.connect("capcup", 50.0, os.environ["WORKCELL"])
threading.Timer(0.5, lambda: print("moving", flush=True)).start()
robot.move_straight((0, 0, -0.1), speed=0.03)
time.sleep(30)
