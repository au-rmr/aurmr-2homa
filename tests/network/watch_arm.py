"""Runs beside the stack. Reports when the arm starts moving and when it stops, in wall time."""
import os
import time

from tahoma_robot import Robot

robot = Robot("capcup", 50.0, os.environ["WORKCELL"])
while len(robot.joints()) < 6:
    time.sleep(0.05)
print("READY", flush=True)
moving, previous, last_change = False, robot.joints(), time.time()
end = time.monotonic() + 180
while time.monotonic() < end:
    time.sleep(0.01)
    now = robot.joints()
    if max(abs(now[k] - previous[k]) for k in now) > 1e-6:
        last_change = time.time()
        if not moving:
            moving = True
            print("MOVING", flush=True)
    elif moving and time.time() - last_change > 0.3:
        moving = False
        print(f"STOPPED {last_change:.3f}", flush=True)
    previous = now
