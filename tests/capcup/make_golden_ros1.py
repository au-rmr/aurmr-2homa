"""Runs the OLD ROS 1 touch detector and haptic search, unchanged, on tests/capcup/sequence.py
and writes what they would have published. Run inside the old capcup image:
  docker run --rm --network none -v $PWD:/repo --entrypoint bash aurmr-p2/inter-capcup:archive \
    -c 'source /opt/ros/noetic/setup.bash; python3 /repo/tests/capcup/make_golden_ros1.py'
Only rospy and the capcup message are stubbed: no ROS master is needed."""
import json
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
OLD = "/ws/src/capacitive_cup"
sys.path[:0] = [HERE, os.path.join(OLD, "src"), os.path.join(OLD, "scripts")]


class Captured:
    def __init__(self, *a, **k):
        self.last = None

    def publish(self, msg):
        self.last = msg


class Obj:
    def __init__(self, *a, **k):
        pass


rospy = types.ModuleType("rospy")
rospy.Subscriber = Obj
rospy.Publisher = Captured
rospy.Timer = Obj
rospy.Duration = Obj
rospy.Time = types.SimpleNamespace(now=lambda: 0)
rospy.loginfo = rospy.logwarn = lambda *a, **k: None
sys.modules["rospy"] = rospy
for name, attrs in {"capacitive_cup": {}, "capacitive_cup.msg": {"CapData": Obj}}.items():
    m = types.ModuleType(name)
    m.__dict__.update(attrs)
    sys.modules[name] = m

# The old code only uses tf.transformations; importing all of tf would need a real rospy.
import importlib.util  # noqa: E402
tf = types.ModuleType("tf")
spec = importlib.util.spec_from_file_location(
    "tf.transformations", "/opt/ros/noetic/lib/python3/dist-packages/tf/transformations.py")
tf.transformations = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tf.transformations)
sys.modules["tf"], sys.modules["tf.transformations"] = tf, tf.transformations

import cap_haptic_search  # noqa: E402
import touch_detector  # noqa: E402
from sequence import samples  # noqa: E402


def num(v):
    import numpy as np
    return float(np.asarray(v).ravel()[0])


touch = touch_detector.TouchDetector(time_constant=10)
search = cap_haptic_search.HapticSearch()
rows = []
for ms, values in samples():
    msg = types.SimpleNamespace(timestamp=ms * 1e-3, values=values)   # inter's bridge: seconds
    touch.callback(msg)
    search._sensor_cb(msg)
    search._pose_pub.last = None
    search._alignment_pub.last = None
    search._update(None)
    pose = search._pose_pub.last
    rows.append({
        "contact": bool(touch.contact),
        "pose": None if pose is None else [num(pose.pose.position.x), num(pose.pose.position.y),
                                           num(pose.pose.position.z), num(pose.pose.orientation.x),
                                           num(pose.pose.orientation.y), num(pose.pose.orientation.z),
                                           num(pose.pose.orientation.w)],
        "aligned": None if search._alignment_pub.last is None else bool(search._alignment_pub.last),
    })
with open(os.path.join(HERE, "golden_ros1.json"), "w") as f:
    json.dump({"source": "aurmr-p2/inter-capcup:archive, /ws/src/capacitive_cup", "rows": rows}, f)
print(f"wrote {len(rows)} rows, contact in {sum(r['contact'] for r in rows)}, "
      f"poses {sum(r['pose'] is not None for r in rows)}, aligned {sum(bool(r['aligned']) for r in rows)}")
