"""Turns a site's workcell.yaml into MoveIt collision objects."""
import numpy as np
import yaml
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive


def load(path):
    with open(path) as f:
        return yaml.safe_load(f)


def _matrix(xyz, rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    T = np.eye(4)
    T[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                 [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                 [-sp, cp * sr, cp * cr]]
    T[:3, 3] = xyz
    return T


def _quaternion(R):
    w = np.sqrt(max(0.0, 1 + np.trace(R))) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return x, y, z, w


def object_names(workcell, names):
    """Scene and object names, in any mix, to object names."""
    out = []
    for name in names:
        if name in workcell["scenes"]:
            out += workcell["scenes"][name]
        elif name in workcell["objects"]:
            out.append(name)
        else:
            raise KeyError(f"no scene or object called {name}")
    return out


def collision_object(workcell, name):
    spec = workcell["objects"][name]
    T = _matrix(spec["xyz"], spec.get("rpy", [0, 0, 0]))
    frame = spec["frame"]
    while frame in workcell["frames"]:
        parent = workcell["frames"][frame]
        T = _matrix(parent["xyz"], parent["rpy"]) @ T
        frame = parent["parent"]
    co = CollisionObject()
    co.id = name
    co.header.frame_id = frame
    co.operation = CollisionObject.ADD
    box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in spec["size"]])
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in T[:3, 3])
    q = pose.orientation
    q.x, q.y, q.z, q.w = (float(v) for v in _quaternion(T[:3, :3]))
    co.primitives.append(box)
    co.primitive_poses.append(pose)
    return co
