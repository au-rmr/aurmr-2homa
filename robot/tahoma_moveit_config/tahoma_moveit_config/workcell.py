"""Turns a site's workcell.yaml into MoveIt collision objects."""
import numpy as np
import yaml
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive


WALL = 0.05
WALL_HEIGHT = 3.0
BEHIND = -1.0  # the walls run back this far past the robot


def _walls(name, limit):
    """Vertical keep-out walls with their inner faces on the limits."""
    x_max, y_min, y_max = limit["x_max"], limit["y_min"], limit["y_max"]
    z = WALL_HEIGHT / 2
    x_mid, x_len = (BEHIND + x_max) / 2, x_max - BEHIND
    y_mid, y_len = (y_min + y_max) / 2, y_max - y_min
    return {
        f"{name}_x_max": {"frame": "base_link", "xyz": [x_max + WALL / 2, y_mid, z],
                          "size": [WALL, y_len, WALL_HEIGHT]},
        f"{name}_y_min": {"frame": "base_link", "xyz": [x_mid, y_min - WALL / 2, z],
                          "size": [x_len, WALL, WALL_HEIGHT]},
        f"{name}_y_max": {"frame": "base_link", "xyz": [x_mid, y_max + WALL / 2, z],
                          "size": [x_len, WALL, WALL_HEIGHT]},
    }


def _bins(name, spec):
    """A pod face's fabric bins as boxes: one plate per shelf level, a full-height divider
    between columns, the back, and the lip (flap) at the front of each bin's floor.

    In the pod frame: x across the face from column 1, y into the pod (0 = the face), z up.
    spec: frame, origin (front bottom left corner of the bottom left bin), columns, bin_width,
    depth, row_heights (bottom row first, rows named a, b, c...), flap_height, min_wall.
    The real fabric is 2 to 7 mm. Walls are modelled at least min_wall thick, centred on the
    real wall, because straight moves are checked only at their waypoints (5 mm apart), so a
    thinner wall could fall between two of them.
    """
    x0, y0, z0 = spec["origin"]
    t = spec["min_wall"]
    width = spec["columns"] * spec["bin_width"]
    depth = spec["depth"]
    levels = [z0]
    for h in spec["row_heights"]:
        levels.append(levels[-1] + h)
    height = levels[-1] - z0
    frame = spec["frame"]
    out = {}
    for i, z in enumerate(levels):
        out[f"{name}_shelf_{i}"] = {"frame": frame, "xyz": [x0 + width / 2, y0 + depth / 2, z],
                                    "size": [width, depth, t]}
    for j in range(spec["columns"] + 1):
        x = x0 + j * spec["bin_width"]
        out[f"{name}_divider_{j}"] = {"frame": frame, "xyz": [x, y0 + depth / 2, z0 + height / 2],
                                      "size": [t, depth, height]}
    out[f"{name}_back"] = {"frame": frame, "xyz": [x0 + width / 2, y0 + depth, z0 + height / 2],
                           "size": [width, t, height]}
    flap = spec["flap_height"]
    for i, z in enumerate(levels[:-1]):
        row = "abcdefghijklmnopqrstuvwxyz"[i]
        out[f"{name}_flap_{row}"] = {"frame": frame, "xyz": [x0 + width / 2, y0, z + flap / 2],
                                     "size": [width, t, flap]}
    return out


def bin_box(workcell, pod, bin_id):
    """Inner corners of bin bin_id (e.g. "2h": column 2, row h) in the pod frame:
    ((x_min, y_min, z_min), (x_max, y_max, z_max)), the real fabric, not the inflated walls."""
    spec = workcell["pods"][pod]
    col, row = int(bin_id[:-1]), "abcdefghijklmnopqrstuvwxyz".index(bin_id[-1].lower())
    if not 1 <= col <= spec["columns"] or row >= len(spec["row_heights"]):
        raise KeyError(f"no bin {bin_id} in {pod}")
    x0, y0, z0 = spec["origin"]
    z = z0 + sum(spec["row_heights"][:row])
    x = x0 + (col - 1) * spec["bin_width"]
    return ((x, y0, z), (x + spec["bin_width"], y0 + spec["depth"], z + spec["row_heights"][row]))


def load(path):
    with open(path) as f:
        workcell = yaml.safe_load(f)
    for name, limit in workcell.get("limits", {}).items():
        walls = _walls(name, limit)
        workcell["objects"].update(walls)
        workcell["scenes"][name] = list(walls)
    for name, spec in workcell.get("pods", {}).items():
        boxes = _bins(name, spec)
        workcell["objects"].update(boxes)
        workcell["scenes"][name] = list(boxes)
    return workcell


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
