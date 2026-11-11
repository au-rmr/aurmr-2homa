"""Shows the pod calibration's touch points as markers on the model pod (RViz, view_scene.sh).

The model pod is the uncalibrated one, so the markers show roughly where to touch: flush on that
surface, near that height. Arrows point the way the cup points. Run inside view_scene.sh:
    VIEW_EXTRA="python3 /repo/tests/arm/touch_targets.py" scripts/view_scene.sh scene:=pod_bins
"""
import os
import sys

import numpy as np
import rclpy
import yaml
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

# name: (point on the surface in pod_base_link, cup direction), order of docs/procedures/pod_calibration.md
TARGETS = {
    "face_left_low": ((0.0, 0.0, 0.5), (0, 1, 0)),
    "face_left_high": ((0.0, 0.0, 1.5), (0, 1, 0)),
    "face_right_low": ((0.900, 0.0, 0.5), (0, 1, 0)),
    "face_right_high": ((0.900, 0.0, 1.5), (0, 1, 0)),
    "left_side_low": ((-0.015, 0.05, 0.6), (1, 0, 0)),
    "left_side_high": ((-0.015, 0.05, 1.4), (1, 0, 0)),
    "right_side_low": ((0.915, 0.05, 0.6), (-1, 0, 0)),
    "base_top_left": ((0.1, 0.1, 0.255), (0, 0, -1)),
    "base_top_right": ((0.84, 0.1, 0.255), (0, 0, -1)),
}


def main():
    # pod_base_link is not in TF (only in the planning scene): place the markers in base_link.
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from fit_pod import matrix_of
    site = os.environ.get("WORKCELL", "/repo/sites/uw_lab/workcell.yaml")
    pod = yaml.safe_load(open(site))["frames"]["pod_base_link"]
    assert pod["parent"] == "base_link", pod
    R, t = matrix_of(pod["rpy"]), np.array(pod["xyz"])
    rclpy.init()
    node = rclpy.create_node("touch_targets")
    pub = node.create_publisher(MarkerArray, "/workcell_objects",
                                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    out = MarkerArray()
    for i, (name, (p, d)) in enumerate(TARGETS.items()):
        first = i == 0
        p, d = R @ np.array(p) + t, R @ np.array(d, float)
        arrow = Marker(ns="touch_targets", id=2 * i, type=Marker.ARROW, action=Marker.ADD)
        arrow.header.frame_id = "base_link"
        arrow.points = [Point(x=float(p[0] - 0.12 * d[0]), y=float(p[1] - 0.12 * d[1]), z=float(p[2] - 0.12 * d[2])),
                        Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))]
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.012, 0.025, 0.03
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = (1.0, 0.1, 0.1, 1.0) if first else (1.0, 0.6, 0.0, 1.0)
        text = Marker(ns="touch_targets", id=2 * i + 1, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
        text.header.frame_id = "base_link"
        text.pose.position.x, text.pose.position.y, text.pose.position.z = (float(v) for v in (p[0] - 0.15 * d[0], p[1] - 0.15 * d[1], p[2] + 0.04))
        text.scale.z = 0.04
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = name
        out.markers += [arrow, text]
    # Every second: the scene's own latched message starts with DELETEALL, so when RViz
    # re-subscribes (display toggled) it can arrive after ours and wipe the targets.
    pub.publish(out)
    node.create_timer(1.0, lambda: pub.publish(out))
    node.get_logger().info(f"published {len(TARGETS)} touch targets (red = face_left_low)")
    rclpy.spin(node)


if __name__ == "__main__":
    main()
