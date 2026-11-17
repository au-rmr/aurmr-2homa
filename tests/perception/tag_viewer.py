"""Live window: the Kinect colour image with every AprilTag it detects (id, size in px). For placing calibration tags.

    python3 tests/perception/tag_viewer.py [--camera /camera_lower_right] [--family 36h11] [--scale 0.3]
Needs DISPLAY. Read-only. Esc or q closes it.
"""
import argparse

import cv2
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--camera", default="/camera_lower_right")
    p.add_argument("--family", default="36h11")
    p.add_argument("--scale", type=float, default=0.3)
    a = p.parse_args()
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, f"DICT_APRILTAG_{a.family}"))
    params = cv2.aruco.DetectorParameters_create()
    rclpy.init()
    node = rclpy.create_node("tag_viewer")
    latest = {}
    node.create_subscription(Image, f"{a.camera}/rgb/image_raw", lambda m: latest.update(m=m),
                             QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    cv2.namedWindow("tags", cv2.WINDOW_NORMAL)
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        m = latest.pop("m", None)
        if m is not None:
            img = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, -1)[..., :3].copy()
            corners, ids, _ = cv2.aruco.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), dictionary, parameters=params)
            small = cv2.resize(img, None, fx=a.scale, fy=a.scale, interpolation=cv2.INTER_AREA)
            seen = []
            if ids is not None:
                for c, i in zip(corners, ids.ravel()):
                    c = c.reshape(4, 2)
                    side = float(np.mean([np.linalg.norm(c[k] - c[(k + 1) % 4]) for k in range(4)]))
                    cv2.polylines(small, [(c * a.scale).astype(np.int32)], True, (0, 255, 0), 3)
                    cv2.putText(small, f"{i} ({side:.0f}px)", tuple((c[0] * a.scale).astype(int) + [0, -10]), 0, 0.9,
                                (0, 255, 0), 2)
                    seen.append(int(i))
            cv2.putText(small, f"tags: {sorted(seen)}", (20, 40), 0, 1.2, (0, 0, 255), 3)
            cv2.imshow("tags", small)
        if cv2.waitKey(1) & 0xFF in (27, ord("q")):
            break
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
