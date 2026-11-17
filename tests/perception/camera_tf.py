"""Places the Kinect in the robot's TF tree from the site calibration (static TF base_link -> camera_base).

    python3 tests/perception/camera_tf.py <site_dir>/calibration/camera_lower_right.yaml

The calibration gives base_link -> rgb_camera_link. The driver already publishes its own factory tree
camera_base -> rgb_camera_link / depth_camera_link, so a second parent for rgb_camera_link would break TF;
instead this reads the driver's camera_base -> rgb_camera_link once and publishes
base_link -> camera_base = (base_link -> rgb_camera_link) * (camera_base -> rgb_camera_link)^-1.
Runs until stopped (static TF is latched).
"""
import sys

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener


def quat_to_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return [float(x), float(y), float(z), float(w)]


def T(R, t):
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = R, np.ravel(t)
    return m


def main():
    cal = yaml.safe_load(open(sys.argv[1]))
    rclpy.init()
    node = rclpy.create_node("camera_tf")
    tf = Buffer()
    TransformListener(tf, node)
    child = cal["child"]
    while rclpy.ok() and not tf.can_transform("camera_base", child, rclpy.time.Time(), Duration(seconds=0)):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tf.lookup_transform("camera_base", child, rclpy.time.Time()).transform
    T_cb_rgb = T(quat_to_R([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]),
                 [t.translation.x, t.translation.y, t.translation.z])
    T_base_cb = T(quat_to_R(cal["quat_xyzw"]), cal["xyz"]) @ np.linalg.inv(T_cb_rgb)
    msg = TransformStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id, msg.child_frame_id = cal["parent"], "camera_base"
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = T_base_cb[:3, 3]
    q = R_to_quat(T_base_cb[:3, :3])
    msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = q
    StaticTransformBroadcaster(node).sendTransform(msg)
    print(f"published {cal['parent']} -> camera_base at {np.round(T_base_cb[:3, 3], 4)}", flush=True)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
