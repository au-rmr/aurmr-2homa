"""Saves one synchronized camera frame (colour, depth aligned to colour, camera info) to an .npz.

Usage (in a container on the camera's ROS domain):
  python3 tests/perception/capture_frames.py <out.npz> [--camera /camera_lower_right] [--timeout 10]
  python3 tests/perception/capture_frames.py <out.npz> --camera /camera1 --realsense    # colour only
The file holds rgb (H x W x 4 BGRA for the Kinect, H x W x 3 rgb8 for the RealSense, as the driver publishes it),
depth (H x W uint16, mm; Kinect only), K (3x3), D, distortion_model, stamp. Used to feed the same frames to the old
and new bin trackers, and by the camera calibration (tests/arm/handeye_pose.sh).
"""
import argparse
import sys

import numpy as np
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


def to_array(msg):
    dtype = {"bgra8": np.uint8, "bgr8": np.uint8, "rgb8": np.uint8, "16UC1": np.uint16, "mono16": np.uint16}[msg.encoding]
    a = np.frombuffer(bytes(msg.data), dtype=dtype)
    return a.reshape(msg.height, msg.width, -1).squeeze() if a.size != msg.height * msg.width else a.reshape(msg.height, msg.width)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("out")
    p.add_argument("--camera", default="/camera_lower_right")
    p.add_argument("--timeout", type=float, default=10.0)
    p.add_argument("--realsense", action="store_true", help="realsense2_camera topics, colour only (MPC demo cameras)")
    a = p.parse_args()
    rclpy.init()
    node = rclpy.create_node("capture_frames")
    qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
    got = {}
    if a.realsense:
        subs = [Subscriber(node, Image, f"{a.camera}/color/image_raw", qos_profile=qos),
                Subscriber(node, CameraInfo, f"{a.camera}/color/camera_info", qos_profile=qos)]
        sync = ApproximateTimeSynchronizer(subs, queue_size=5, slop=0.02)
        sync.registerCallback(lambda rgb, info: got.setdefault("frame", (rgb, None, info)))
    else:
        subs = [Subscriber(node, Image, f"{a.camera}/rgb/image_raw", qos_profile=qos),
                Subscriber(node, Image, f"{a.camera}/depth_to_rgb/image_raw", qos_profile=qos),
                Subscriber(node, CameraInfo, f"{a.camera}/rgb/camera_info", qos_profile=qos)]
        sync = ApproximateTimeSynchronizer(subs, queue_size=5, slop=0.02)
        sync.registerCallback(lambda rgb, depth, info: got.setdefault("frame", (rgb, depth, info)))
    end = node.get_clock().now().nanoseconds + a.timeout * 1e9
    while "frame" not in got and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if "frame" not in got:
        print(f"no synchronized frame on {a.camera} within {a.timeout} s", file=sys.stderr)
        sys.exit(1)
    rgb, depth, info = got["frame"]
    extra = {} if depth is None else {"depth": to_array(depth), "depth_encoding": depth.encoding}
    np.savez(a.out, rgb=to_array(rgb), K=np.array(info.k).reshape(3, 3), D=np.array(info.d),
             distortion_model=info.distortion_model, stamp=rgb.header.stamp.sec + rgb.header.stamp.nanosec * 1e-9,
             rgb_encoding=rgb.encoding, frame_id=rgb.header.frame_id, **extra)
    print(f"saved {a.out}: rgb {rgb.width}x{rgb.height} {rgb.encoding}, depth {extra.get('depth_encoding', 'none')}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
