"""Image and point helpers for the pod perception node (numpy + OpenCV, no ROS imports beyond message fields)."""
import cv2
import numpy as np

_DTYPES = {"bgra8": (np.uint8, 4), "bgr8": (np.uint8, 3), "rgb8": (np.uint8, 3), "rgba8": (np.uint8, 4),
           "mono8": (np.uint8, 1), "16UC1": (np.uint16, 1), "mono16": (np.uint16, 1), "32FC1": (np.float32, 1)}


def image_to_numpy(msg):
    """sensor_msgs/Image -> H x W [x C] array (no copy of the channel order: bgra8 stays B, G, R, A)."""
    dtype, ch = _DTYPES[msg.encoding]
    a = np.frombuffer(bytes(msg.data), dtype=dtype)
    if msg.is_bigendian != (np.dtype(dtype).byteorder == ">"):
        a = a.byteswap() if np.dtype(dtype).itemsize > 1 else a
    row = msg.step // np.dtype(dtype).itemsize
    a = a.reshape(msg.height, row)[:, :msg.width * ch]
    return a.reshape(msg.height, msg.width, ch) if ch > 1 else a.reshape(msg.height, msg.width)


def backproject(depth_mm, mask, K, D, offset=(0, 0)):
    """3D points (m, camera optical frame) of the masked pixels with depth > 0. depth_mm and mask are a crop whose
    top-left pixel is `offset` (x, y) in the full image; K, D are the full image's intrinsics and distortion."""
    v, u = np.nonzero(mask & (depth_mm > 0))
    if len(u) == 0:
        return np.zeros((0, 3))
    z = depth_mm[v, u].astype(np.float64) / 1000.0
    px = np.stack([u + offset[0], v + offset[1]], axis=1).astype(np.float64).reshape(-1, 1, 2)
    xy = cv2.undistortPoints(px, np.asarray(K, float), np.asarray(D, float)).reshape(-1, 2)
    return np.c_[xy * z[:, None], z]


def transform_points(T, pts):
    """Apply a 4x4 transform to N x 3 points."""
    pts = np.asarray(pts, float)
    return pts @ T[:3, :3].T + T[:3, 3]
