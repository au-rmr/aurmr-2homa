"""Live window: both MPC demo cameras side by side with the tags apriltag_ros detects (what the tracker gets).

    python3 tests/perception/mpc_tag_viewer.py [--cameras camera1 camera2] [--scale 0.5]

For placing cameras, cubes and calibration tags. Any family apriltag_ros knows (the Kinect tag_viewer.py uses
OpenCV's detector: 36h11 only, no tagStandard41h12). Per tag: id, side in px, decision margin (low = unreliable).
Top line per camera: ids seen; bottom bar: ids seen by both cameras. Images come JPEG-compressed (the cameras' host
may be another machine). Needs DISPLAY. Read-only. Esc or q closes it.
"""
import argparse
import time

import cv2
import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

GREEN, YELLOW, RED, WHITE = (0, 200, 0), (0, 220, 255), (0, 0, 255), (255, 255, 255)
LOW_MARGIN = 40.0   # decision margin below this: drawn yellow (detected, but marginal)


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


class Camera:
    def __init__(self, node, name):
        self.name, self.images, self.dets, self.last_img_t, self.shown = name, {}, {}, 0.0, None
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(CompressedImage, f"/{name}/color/image_raw/compressed", self.on_image, qos)
        node.create_subscription(AprilTagDetectionArray, f"/{name}/detections", self.on_dets, 10)

    def on_image(self, m):
        self.images[stamp_ns(m.header)] = m
        self.last_img_t = time.monotonic()
        while len(self.images) > 30:
            self.images.pop(min(self.images))

    def on_dets(self, m):
        self.dets[stamp_ns(m.header)] = m
        while len(self.dets) > 30:
            self.dets.pop(min(self.dets))

    def frame(self):
        """Newest image that has its detections (same stamp); the newest image alone if detection is off."""
        both = sorted(set(self.images) & set(self.dets))
        if both:
            t = both[-1]
            return self.images[t], self.dets[t]
        if self.images and not self.dets:
            return self.images[max(self.images)], None
        return None, None


def draw(cam, scale):
    img_msg, dets = cam.frame()
    if img_msg is None:
        blank = np.zeros((int(720 * scale), int(1280 * scale), 3), np.uint8)
        cv2.putText(blank, f"{cam.name}: no image", (20, 40), 0, 1.0, RED, 2)
        return blank, set()
    # BGR: compressed_image_transport encodes rgb8 as "rgb8; jpeg compressed bgr8".
    img = cv2.imdecode(np.frombuffer(bytes(img_msg.data), np.uint8), cv2.IMREAD_COLOR)
    small = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    seen = set()
    for d in (dets.detections if dets else []):
        c = np.array([[p.x, p.y] for p in d.corners]) * scale
        side = float(np.mean([np.linalg.norm(c[k] - c[(k + 1) % 4]) for k in range(4)])) / scale
        colour = GREEN if d.decision_margin >= LOW_MARGIN else YELLOW
        cv2.polylines(small, [c.astype(np.int32)], True, colour, 2)
        cv2.putText(small, f"{d.id} {side:.0f}px m{d.decision_margin:.0f}", tuple(c[0].astype(int) + [0, -8]),
                    0, 0.6, colour, 2)
        seen.add(int(d.id))
    status = f"{cam.name}: " + (f"tags {sorted(seen)}" if dets else "detection off")
    if time.monotonic() - cam.last_img_t > 1.0:
        status += "  STALE"
    cv2.putText(small, status, (12, 30), 0, 0.8, RED, 2)
    return small, seen


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--cameras", nargs="+", default=["camera1", "camera2"])
    p.add_argument("--scale", type=float, default=0.5)
    a = p.parse_args()
    rclpy.init()
    node = rclpy.create_node("mpc_tag_viewer")
    cams = [Camera(node, n) for n in a.cameras]
    cv2.namedWindow("mpc tags", cv2.WINDOW_NORMAL)
    next_draw, sized = 0.0, False
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.02)
        if time.monotonic() < next_draw:
            continue
        next_draw = time.monotonic() + 1 / 15
        panels, seen = zip(*(draw(c, a.scale) for c in cams))
        h = max(x.shape[0] for x in panels)
        row = np.hstack([cv2.copyMakeBorder(x, 0, h - x.shape[0], 0, 4, cv2.BORDER_CONSTANT) for x in panels])
        bar = np.zeros((40, row.shape[1], 3), np.uint8)
        common = sorted(set.intersection(*seen)) if seen else []
        cv2.putText(bar, f"seen by all cameras: {common}", (12, 28), 0, 0.8, WHITE, 2)
        frame = np.vstack([row, bar])
        if not sized:   # WINDOW_NORMAL opens tiny; start at the image's size (resizable after)
            cv2.resizeWindow("mpc tags", frame.shape[1], frame.shape[0])
            sized = True
        cv2.imshow("mpc tags", frame)
        if cv2.waitKey(1) & 0xFF in (27, ord("q")):
            break
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
