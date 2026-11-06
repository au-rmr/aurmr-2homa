"""D5 network test: stream camera-sized images between hosts and report the received rate.

Usage: image_rate.py pub|sub WIDTH HEIGHT RATE_HZ SECONDS
"""
import sys
import time

import rclpy
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image

mode, width, height, rate, seconds = sys.argv[1], *map(int, sys.argv[2:6])
rclpy.init()
node = rclpy.create_node(f"image_rate_{mode}")
qos = QoSProfile(depth=5)
size = width * height * 3

if mode == "pub":
    pub = node.create_publisher(Image, "/net_test/image", qos)
    msg = Image(width=width, height=height, encoding="rgb8", step=width * 3, data=bytes(size))
    time.sleep(3.0)  # discovery
    end, sent, next_t = time.time() + seconds, 0, time.time()
    while time.time() < end:
        pub.publish(msg)
        sent += 1
        next_t += 1.0 / rate
        time.sleep(max(0.0, next_t - time.time()))
    print(f"sent {sent} frames, {sent / seconds:.1f} Hz, {size * rate * 8 / 1e6:.0f} Mbit/s nominal")
else:
    stamps = []
    node.create_subscription(Image, "/net_test/image", lambda m: stamps.append(time.time()), qos)
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if len(stamps) > 1:
        gaps = [b - a for a, b in zip(stamps, stamps[1:])]
        print(f"received {len(stamps)} frames, {(len(stamps) - 1) / (stamps[-1] - stamps[0]):.1f} Hz, "
              f"max gap {max(gaps) * 1000:.0f} ms")
    else:
        print(f"received {len(stamps)} frames")
