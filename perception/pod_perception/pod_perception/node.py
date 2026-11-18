"""Pod perception node: tracks which item is in which bin (bin_tracker + uois) and finds suction grasps (optigrasp).

ROS 2 port of the ROS 1 aurmr_perception pod_model.py (PodPerceptionROS + DiffPodModel) and the RGB path of
grasp_detection.py. Services (in the node's namespace):
  ~/capture_empty_pod  std_srvs/Trigger         empty-pod reference; bin rectangles from the pod model + camera TF
  ~/stow ~/pick ~/update ~/reset_bin  perception_interfaces/BinItem
  ~/get_item           perception_interfaces/GetItem       mask + points of an item
  ~/detect_grasp       perception_interfaces/DetectGrasp   OptiGrasp on the item's bin crop
Two ROS domains in one process: the camera's images (<camera>/rgb/image_raw bgra8, <camera>/depth_to_rgb/image_raw
16UC1 mm, <camera>/rgb/camera_info; ~300 MB/s at 3072P) arrive on the camera domain (camera_domain, loopback only, see
docker/cyclonedds.xml); services, TF and everything else are on the robot domain (ROS_DOMAIN_ID). On the robot domain
the node publishes the site calibration as a static TF base_link -> rgb_camera_link (the driver's own camera_base tree
stays on the camera domain, so the two cannot clash).
Parameters: camera, camera_domain, calibration (path), workcell (path), pod, uois_weights, optigrasp_weights, device,
state_dir ("" = no files).
"""
import threading

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PointStamped, TransformStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from perception_interfaces.srv import BinItem, DetectGrasp, GetItem
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.context import Context
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

from .bin_bounds import bin_bounds, matrix
from .geometry import backproject, image_to_numpy, transform_points

CAMERA_FRAME = "rgb_camera_link"


class PodPerception(Node):
    def __init__(self):
        super().__init__("pod_perception")
        p = {k: self.declare_parameter(k, v).value for k, v in {
            "camera": "/camera_lower_right", "camera_domain": 77, "calibration": "", "workcell": "",
            "pod": "pod_bins", "uois_weights": "", "optigrasp_weights": "", "device": "cuda", "state_dir": ""}.items()}
        self.p = p
        self.workcell = yaml.safe_load(open(p["workcell"]))
        cal = yaml.safe_load(open(p["calibration"]))
        if cal["parent"] != "base_link" or cal["child"] != CAMERA_FRAME:
            raise ValueError(f"calibration must be base_link -> {CAMERA_FRAME}, is {cal['parent']} -> {cal['child']}")
        self.T_base_cam = matrix(cal["xyz"], quat_xyzw=cal["quat_xyzw"])
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = "base_link", CAMERA_FRAME
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = cal["xyz"]
        (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,
         t.transform.rotation.w) = cal["quat_xyzw"]
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform(t)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.lock = threading.Lock()
        self.latest = None              # (bgr, depth_mm, K, D, stamp)
        self.tracker = None
        self.bounds = {}
        self.not_seen = set()           # items whose stow saw no depth change
        self.segmenter = self.grasp_model = None

        # Camera side: its own context on the camera domain, spun by main().
        cam, qos = p["camera"], QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.camera_context = Context()
        rclpy.init(context=self.camera_context, domain_id=int(p["camera_domain"]))
        self.camera_node = rclpy.create_node("pod_perception_camera", context=self.camera_context)
        subs = [Subscriber(self.camera_node, Image, f"{cam}/rgb/image_raw", qos_profile=qos),
                Subscriber(self.camera_node, Image, f"{cam}/depth_to_rgb/image_raw", qos_profile=qos),
                Subscriber(self.camera_node, CameraInfo, f"{cam}/rgb/camera_info", qos_profile=qos)]
        self.sync = ApproximateTimeSynchronizer(subs, 5, 0.05)
        self.sync.registerCallback(self.on_images)

        services = MutuallyExclusiveCallbackGroup()   # one request at a time: the models are not shared
        for name, srv, cb in [("capture_empty_pod", Trigger, self.on_empty_pod), ("stow", BinItem, self.on_stow),
                              ("pick", BinItem, self.on_pick), ("update", BinItem, self.on_update),
                              ("reset_bin", BinItem, self.on_reset), ("get_item", GetItem, self.on_get_item),
                              ("detect_grasp", DetectGrasp, self.on_detect_grasp)]:
            self.create_service(srv, f"~/{name}", cb, callback_group=services)
        self.get_logger().info(f"waiting for {cam} images on domain {p['camera_domain']}")

    # ---- inputs ---------------------------------------------------------------------------------------------------
    def on_images(self, rgb, depth, info):
        bgr = image_to_numpy(rgb)[..., :3]
        d = image_to_numpy(depth).astype(np.float32)
        with self.lock:
            first = self.latest is None
            self.latest = (bgr, d, np.array(info.k).reshape(3, 3), np.array(info.d), rgb.header.stamp)
        if first:
            self.get_logger().info(f"images {bgr.shape[1]}x{bgr.shape[0]}, distortion {info.distortion_model}")

    def capture(self):
        with self.lock:
            if self.latest is None:
                raise RuntimeError("no camera images yet")
            return self.latest

    def camera_pose(self, frame="base_link"):
        """frame -> rgb_camera_link: the calibration for base_link, TF on the robot domain for other frames."""
        if frame == "base_link":
            return self.T_base_cam
        t = self.tf.lookup_transform(frame, "base_link", rclpy.time.Time(), Duration(seconds=1)).transform
        return matrix([t.translation.x, t.translation.y, t.translation.z],
                      quat_xyzw=[t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]) @ self.T_base_cam

    def models(self):
        if self.segmenter is None:
            from uois import Segmenter
            import optigrasp
            self.segmenter = Segmenter(self.p["uois_weights"], device=self.p["device"])
            self.grasp_model = optigrasp.load_model(self.p["optigrasp_weights"], device=self.p["device"])
        return self.segmenter, self.grasp_model

    # ---- services -------------------------------------------------------------------------------------------------
    def on_empty_pod(self, req, res):
        try:
            from bin_tracker import BinTracker
            bgr, depth, K, D, _ = self.capture()
            pod = self.workcell["pods"][self.p["pod"]]
            f = self.workcell["frames"][pod["frame"]]
            if f["parent"] != "base_link":
                raise RuntimeError(f"pod frame {pod['frame']} must hang off base_link")
            T_pod = matrix(f["xyz"], rpy=f["rpy"])
            self.bounds = bin_bounds(pod, T_pod, self.camera_pose(), K, D, (bgr.shape[1], bgr.shape[0]))
            seg, _ = self.models()
            self.tracker = BinTracker(self.bounds, seg.segment, depth, state_dir=self.p["state_dir"] or None)
            self.not_seen.clear()
            res.success, res.message = True, f"empty pod captured; {len(self.bounds)} bins in view: " + \
                " ".join(sorted(self.bounds, key=lambda b: (b[-1], int(b[:-1]))))
        except Exception as e:                          # noqa: BLE001  (reported to the caller)
            res.success, res.message = False, f"{type(e).__name__}: {e}"
        self.get_logger().info(res.message)
        return res

    def _bin_op(self, res, fn, label):
        try:
            if self.tracker is None:
                raise RuntimeError("capture_empty_pod first")
            st = fn()
            res.success, res.status, res.message = int(st) == 0, st.name, f"{label}: {st.name}"
        except Exception as e:                          # noqa: BLE001
            res.success, res.status, res.message = False, "ERROR", f"{label}: {type(e).__name__}: {e}"
        self.get_logger().info(res.message)
        return res

    def on_stow(self, req, res):
        def run():
            if req.bin_id not in self.bounds:
                raise KeyError(f"bin {req.bin_id} not in view")
            bgr, depth, *_ = self.capture()
            st = self.tracker.stow(req.bin_id, req.item_id, bgr, depth)
            if st.name == "NO_OBJ_STORED":
                self.not_seen.add(req.item_id)
            else:
                self.not_seen.discard(req.item_id)
            return st
        return self._bin_op(res, run, f"stow {req.item_id} in {req.bin_id}")

    def on_pick(self, req, res):
        bgr_depth = lambda: self.capture()[:2]          # noqa: E731
        return self._bin_op(res, lambda: self.tracker.pick(req.item_id, *bgr_depth()), f"pick {req.item_id}")

    def on_update(self, req, res):
        return self._bin_op(res, lambda: self.tracker.update(req.bin_id, *self.capture()[:2]), f"update {req.bin_id}")

    def on_reset(self, req, res):
        return self._bin_op(res, lambda: self.tracker.reset_bin(req.bin_id), f"reset {req.bin_id}")

    def _item(self, item_id):
        if self.tracker is None:
            raise RuntimeError("capture_empty_pod first")
        if item_id in self.not_seen:
            raise LookupError(f"{item_id} was not seen when stowed (occluded or never placed)")
        if item_id not in self.tracker.items:
            raise LookupError(f"{item_id} is not in the pod")
        bin_id = self.tracker.items[item_id][0]
        mask = self.tracker.get_obj_mask(item_id)
        if mask is None:
            raise LookupError(f"bin {bin_id} lost track of its items (bad bin); reset it")
        return bin_id, mask

    def on_get_item(self, req, res):
        try:
            bin_id, mask = self._item(req.item_id)
            res.bin_id = bin_id
            res.mask = Image(height=mask.shape[0], width=mask.shape[1], encoding="mono8", step=mask.shape[1],
                             data=mask.tobytes())
            res.mask.header.frame_id = CAMERA_FRAME
            if not req.mask_only:
                b = self.tracker.bins[bin_id]
                _, _, K, D, stamp = self.capture()
                r1, _, c1, _ = b.bounds
                pts = backproject(b.current["depth"], b.crop(mask) > 0, K, D, offset=(c1, r1))
                frame = req.frame_id or CAMERA_FRAME
                if frame != CAMERA_FRAME:
                    pts = transform_points(self.camera_pose(frame), pts)
                res.points = point_cloud2.create_cloud_xyz32(res.mask.header, pts.astype(np.float32).tolist())
                res.points.header.frame_id = frame
            res.success, res.message = True, f"{req.item_id} in {bin_id}: {int(mask.sum())} px"
        except Exception as e:                          # noqa: BLE001
            res.success, res.message = False, f"{type(e).__name__}: {e}"
        return res

    def on_detect_grasp(self, req, res):
        try:
            import optigrasp
            bin_id, mask = self._item(req.item_id)
            bgr, depth, K, D, stamp = self.capture()
            r1, r2, c1, c2 = self.bounds[bin_id]
            _, model = self.models()
            g = optigrasp.predict(model, bgr[r1:r2, c1:c2], mask[r1:r2, c1:c2], device=self.p["device"])
            u, v = int(g.pixel[0]) + c1, int(g.pixel[1]) + r1
            p = backproject(depth[v:v + 1, u:u + 1], np.ones((1, 1), bool), K, D, offset=(u, v))
            if len(p) == 0:
                raise RuntimeError(f"no depth at the grasp pixel ({u}, {v})")
            frame = req.frame_id or CAMERA_FRAME
            if frame != CAMERA_FRAME:
                p = transform_points(self.camera_pose(frame), p)
            res.point = PointStamped()
            res.point.header.frame_id, res.point.header.stamp = frame, stamp
            res.point.point.x, res.point.point.y, res.point.point.z = (float(x) for x in p[0])
            res.bin_id, res.roll, res.pitch, res.pixel_x, res.pixel_y = bin_id, g.roll, g.pitch, u, v
            res.success, res.message = True, f"grasp on {req.item_id} in {bin_id} at pixel ({u}, {v})"
        except Exception as e:                          # noqa: BLE001
            res.success, res.message = False, f"{type(e).__name__}: {e}"
        self.get_logger().info(res.message)
        return res


def main():
    rclpy.init()
    node = PodPerception()
    cam_ex = SingleThreadedExecutor(context=node.camera_context)
    cam_ex.add_node(node.camera_node)
    threading.Thread(target=cam_ex.spin, daemon=True).start()
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.camera_node.destroy_node()
        rclpy.try_shutdown(context=node.camera_context)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
