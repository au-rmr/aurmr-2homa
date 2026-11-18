"""Client for the pod perception node's services (perception/pod_perception)."""
import rclpy
from perception_interfaces.srv import BinItem, DetectGrasp, GetItem, ListItems
from std_srvs.srv import Trigger

NODE = "/pod_perception"


class Perception:
    def __init__(self, node, timeout_s=300.0):
        self.node, self.timeout_s = node, timeout_s
        self.clients = {}

    def _call(self, srv, name, **fields):
        c = self.clients.get(name)
        if c is None:
            c = self.clients[name] = self.node.create_client(srv, f"{NODE}/{name}")
        if not c.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f"{NODE}/{name} is not available: is the perception profile up?")
        req = srv.Request()
        for k, v in fields.items():
            setattr(req, k, v)
        f = c.call_async(req)
        rclpy.spin_until_future_complete(self.node, f, timeout_sec=self.timeout_s)
        if not f.done():
            raise TimeoutError(f"{NODE}/{name} did not answer in {self.timeout_s:.0f} s")
        return f.result()

    def empty_pod(self):
        return self._call(Trigger, "capture_empty_pod")

    def stow(self, bin_id, item_id):
        return self._call(BinItem, "stow", bin_id=bin_id, item_id=item_id)

    def taken(self, item_id):
        return self._call(BinItem, "pick", item_id=item_id)

    def update(self, bin_id):
        return self._call(BinItem, "update", bin_id=bin_id)

    def reset_bin(self, bin_id):
        return self._call(BinItem, "reset_bin", bin_id=bin_id)

    def where(self, item_id):
        return self._call(GetItem, "get_item", item_id=item_id, frame_id="base_link", mask_only=False)

    def grasp(self, item_id, frame_id="base_link"):
        return self._call(DetectGrasp, "detect_grasp", item_id=item_id, frame_id=frame_id)

    def items(self):
        return self._call(ListItems, "list_items")
