"""Run on another host of the robot network. Checks what that host can see of the control host:
- shared domain: only the expected nodes, and no topic or service named like ros2_control or the
  UR driver (controller, broadcaster, dashboard) unless it is one of motion_server's copies with
  motion_server its only publisher. motion_server copies force and speed scaling out under the
  driver's own names, so a name alone isn't a leak; who is at the other end is.
- private domain (142): nothing at all except the stranger itself.
Prints ISOLATED or LEAK with what it saw, and the whole graph as a GRAPH line for the log.
STRANGER_DUMP=1 prints the graph and exits 0 (to build the expected set from a real run)."""
import json
import os
import re
import sys
import time

import rclpy
from rclpy.context import Context

SHARED = int(os.environ.get("ROS_DOMAIN_ID", "0"))
PRIVATE = int(os.environ.get("AURMR_PRIVATE_DOMAIN", "142"))
# Nodes another host may see on the shared domain, from a real run's graph: the stack's own
# (MoveIt and tf add nodes with random suffixes), and the test's.
EXPECTED_NODES = [re.compile(p) for p in (
    r"/motion_server", r"/move_group", r"/move_group/moveit", r"/move_group_private_\d+",
    r"/moveit_\d+", r"/moveit_simple_controller_manager", r"/robot_state_publisher",
    r"/transform_listener_impl_[0-9a-f]+", r"/stranger",
    # test_network_cut.sh's own: Robot in the arm watcher, and the fake sensors.
    r"/robot_[0-9a-f]+", r"/fake_sensors",
)]
# move_group always loads MoveIt's controller manager plugin; with no controller_names it holds
# no controllers and offers only its parameter services, so it is not a way to the arm.
MOVEIT_PLUGIN = "/moveit_simple_controller_manager"
BASE_TOPICS = {"/rosout", "/parameter_events"}


def full(name, ns):
    return (ns.rstrip("/") + "/" + name) if ns != "/" else "/" + name


def suspect(name):
    if name == MOVEIT_PLUGIN or name.startswith(MOVEIT_PLUGIN + "/"):
        return False
    return any(w in name for w in ("controller", "broadcaster", "dashboard"))


def graph(node):
    nodes = sorted(full(n, ns) for n, ns in node.get_node_names_and_namespaces())
    topics = {}
    for t, _ in node.get_topic_names_and_types():
        topics[t] = {"pub": sorted(full(i.node_name, i.node_namespace)
                                   for i in node.get_publishers_info_by_topic(t)),
                     "sub": sorted(full(i.node_name, i.node_namespace)
                                   for i in node.get_subscriptions_info_by_topic(t))}
    services = sorted(s for s, _ in node.get_service_names_and_types())
    return {"nodes": nodes, "topics": topics, "services": services}


def look(domain, ready, least_s, most_s):
    """The graph once ready(graph) holds and at least least_s have passed, or at most_s."""
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = rclpy.create_node("stranger", context=context)
    start = time.monotonic()
    while True:
        time.sleep(0.5)
        g = graph(node)
        waited = time.monotonic() - start
        if (ready(g) and waited >= least_s) or waited >= most_s:
            break
    node.destroy_node()
    rclpy.shutdown(context=context)
    return g, waited


shared_ok = lambda g: "/motion_server/connect" in g["services"]
shared, waited = look(SHARED, shared_ok, 4.0, 30.0)
# Nothing should ever show on the private side, so give it at least as long as the shared domain
# took to show motion_server.
private, _ = look(PRIVATE, lambda g: False, 0.0, max(waited, 4.0) + 2.0)
print("GRAPH " + json.dumps({SHARED: shared, PRIVATE: private}, sort_keys=True), flush=True)
if os.environ.get("STRANGER_DUMP"):
    sys.exit(0)
if not shared_ok(shared):
    print(f"CONTROL FAILED: motion_server not visible on the shared domain {SHARED}", flush=True)
    sys.exit(1)

leaks = []
for n in shared["nodes"]:
    if not any(p.fullmatch(n) for p in EXPECTED_NODES):
        leaks.append(f"{SHARED}: unexpected node {n}")
for t, ends in shared["topics"].items():
    if not suspect(t):
        continue
    if "/motion_server" in ends["pub"]:
        # One of motion_server's copies: anyone may read it, nobody else may publish it.
        others = [n for n in ends["pub"] if n != "/motion_server"]
    else:
        # Not a copy, so any end of it is ros2_control's or the driver's.
        others = ends["pub"] + ends["sub"]
    if others:
        leaks.append(f"{SHARED}: {t} has endpoints besides motion_server: {sorted(set(others))}")
for s in shared["services"]:
    if suspect(s):
        leaks.append(f"{SHARED}: service {s}")
for n in private["nodes"]:
    if n != "/stranger":
        leaks.append(f"{PRIVATE}: node {n}")
for t in private["topics"]:
    if t not in BASE_TOPICS:
        leaks.append(f"{PRIVATE}: topic {t}")
for s in private["services"]:
    if not s.startswith("/stranger/"):
        leaks.append(f"{PRIVATE}: service {s}")
print("ISOLATED" if not leaks else "LEAK " + "; ".join(leaks), flush=True)
sys.exit(0 if not leaks else 1)
