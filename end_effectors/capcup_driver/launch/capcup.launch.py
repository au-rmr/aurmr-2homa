"""Capcup drivers. part: compute (cap board: bridge, contact, haptic search; on the host the
board streams to), control (vacuum ejector; on the host it is wired to), or all.
site_dir: reads sites/<site>/tools/capcup.yaml. fake_vacuum: no ejector needed (tests)."""
import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    get = lambda n: LaunchConfiguration(n).perform(context)
    part = get("part")
    with open(os.path.join(get("site_dir"), "tools", "capcup.yaml")) as f:
        site = yaml.safe_load(f)
    nodes = []
    if part in ("compute", "all"):
        nodes += [
            Node(package="capcup_driver", executable="cap_bridge",
                 parameters=[{"port": site["cap_board_port"]}]),
            Node(package="capcup_driver", executable="contact_detector"),
            Node(package="capcup_driver", executable="haptic_search",
                 parameters=[{"calibration": os.path.join(get("site_dir"), "tools", "capcup_calibration.yaml")
                              if site.get("search_gains", False) else ""}]),
        ]
    if part in ("control", "all"):
        nodes.append(Node(package="capcup_driver", executable="vacuum", name="vacuum",
                          parameters=[{"ip": site["ejector_ip"], "seal_mbar": -1.0 if site["seal_mbar"] == "device" else float(site["seal_mbar"]),
                                       "fake": get("fake_vacuum") == "true",
                                       "fake_item_topic": get("fake_item_topic")}]))
    if not nodes:
        raise RuntimeError(f"part must be compute, control or all, not {part}")
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("site_dir"),
        DeclareLaunchArgument("part", default_value="all"),
        DeclareLaunchArgument("fake_vacuum", default_value="false"),
        DeclareLaunchArgument("fake_item_topic", default_value=""),
        OpaqueFunction(function=setup),
    ])
