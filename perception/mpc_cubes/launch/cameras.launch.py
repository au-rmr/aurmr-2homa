"""The MPC/MCTS demo's RealSense cameras, plus AprilTag detection per camera when the site enables it.

    ros2 launch perception/mpc_cubes/launch/cameras.launch.py site_dir:=sites/uw_lab
        [tags:=view] [family:=36h11] [size:=0.036]

tags:=view turns detection on whatever the site says, for tests/perception/mpc_tag_viewer.py (placing cameras,
cubes, calibration tags); without a size the poses are not to scale (ids and corners are right). family/size
override the site's (36h11 = the arm's calibration tags).

Topics per camera N (namespace /cameraN): color/image_raw, color/camera_info (factory intrinsics), and with tags
on: detections (apriltag_msgs/AprilTagDetectionArray) + TF cameraN_color_optical_frame -> tag. Settings:
<site_dir>/mpc_cameras.yaml.
"""
import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def nodes(context):
    site_dir = LaunchConfiguration("site_dir").perform(context)
    with open(os.path.join(site_dir, "mpc_cameras.yaml")) as f:
        cfg = yaml.safe_load(f)
    tags = dict(cfg.get("tags") or {})
    for key in ("family", "size"):
        if LaunchConfiguration(key).perform(context):
            tags[key] = LaunchConfiguration(key).perform(context)
    if LaunchConfiguration("tags").perform(context) == "view":
        tags["enabled"] = True
        tags.setdefault("size", None)
        if not tags["size"]:
            tags["size"] = 1.0
            print("tags:=view without a size: tag poses NOT to scale")
    if tags.get("enabled") and not tags.get("size"):
        raise RuntimeError("mpc_cameras.yaml: tags.enabled needs tags.size (the black square's edge, m)")
    containers = []
    for name, cam in cfg["cameras"].items():
        parts = [ComposableNode(
            package="realsense2_camera", plugin="realsense2_camera::RealSenseNodeFactory",
            name=name, namespace="",
            parameters=[{"serial_no": f"_{cam['serial']}",   # leading _ keeps it a string
                         "camera_name": name,
                         "enable_color": True, "enable_depth": False,
                         "enable_infra1": False, "enable_infra2": False,
                         "enable_gyro": False, "enable_accel": False,
                         "rgb_camera.color_profile": cfg["color_profile"],
                         "initial_reset": True}])]
        if tags.get("enabled"):
            parts.append(ComposableNode(
                package="apriltag_ros", plugin="AprilTagNode", name="apriltag", namespace=name,
                remappings=[("image_rect", "color/image_raw"), ("camera_info", "color/camera_info")],
                parameters=[{"family": tags["family"], "size": float(tags["size"]),
                             "max_hamming": 0, "detector.threads": 2,
                             "detector.decimate": float(tags.get("decimate", 2.0)),
                             "detector.blur": float(tags.get("blur", 0.0))}],
                extra_arguments=[{"use_intra_process_comms": True}]))
        # One process per camera: a camera's driver crash does not take the other down.
        containers.append(ComposableNodeContainer(
            name=f"{name}_container", namespace="", package="rclcpp_components",
            executable="component_container", composable_node_descriptions=parts, output="screen"))
    return containers


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("site_dir"),
        DeclareLaunchArgument("tags", default_value="", description="view = detection on (viewer preset)"),
        DeclareLaunchArgument("family", default_value=""),
        DeclareLaunchArgument("size", default_value=""),
        OpaqueFunction(function=nodes),
    ])
