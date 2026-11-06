"""RViz + scene_viewer. Args: site_dir (absolute), tool, pose, scene."""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from tahoma_moveit_config.model import robot_description, robot_semantic


def setup(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    site, tool = get("site_dir"), get("tool")
    pkg = share("tahoma_moveit_config")
    urdf = robot_description(site, tool)
    srdf = robot_semantic(site, tool)
    workcell = os.path.join(site, "workcell.yaml")
    out = tempfile.mkdtemp()
    urdf_path, srdf_path = os.path.join(out, "m.urdf"), os.path.join(out, "m.srdf")
    with open(urdf_path, "w") as f:
        f.write(urdf)
    with open(srdf_path, "w") as f:
        f.write(srdf)
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf}]),
        Node(package="rviz2", executable="rviz2",
             arguments=["-d", os.path.join(pkg, "config", "view_scene.rviz")],
             on_exit=Shutdown()),
        ExecuteProcess(cmd=["python3", "-m", "tahoma_moveit_config.scene_viewer",
                            "--urdf", urdf_path, "--srdf", srdf_path, "--workcell", workcell,
                            "--pose", get("pose"), "--scene", get("scene")],
                       output="screen", on_exit=Shutdown()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("site_dir"),
        DeclareLaunchArgument("tool", default_value="capcup"),
        DeclareLaunchArgument("pose", default_value="drop_hide"),
        DeclareLaunchArgument("scene", default_value="limits_drop_hide"),
        OpaqueFunction(function=setup),
    ])
