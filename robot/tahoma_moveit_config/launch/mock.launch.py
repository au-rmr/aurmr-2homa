"""The robot on mock hardware: robot.launch.py with hardware ur_mock unless told otherwise."""
import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    args = ["site_dir", "tool", "start_pose", "hardware", "speed_scaling_topic"]
    defaults = {"tool": "capcup", "start_pose": "home", "hardware": "ur_mock",
                "speed_scaling_topic": ""}
    return LaunchDescription(
        [DeclareLaunchArgument(a, **({"default_value": defaults[a]} if a in defaults else {}))
         for a in args] +
        [IncludeLaunchDescription(
            os.path.join(share("tahoma_moveit_config"), "launch", "robot.launch.py"),
            launch_arguments={a: LaunchConfiguration(a) for a in args}.items())])
