"""Robot on mock hardware with MoveIt. Args: site_dir (absolute), tool, start_pose."""
import os

import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from tahoma_moveit_config.model import robot_description, robot_semantic


def config(name):
    with open(os.path.join(share("tahoma_moveit_config"), "config", name)) as f:
        return yaml.safe_load(f)


def setup(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    site, tool = get("site_dir"), get("tool")
    urdf = robot_description(site, tool, mock_start=get("start_pose"))
    description = {"robot_description": urdf}
    controllers = os.path.join(share("tahoma_moveit_config"), "config", "ros2_controllers.yaml")
    spawner = lambda name: Node(package="controller_manager", executable="spawner", arguments=[name])
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[description]),
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[controllers], remappings=[("~/robot_description", "/robot_description")]),
        spawner("joint_state_broadcaster"),
        spawner("arm_controller"),
        Node(package="moveit_ros_move_group", executable="move_group", output="screen",
             parameters=[description,
                         {"robot_description_semantic": robot_semantic(site, tool),
                          "robot_description_kinematics": config("kinematics.yaml"),
                          "robot_description_planning": config("joint_limits.yaml"),
                          "planning_pipelines": ["ompl"],
                          "default_planning_pipeline": "ompl",
                          "ompl": config("ompl_planning.yaml"),
                          "publish_robot_description_semantic": True},
                         config("moveit_controllers.yaml")]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("site_dir"),
        DeclareLaunchArgument("tool", default_value="capcup"),
        DeclareLaunchArgument("start_pose", default_value="home"),
        OpaqueFunction(function=setup),
    ])
