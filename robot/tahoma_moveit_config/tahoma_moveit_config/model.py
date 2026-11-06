"""Builds the robot description and SRDF for a site and tool."""
import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory as share

from tahoma_moveit_config.srdf import ARM_JOINTS, compose


def _mock_hardware(initial):
    joints = "".join(
        f'<joint name="{name}"><command_interface name="position"/>'
        f'<state_interface name="position"><param name="initial_value">{value}</param>'
        '</state_interface><state_interface name="velocity"/></joint>'
        for name, value in zip(ARM_JOINTS, initial, strict=True))
    return ('<ros2_control name="mock_arm" type="system"><hardware>'
            "<plugin>mock_components/GenericSystem</plugin></hardware>"
            f"{joints}</ros2_control>")


def robot_description(site_dir, tool, mock_start=None):
    """mock_start: a named pose; adds mock hardware that starts there."""
    urdf = xacro.process_file(
        os.path.join(share("tahoma_description"), "urdf", "tahoma.urdf.xacro"),
        mappings={"kinematics_params": os.path.join(site_dir, "calibration", "arm_kinematics.yaml"),
                  "tool_xacro": os.path.join(share(tool), "urdf", "tool.xacro")}).toxml()
    if mock_start is None:
        return urdf
    with open(os.path.join(site_dir, "workcell.yaml")) as f:
        initial = yaml.safe_load(f)["named_poses"][mock_start]
    return urdf.replace("</robot>", _mock_hardware(initial) + "</robot>")


def robot_semantic(site_dir, tool):
    return compose(os.path.join(share("tahoma_moveit_config"), "config", "base.srdf"),
                   os.path.join(share(tool), "moveit", "collisions.srdf"),
                   os.path.join(site_dir, "workcell.yaml"))
