"""Builds the robot description and SRDF for a site and tool."""
import os
import tempfile

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


def _named_pose(site_dir, name):
    with open(os.path.join(site_dir, "workcell.yaml")) as f:
        return yaml.safe_load(f)["named_poses"][name]


def robot_description(site_dir, tool, mock_start=None, hardware=None, robot_ip="0.0.0.0",
                      headless=False):
    """hardware: None (model only), "mock" (generic mock), "ur_mock" (UR driver's mock), "ur".

    mock_start: the named pose mock hardware starts at.
    """
    mappings = {"kinematics_params": os.path.join(site_dir, "calibration", "arm_kinematics.yaml"),
                "tool_xacro": os.path.join(share(tool), "urdf", "tool.xacro")}
    if hardware is None and mock_start is not None:
        hardware = "mock"   # older callers
    if hardware in ("ur", "ur_mock"):
        mappings.update(ur_control="true", robot_ip=robot_ip,
                        use_mock_hardware=str(hardware == "ur_mock").lower(),
                        headless_mode=str(headless).lower())
        if mock_start is not None:
            names = [n.removeprefix("arm_") for n in ARM_JOINTS]
            start = dict(zip(names, map(float, _named_pose(site_dir, mock_start)), strict=True))
            f = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
            yaml.safe_dump(start, f)
            f.close()
            mappings["initial_positions_file"] = f.name
    urdf = xacro.process_file(
        os.path.join(share("tahoma_description"), "urdf", "tahoma.urdf.xacro"),
        mappings=mappings).toxml()
    if hardware == "mock":
        initial = _named_pose(site_dir, mock_start or "home")
        urdf = urdf.replace("</robot>", _mock_hardware(initial) + "</robot>")
    return urdf


def robot_semantic(site_dir, tool):
    return compose(os.path.join(share("tahoma_moveit_config"), "config", "base.srdf"),
                   os.path.join(share(tool), "moveit", "collisions.srdf"),
                   os.path.join(site_dir, "workcell.yaml"))
