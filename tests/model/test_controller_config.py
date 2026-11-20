"""Controller settings that keep the arm from moving when controllers switch (S7, test agent T30).

The streaming session ends by switching streaming_velocity_controller -> arm_controller. If the trajectory controller
took its last *command* as its state on activation, it would hold the pose from before the session and drive the arm
back (the dev's S7 finding, 2026-11-18). On the mock that is a jump; on the arm a fast move. motion_server's jump check
only reports it afterwards, so this file is the guard that runs before any arm."""
import os

import yaml

CONFIG = os.path.join(os.path.dirname(__file__), "..", "..", "robot", "tahoma_moveit_config", "config",
                      "ur_controllers.yaml")


def params(name):
    with open(CONFIG) as f:
        return yaml.safe_load(f)[name]["ros__parameters"]


def test_trajectory_controller_holds_the_measured_pose_on_activation():
    arm = params("arm_controller")
    # Jazzy's default is true: it must be set, and set to false.
    assert arm.get("set_last_command_interface_value_as_state_on_activation") is False
    # Same effect through the other door: interpolating from the last command instead of the state.
    assert not arm.get("interpolate_from_desired_state", False)
    assert not arm.get("open_loop_control", False)


def test_streaming_controller_ceilings_are_set_and_small():
    s = params("streaming_velocity_controller")
    assert 0 < s["stale_timeout_s"] <= 0.15
    assert 0 < s["max_velocity"] <= 0.25
    assert 0 < s["max_acceleration"] <= 1.0
    assert s["limit_margin_rad"] >= 0.02
