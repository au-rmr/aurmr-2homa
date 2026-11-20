// Mock arm hardware that switches modes like the real UR driver (ur_mock only; never on the arm).
//
// ros2_control's mock_components/GenericSystem (4.50.0) keeps a position command across a mode switch: back from
// streaming (velocity) to arm_controller (position), its next read() copied the PRE-stream position command into the
// state, and the mock arm jumped there for one cycle (s7_design.md 6b, 2026-11-19). The real driver does not:
// ur_robot_driver 3.10.0 hardware_interface.cpp:1510 sets the position commands to the measured joints whenever a
// position interface starts. This does the same, so the mock tests the stack, not a mock artefact; motion_server's
// jump watch stays strict and would still catch a driver that kept a stale command.
#include <string>
#include <vector>

#include "mock_components/generic_system.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace tahoma_control
{
class UrLikeMockSystem : public mock_components::GenericSystem
{
public:
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override
  {
    const std::string suffix = std::string("/") + hardware_interface::HW_IF_POSITION;
    for (const auto & key : start_interfaces) {
      if (key.size() > suffix.size() && key.compare(key.size() - suffix.size(), suffix.size(), suffix) == 0 &&
          has_command(key) && has_state(key))
      {
        set_command(key, get_state(key));
      }
    }
    return GenericSystem::perform_command_mode_switch(start_interfaces, stop_interfaces);
  }
};
}  // namespace tahoma_control

PLUGINLIB_EXPORT_CLASS(tahoma_control::UrLikeMockSystem, hardware_interface::SystemInterface)
