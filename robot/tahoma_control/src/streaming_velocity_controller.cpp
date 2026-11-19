// Last line of defence for streamed joint velocities (S7). Runs inside controller_manager's loop.
//
// Its only input is motion_server, on the private domain (~/commands, one velocity per joint in
// `joints` order). motion_server holds the session rules (guard, force, per-session limits); this
// controller holds the rules that must survive motion_server hanging:
// - a command older than stale_timeout_s, or none yet: ramp to zero (recovers when fresh ones come);
// - a command that is not finite, has the wrong size or is above max_velocity: ramp to zero and
//   stay there until the controller is activated again (no clamping: a bad command means a bad sender);
// - a joint within limit_margin_rad of its position limit, moving towards it: same latch;
// - every change of velocity limited to max_acceleration.
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace tahoma_control
{
namespace
{
struct Command
{
  std::vector<double> velocity;
  rclcpp::Time received;
};
}  // namespace

class StreamingVelocityController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override
  {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<double>("stale_timeout_s", 0.15);
    auto_declare<double>("max_velocity", 0.1);
    auto_declare<double>("max_acceleration", 0.5);
    auto_declare<double>("limit_margin_rad", 0.05);
    auto_declare<std::vector<double>>("position_lower", {});
    auto_declare<std::vector<double>>("position_upper", {});
    return CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration c;
    c.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & j : joints_) {c.names.push_back(j + "/velocity");}
    return c;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration c;
    c.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & j : joints_) {c.names.push_back(j + "/position");}
    return c;
  }

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    auto node = get_node();
    joints_ = node->get_parameter("joints").as_string_array();
    stale_ = node->get_parameter("stale_timeout_s").as_double();
    max_vel_ = node->get_parameter("max_velocity").as_double();
    max_acc_ = node->get_parameter("max_acceleration").as_double();
    margin_ = node->get_parameter("limit_margin_rad").as_double();
    lower_ = node->get_parameter("position_lower").as_double_array();
    upper_ = node->get_parameter("position_upper").as_double_array();
    const auto n = joints_.size();
    if (n == 0 || lower_.size() != n || upper_.size() != n) {
      RCLCPP_ERROR(node->get_logger(), "joints, position_lower and position_upper must match");
      return CallbackReturn::ERROR;
    }
    for (const double v : {stale_, max_vel_, max_acc_}) {
      if (!(v > 0) || !std::isfinite(v)) {
        RCLCPP_ERROR(node->get_logger(), "stale_timeout_s, max_velocity, max_acceleration: > 0");
        return CallbackReturn::ERROR;
      }
    }
    if (!(margin_ >= 0) || !std::isfinite(margin_)) {
      RCLCPP_ERROR(node->get_logger(), "limit_margin_rad must be >= 0");
      return CallbackReturn::ERROR;
    }
    sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/commands", rclcpp::SystemDefaultsQoS(),
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr m) {
        Command c{m->data, get_node()->now()};
        buffer_.writeFromNonRT(c);
      });
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    // Only commands that arrive after activation count.
    buffer_.writeFromNonRT(Command{{}, rclcpp::Time(0, 0, get_node()->get_clock()->get_clock_type())});
    current_.assign(joints_.size(), 0.0);
    latched_ = false;
    for (auto & ci : command_interfaces_) {(void)ci.set_value(0.0);}
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    for (auto & ci : command_interfaces_) {(void)ci.set_value(0.0);}
    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override
  {
    const auto n = joints_.size();
    std::vector<double> target(n, 0.0);
    const Command * cmd = buffer_.readFromRT();
    const bool fresh = cmd && cmd->received.nanoseconds() > 0 &&
      (time - cmd->received).seconds() <= stale_;
    if (!latched_ && fresh) {
      bool ok = cmd->velocity.size() == n;
      for (size_t i = 0; ok && i < n; ++i) {
        const double v = cmd->velocity[i];
        ok = std::isfinite(v) && std::abs(v) <= max_vel_;
      }
      if (ok) {
        target = cmd->velocity;
      } else {
        latch("command not finite, wrong size or above max_velocity");
      }
    }
    for (size_t i = 0; i < n && !latched_; ++i) {
      const auto q = state_interfaces_[i].get_optional();
      const double pos = q ? *q : std::numeric_limits<double>::quiet_NaN();
      if (!std::isfinite(pos)) {latch("joint position not available"); break;}
      const bool towards_low = pos <= lower_[i] + margin_ && (target[i] < 0 || current_[i] < 0);
      const bool towards_high = pos >= upper_[i] - margin_ && (target[i] > 0 || current_[i] > 0);
      if (towards_low || towards_high) {latch("joint " + joints_[i] + " at its position limit");}
    }
    if (latched_) {std::fill(target.begin(), target.end(), 0.0);}
    const double step = max_acc_ * std::max(period.seconds(), 0.0);
    for (size_t i = 0; i < n; ++i) {
      current_[i] += std::clamp(target[i] - current_[i], -step, step);
      (void)command_interfaces_[i].set_value(current_[i]);
    }
    return controller_interface::return_type::OK;
  }

private:
  void latch(const std::string & why)
  {
    if (!latched_) {RCLCPP_ERROR(get_node()->get_logger(), "stopping until re-activated: %s", why.c_str());}
    latched_ = true;
  }

  std::vector<std::string> joints_;
  double stale_{0.15}, max_vel_{0.1}, max_acc_{0.5}, margin_{0.05};
  std::vector<double> lower_, upper_, current_;
  bool latched_{false};
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
  realtime_tools::RealtimeBuffer<Command> buffer_;
};
}  // namespace tahoma_control

PLUGINLIB_EXPORT_CLASS(tahoma_control::StreamingVelocityController, controller_interface::ControllerInterface)
