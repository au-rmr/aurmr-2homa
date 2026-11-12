// The one path to the arm. MoveIt plans; this runs the plan and stops it when force, a stop
// signal, a stale sensor or a lost demo says so.
//
// With private_domain_id set, the controllers and the driver live in a ROS domain that only the
// control host can see, and this process is the only one in both: it talks to the controller
// and reads the arm's sensors there (the "hardware" node), and offers its action, the bond and
// copies of joint states, force and speed scaling on the shared domain. Nothing else on the
// robot network can reach the controllers.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <map>
#include <memory>
#include <future>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "bondcpp/bond.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tahoma_interfaces/action/execute_trajectory.hpp"
#include "tahoma_interfaces/srv/connect.hpp"

using namespace std::chrono_literals;
using Execute = tahoma_interfaces::action::ExecuteTrajectory;
using Follow = control_msgs::action::FollowJointTrajectory;
using Connect = tahoma_interfaces::srv::Connect;
using ExecuteHandle = rclcpp_action::ServerGoalHandle<Execute>;
using FollowHandle = rclcpp_action::ClientGoalHandle<Follow>;
using Result = Execute::Result;

namespace
{
constexpr uint8_t kNoStop = 255;

struct Force
{
  double x{0}, y{0}, z{0};
  double distance(const Force & o) const
  {
    return std::sqrt(
      (x - o.x) * (x - o.x) + (y - o.y) * (y - o.y) + (z - o.z) * (z - o.z));
  }
};

struct Signal
{
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub;
  bool value{false};
  rclcpp::Time stamp{0, 0, RCL_STEADY_TIME};
  double stale_s{0.1};   // per signal: the capcup board sends contact at 5.8 Hz (S6, 2026-11-12)
};
}  // namespace

class MotionServer : public rclcpp::Node
{
public:
  MotionServer()
  // No parameter services: bondcpp reads a parameter that turns its heartbeat timeout off, so
  // nothing outside this process may set parameters here.
  : Node("motion_server", rclcpp::NodeOptions().start_parameter_services(false)),
    steady_(RCL_STEADY_TIME)
  {
    wrench_stale_ = declare_parameter("wrench_stale_s", 0.05);
    signal_stale_ = declare_parameter("signal_stale_s", 0.1);
    joints_stale_ = declare_parameter("joints_stale_s", 0.5);
    start_tolerance_ = declare_parameter("start_tolerance_rad", 0.01);
    baseline_window_ = declare_parameter("baseline_window_s", 0.05);
    heartbeat_timeout_ = declare_parameter("bond_heartbeat_timeout_s", 0.5);
    heartbeat_period_ = declare_parameter("bond_heartbeat_period_s", 0.1);
    connect_timeout_ = declare_parameter("bond_connect_timeout_s", 5.0);
    max_force_limit_ = declare_parameter("max_force_limit_n", 150.0);
    // UR speed scaling, in percent: the tablet slider times any safety slowdown, 0 when stopped.
    use_scaling_ = declare_parameter("use_speed_scaling", false);
    scaling_stale_ = declare_parameter("speed_scaling_stale_s", 0.5);
    stall_timeout_ = declare_parameter("stall_timeout_s", 3.0);
    overtime_ = declare_parameter("overtime_s", 2.0);
    auto names = declare_parameter("signal_names", std::vector<std::string>{});
    auto topics = declare_parameter("signal_topics", std::vector<std::string>{});
    // One per signal, or empty for signal_stale_s on all. The tool's tool.yaml sets them.
    auto stales = declare_parameter("signal_stale_each_s", std::vector<double>{});
    auto controller = declare_parameter(
      "controller_action", std::string("arm_controller/follow_joint_trajectory"));
    // Topic names on the hardware side, as parameters: launch remappings don't reach a second
    // context. Republished under the same names on the shared side.
    const auto wrench_topic = declare_parameter("wrench_topic", std::string("wrench"));
    const auto joints_topic = declare_parameter("joints_topic", std::string("joint_states"));
    const auto scaling_topic = declare_parameter("speed_scaling_topic", std::string("speed_scaling"));
    const auto private_domain = declare_parameter("private_domain_id", -1);
    if (names.size() != topics.size()) {
      throw std::runtime_error("signal_names and signal_topics differ in length");
    }
    if (!stales.empty() && stales.size() != names.size()) {
      throw std::runtime_error("signal_stale_each_s must be empty or one per signal");
    }

    group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions options;
    options.callback_group = group_;

    rclcpp::Node * hw = this;
    rclcpp::SubscriptionOptions hw_options = options;
    if (private_domain >= 0) {
      private_context_ = std::make_shared<rclcpp::Context>();
      rclcpp::InitOptions init;
      init.set_domain_id(static_cast<size_t>(private_domain));
      init.auto_initialize_logging(false);
      private_context_->init(0, nullptr, init);
      hardware_ = std::make_shared<rclcpp::Node>(
        "motion_server_hardware",
        rclcpp::NodeOptions().context(private_context_).start_parameter_services(false));
      hw = hardware_.get();
      hw_group_ = hardware_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
      hw_options.callback_group = hw_group_;
      // Reliable, as the ros2_control broadcasters publish: it serves reliable subscribers
      // (MoveIt, robot_state_publisher) and best-effort ones alike.
      wrench_copy_ = create_publisher<geometry_msgs::msg::WrenchStamped>(wrench_topic, 10);
      joints_copy_ = create_publisher<sensor_msgs::msg::JointState>(joints_topic, 10);
      if (use_scaling_) {
        scaling_copy_ = create_publisher<std_msgs::msg::Float64>(scaling_topic, 10);
      }
      RCLCPP_INFO(get_logger(), "controllers and sensors on private domain %ld", private_domain);
    }
    wrench_sub_ = hw->create_subscription<geometry_msgs::msg::WrenchStamped>(
      wrench_topic, rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::WrenchStamped::ConstSharedPtr m) {
        if (wrench_copy_) {wrench_copy_->publish(*m);}
        on_wrench(*m);
      }, hw_options);
    joints_sub_ = hw->create_subscription<sensor_msgs::msg::JointState>(
      joints_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
        if (joints_copy_) {joints_copy_->publish(*m);}
        on_joints(m);
      }, hw_options);
    if (use_scaling_) {
      scaling_sub_ = hw->create_subscription<std_msgs::msg::Float64>(
        scaling_topic, rclcpp::SensorDataQoS(),
        [this](std_msgs::msg::Float64::ConstSharedPtr m) {
          if (scaling_copy_) {scaling_copy_->publish(*m);}
          on_scaling(m->data);
        }, hw_options);
    }
    for (size_t i = 0; i < names.size(); ++i) {
      const auto name = names[i];
      signals_[name].stale_s = stales.empty() ? signal_stale_ : stales[i];
      if (!(signals_[name].stale_s > 0.0)) {
        throw std::runtime_error("signal " + name + ": stale limit must be above 0");
      }
      signals_[name].sub = create_subscription<std_msgs::msg::Bool>(
        topics[i], rclcpp::SensorDataQoS(),
        [this, name](std_msgs::msg::Bool::ConstSharedPtr m) {on_signal(name, m->data);}, options);
    }

    controller_ = hardware_ ?
      rclcpp_action::create_client<Follow>(hardware_, controller, hw_group_) :
      rclcpp_action::create_client<Follow>(this, controller, group_);
    connect_service_ = create_service<Connect>(
      "~/connect",
      [this](Connect::Request::ConstSharedPtr req, Connect::Response::SharedPtr res) {
        on_connect(*req, *res);
      }, rclcpp::ServicesQoS(), group_);
    server_ = rclcpp_action::create_server<Execute>(
      this, "~/execute",
      [](const rclcpp_action::GoalUUID &, Execute::Goal::ConstSharedPtr) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<ExecuteHandle>) {
        request_stop(Result::STOPPED_REQUEST);
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<ExecuteHandle> handle) {
        std::thread([this, handle] {run(handle);}).detach();
      },
      rcl_action_server_get_default_options(), group_);
    stale_timer_ = create_wall_timer(5ms, [this] {check_stale();}, group_);
  }

  // The node in the private domain, or null when there is none. Spun by its own executor.
  rclcpp::Node::SharedPtr hardware() const {return hardware_;}

  ~MotionServer() override
  {
    if (private_context_) {private_context_->shutdown("motion_server exiting");}
  }

private:
  rclcpp::Time now_steady() {return steady_.now();}

  // --- Guard ---------------------------------------------------------------------------------
  void on_connect(const Connect::Request & req, Connect::Response & res)
  {
    std::lock_guard<std::mutex> lock(bond_mutex_);
    if (bond_ok_ || bond_forming_) {
      res.accepted = false;
      res.message = bond_ok_ ? "another demo holds the motion guard" :
        "another demo is connecting to the motion guard";
      return;
    }
    // A client that dies while connecting frees the guard after the connect timeout.
    bond_forming_ = true;
    bond_id_ = req.bond_id;
    bond_ = std::make_unique<bond::Bond>(
      "/tahoma/bond", req.bond_id, shared_from_this(),
      [this] {
        bond_ok_ = false;
        bond_forming_ = false;
        request_stop(Result::ABORTED_GUARD_LOST);
      },
      [this] {
        bond_ok_ = true;
        bond_forming_ = false;
      });
    bond_->setConnectTimeout(connect_timeout_);
    bond_->setHeartbeatTimeout(heartbeat_timeout_);
    bond_->setHeartbeatPeriod(heartbeat_period_);
    bond_->start();
    res.accepted = true;
  }

  // --- Sensors -------------------------------------------------------------------------------
  void on_wrench(const geometry_msgs::msg::WrenchStamped & msg)
  {
    const Force f{msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z};
    // Skipped, not stored: NaN would pass every comparison. If it persists, the stale check fires.
    if (!std::isfinite(f.x) || !std::isfinite(f.y) || !std::isfinite(f.z)) {return;}
    const auto now = now_steady();
    uint8_t stop = kNoStop;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_wrench_ = now;
      recent_.emplace_back(now, f);
      while (recent_.size() > 2 && (now - recent_.front().first).seconds() > baseline_window_) {
        recent_.pop_front();
      }
      if (active_) {
        const double change = f.distance(baseline_);
        peak_ = std::max(peak_, change);
        if (change > force_limit_) {
          stop = Result::ABORTED_FORCE_LIMIT;
        } else if (stop_force_ > 0 && change > stop_force_) {
          stop = Result::STOPPED_FORCE;
        }
      }
    }
    if (stop != kNoStop) {request_stop(stop);}
  }

  void on_joints(sensor_msgs::msg::JointState::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    joints_ = msg;
    last_joints_ = now_steady();
  }

  void on_signal(const std::string & name, bool value)
  {
    bool fire = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto & s = signals_[name];
      s.value = value;
      s.stamp = now_steady();
      fire = active_ && value && requested(name);
      if (fire && fired_signal_.empty()) {fired_signal_ = name;}
    }
    if (fire) {request_stop(Result::STOPPED_SIGNAL);}
  }

  void on_scaling(double percent)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (std::isfinite(percent)) {
      scaling_ = std::clamp(percent / 100.0, 0.0, 1.0);
      last_scaling_ = now_steady();
    }
  }

  bool requested(const std::string & name) const
  {
    return std::find(active_signals_.begin(), active_signals_.end(), name) !=
           active_signals_.end();
  }

  void check_stale()
  {
    bool stale = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!active_) {return;}
      const auto now = now_steady();
      stale = (now - last_wrench_).seconds() > wrench_stale_;
      stale = stale || (use_scaling_ && (now - last_scaling_).seconds() > scaling_stale_);
      for (const auto & name : active_signals_) {
        stale = stale || (now - signals_[name].stamp).seconds() > signals_[name].stale_s;
      }
    }
    if (stale) {request_stop(Result::ABORTED_SENSOR_STALE);}
  }

  // --- Stopping ------------------------------------------------------------------------------
  // First reason wins; returns whether this one did. Cancelling the controller goal makes it
  // hold position.
  bool request_stop(uint8_t reason)
  {
    uint8_t expected = kNoStop;
    if (!stop_reason_.compare_exchange_strong(expected, reason)) {return false;}
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (goal_) {cancel(goal_);}
    return true;
  }

  // A goal that has just ended is unknown to the client, which then throws. Nothing to stop.
  void cancel(const FollowHandle::SharedPtr & goal)
  {
    try {
      controller_->async_cancel_goal(goal);
    } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
    }
  }

  // --- Running a trajectory ------------------------------------------------------------------
  std::string validate(const Execute::Goal & goal)
  {
    if (!bond_ok_) {return "no demo holds the motion guard";}
    {
      std::lock_guard<std::mutex> lock(bond_mutex_);
      if (goal.bond_id != bond_id_) {return "this client does not hold the motion guard";}
    }
    if (unresolved_) {return "the controller has not confirmed the last move ended";}
    if (!(goal.force_limit_n > 0) || !(goal.force_limit_n <= max_force_limit_)) {
      return "force_limit_n is required: above 0, at most " + std::to_string(max_force_limit_);
    }
    if (!(goal.stop_force_n >= 0) || !std::isfinite(goal.stop_force_n)) {
      return "stop_force_n must be finite and not negative";
    }
    if (goal.trajectory.points.empty()) {return "empty trajectory";}
    if (goal.trajectory.joint_names.empty()) {return "trajectory names no joints";}
    const auto now = now_steady();
    if ((now - last_wrench_).seconds() > wrench_stale_) {return "no recent wrench";}
    if (recent_.size() < 3) {return "not enough wrench samples for a baseline";}
    if (!joints_ || (now - last_joints_).seconds() > joints_stale_) {return "no recent joints";}
    if (use_scaling_ && (now - last_scaling_).seconds() > scaling_stale_) {
      return "no recent speed scaling";
    }
    for (const auto & name : goal.stop_signals) {
      const auto it = signals_.find(name);
      if (it == signals_.end()) {return "this tool has no stop signal called " + name;}
      if ((now - it->second.stamp).seconds() > it->second.stale_s) {
        return "no recent signal " + name;
      }
    }
    const auto joints = goal.trajectory.joint_names.size();
    for (const auto & p : goal.trajectory.points) {
      if (p.positions.size() != joints) {return "a point does not match the joint names";}
      for (const auto * values : {&p.positions, &p.velocities, &p.accelerations}) {
        for (const double v : *values) {
          if (!std::isfinite(v)) {return "trajectory has a value that is not finite";}
        }
      }
    }
    const auto & first = goal.trajectory.points.front().positions;
    for (size_t i = 0; i < goal.trajectory.joint_names.size(); ++i) {
      const auto & names = joints_->name;
      const auto it = std::find(names.begin(), names.end(), goal.trajectory.joint_names[i]);
      if (it == names.end()) {return "unknown joint " + goal.trajectory.joint_names[i];}
      const double current = joints_->position[it - names.begin()];
      if (std::abs(current - first[i]) > start_tolerance_) {
        return "trajectory does not start at the current joints";
      }
    }
    return "";
  }

  void finish(
    std::shared_ptr<ExecuteHandle> handle, Result::SharedPtr result, bool was_active)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (joints_) {result->final_state = *joints_;}
      if (was_active) {
        result->peak_force_n = peak_;
        result->stop_signal = fired_signal_;
        active_ = false;
      }
    }
    if (was_active) {busy_ = false;}
    if (handle->is_canceling()) {
      handle->canceled(result);
    } else {
      handle->succeed(result);
    }
  }

  void run(std::shared_ptr<ExecuteHandle> handle)
  {
    auto result = std::make_shared<Result>();
    result->outcome = Result::FAILED;  // anything that leaves early must not read as DONE
    result->message = "motion server stopped before the move ended";
    const auto goal = handle->get_goal();
    if (busy_.exchange(true)) {
      result->outcome = Result::REJECTED;
      result->message = "another move is running";
      finish(handle, result, false);
      return;
    }
    bool already_signalled = false;
    std::string problem;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      problem = validate(*goal);
    }
    if (!problem.empty()) {
      busy_ = false;
      result->outcome = Result::REJECTED;
      result->message = problem;
      finish(handle, result, false);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      Force sum;
      for (const auto & s : recent_) {
        sum.x += s.second.x;
        sum.y += s.second.y;
        sum.z += s.second.z;
      }
      const double n = static_cast<double>(recent_.size());
      baseline_ = Force{sum.x / n, sum.y / n, sum.z / n};
      force_limit_ = goal->force_limit_n;
      stop_force_ = goal->stop_force_n;
      active_signals_ = goal->stop_signals;
      fired_signal_.clear();
      peak_ = 0;
      stop_reason_ = kNoStop;
      active_ = true;
      for (const auto & name : active_signals_) {
        if (signals_[name].value) {
          already_signalled = true;
          fired_signal_ = name;
        }
      }
    }
    if (already_signalled) {request_stop(Result::STOPPED_SIGNAL);}
    // The bond may have broken after validate(), and that stop was cleared above.
    if (!bond_ok_) {request_stop(Result::ABORTED_GUARD_LOST);}
    // Same for a cancel. send() keeps checking: the cancel callback runs before is_canceling is set.
    if (handle->is_canceling()) {request_stop(Result::STOPPED_REQUEST);}

    bool ended = true;
    if (stop_reason_ == kNoStop) {
      result->message.clear();
      ended = send(*goal, handle, result);
    }
    if (ended && stop_reason_ != kNoStop) {
      result->outcome = stop_reason_;
      if (result->message.empty()) {result->message = "stopped";}
    }
    {
      // A move can pass the session limit after another stop has already won, while it slows down.
      std::lock_guard<std::mutex> lock(mutex_);
      if (ended && result->outcome != Result::FAILED && peak_ > force_limit_) {
        result->outcome = Result::ABORTED_FORCE_LIMIT;
        result->message = "session force limit passed";
      }
    }
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      goal_.reset();
    }
    finish(handle, result, true);
  }

  // Returns false when the arm's state is unknown. unresolved_ then refuses moves until the
  // controller reports the end of this goal, however late.
  bool send(
    const Execute::Goal & goal, std::shared_ptr<ExecuteHandle> request, Result::SharedPtr result)
  {
    if (!controller_->wait_for_action_server(1s)) {
      result->message = "trajectory controller is not available";
      return true;
    }
    Follow::Goal follow;
    follow.trajectory = goal.trajectory;
    follow.trajectory.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);

    // The result arrives through the callback only: asking for it later throws if the goal has
    // already ended. The client's own result request keeps the handle alive until then.
    auto abandoned = std::make_shared<std::atomic<bool>>(false);
    auto ended = std::make_shared<std::promise<FollowHandle::WrappedResult>>();
    auto finished = ended->get_future().share();
    rclcpp_action::Client<Follow>::SendGoalOptions options;
    options.goal_response_callback = [this, abandoned](FollowHandle::SharedPtr late) {
        if (!late) {
          unresolved_ = false;
        } else if (*abandoned) {
          cancel(late);
        }
      };
    options.result_callback = [this, ended](const FollowHandle::WrappedResult & wrapped) {
        unresolved_ = false;
        ended->set_value(wrapped);
      };
    unresolved_ = true;
    auto sent = controller_->async_send_goal(follow, options);
    if (sent.wait_for(2s) != std::future_status::ready) {
      *abandoned = true;
      // It may have answered just before abandoned was set, and then nothing would cancel it.
      if (sent.wait_for(0s) == std::future_status::ready && sent.get()) {cancel(sent.get());}
      RCLCPP_ERROR(get_logger(), "controller did not answer a goal, refusing moves until it does");
      result->message = "trajectory controller did not answer the goal";
      return false;
    }
    if (!sent.get()) {
      result->message = "trajectory controller refused the goal";
      return true;
    }
    auto handle = sent.get();
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      goal_ = handle;
    }
    // A stop may have arrived before goal_ was set.
    if (stop_reason_ != kNoStop) {cancel(handle);}

    // Progress in planned seconds: the controller slows the trajectory by the same scaling.
    const auto & end = goal.trajectory.points.back().time_from_start;
    const double planned = end.sec + end.nanosec * 1e-9;
    double progress = 0;
    auto last = now_steady();
    auto moving_since = last;
    while (finished.wait_for(1ms) != std::future_status::ready) {
      if (request->is_canceling()) {request_stop(Result::STOPPED_REQUEST);}
      const auto now = now_steady();
      double scaling = 1.0;
      if (use_scaling_) {
        std::lock_guard<std::mutex> lock(mutex_);
        scaling = scaling_;
      }
      progress += (now - last).seconds() * scaling;
      last = now;
      // Under 2% counts as stopped: the driver can set the slider no lower than 1%.
      if (scaling >= 0.02) {moving_since = now;}
      // The message is written only if this reason won, so it can't label another stop.
      if ((now - moving_since).seconds() > stall_timeout_ &&
        request_stop(Result::ABORTED_STALLED))
      {
        result->message = "speed scaling near 0 for " + std::to_string(stall_timeout_) +
          " s: tablet slider at 0 or a protective stop";
      }
      if (progress > planned + overtime_ && request_stop(Result::ABORTED_STALLED)) {
        result->message = "move still running " + std::to_string(overtime_) +
          " s after its planned end";
      }
      if (stop_reason_ != kNoStop) {
        if (finished.wait_for(2s) != std::future_status::ready &&
          !hold_here(goal.trajectory.joint_names))
        {
          RCLCPP_ERROR(get_logger(), "controller did not confirm the stop");
          result->message = "controller did not confirm the stop";
          return false;
        }
        unresolved_ = false;
        return true;
      }
      if (!rclcpp::ok()) {
        result->message = "motion server stopped before the move ended";
        return true;
      }
    }
    unresolved_ = false;
    const auto wrapped = finished.get();
    if (stop_reason_ != kNoStop) {return true;}
    if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
      wrapped.result->error_code == Follow::Result::SUCCESSFUL)
    {
      result->outcome = Result::DONE;
      result->message.clear();
    } else {
      result->outcome = Result::FAILED;
      result->message = "trajectory controller: " + wrapped.result->error_string;
    }
    return true;
  }

  // A cancel that reaches the controller after it has finished a goal, but before it has sent the
  // result, leaves that goal canceling with no result ever: realtime_tools 3.13 skips succeed
  // while canceling. A new goal that holds the current joints replaces it; once that one
  // succeeds, the controller is holding still and its state is known again.
  bool hold_here(const std::vector<std::string> & names)
  {
    Follow::Goal hold;
    hold.trajectory.joint_names = names;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!joints_) {return false;}
      for (const auto & name : names) {
        const auto it = std::find(joints_->name.begin(), joints_->name.end(), name);
        if (it == joints_->name.end()) {return false;}
        point.positions.push_back(joints_->position[it - joints_->name.begin()]);
      }
    }
    point.time_from_start = rclcpp::Duration(100ms);
    hold.trajectory.points = {point};
    hold.trajectory.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    auto ended = std::make_shared<std::promise<bool>>();
    auto held = ended->get_future();
    rclcpp_action::Client<Follow>::SendGoalOptions options;
    options.result_callback = [ended](const FollowHandle::WrappedResult & wrapped) {
        ended->set_value(wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
          wrapped.result->error_code == Follow::Result::SUCCESSFUL);
      };
    auto sent = controller_->async_send_goal(hold, options);
    if (sent.wait_for(2s) != std::future_status::ready || !sent.get()) {return false;}
    if (held.wait_for(2s) != std::future_status::ready || !held.get()) {return false;}
    RCLCPP_WARN(get_logger(), "controller did not confirm the stop; a hold goal settled it");
    return true;
  }

  rclcpp::Clock steady_;
  double wrench_stale_, signal_stale_, joints_stale_, start_tolerance_, baseline_window_;
  double heartbeat_timeout_, heartbeat_period_, connect_timeout_, max_force_limit_;
  bool use_scaling_;
  double scaling_stale_, stall_timeout_, overtime_;
  double scaling_{1.0};
  rclcpp::Time last_scaling_{0, 0, RCL_STEADY_TIME};
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr scaling_sub_;

  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Context::SharedPtr private_context_;
  rclcpp::Node::SharedPtr hardware_;
  rclcpp::CallbackGroup::SharedPtr hw_group_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_copy_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_copy_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr scaling_copy_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
  rclcpp_action::Client<Follow>::SharedPtr controller_;
  rclcpp::Service<Connect>::SharedPtr connect_service_;
  rclcpp_action::Server<Execute>::SharedPtr server_;
  rclcpp::TimerBase::SharedPtr stale_timer_;

  std::mutex bond_mutex_;
  std::unique_ptr<bond::Bond> bond_;
  std::atomic<bool> bond_ok_{false};
  std::atomic<bool> bond_forming_{false};
  std::string bond_id_;

  std::mutex mutex_;  // sensors and the active move
  std::deque<std::pair<rclcpp::Time, Force>> recent_;
  rclcpp::Time last_wrench_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time last_joints_{0, 0, RCL_STEADY_TIME};
  sensor_msgs::msg::JointState::ConstSharedPtr joints_;
  std::map<std::string, Signal> signals_;
  bool active_{false};
  Force baseline_;
  double force_limit_{0}, stop_force_{0}, peak_{0};
  std::vector<std::string> active_signals_;
  std::string fired_signal_;

  std::atomic<bool> busy_{false};
  std::atomic<uint8_t> stop_reason_{kNoStop};
  std::mutex goal_mutex_;
  FollowHandle::SharedPtr goal_;
  std::atomic<bool> unresolved_{false};  // a goal was sent and its end has not been seen
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionServer>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  std::thread hardware_spin;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> hardware_executor;
  if (auto hw = node->hardware()) {
    rclcpp::ExecutorOptions options;
    options.context = hw->get_node_base_interface()->get_context();
    hardware_executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(options, 4);
    hardware_executor->add_node(hw);
    hardware_spin = std::thread([&] {hardware_executor->spin();});
  }
  executor.spin();
  if (hardware_executor) {
    hardware_executor->cancel();
    hardware_spin.join();
  }
  rclcpp::shutdown();
  return 0;
}
