#include "fss_time/time_participant.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <string>
#include <utility>

#include "fss_time/time_types.hpp"

namespace fss_time
{

namespace
{

template<typename T>
T declare_or_get_parameter(rclcpp::Node & node, const std::string & name, const T & default_value)
{
  if (!node.has_parameter(name)) {
    return node.declare_parameter<T>(name, default_value);
  }

  T value{};
  node.get_parameter(name, value);
  return value;
}

std::string sanitize_helics_name(std::string name)
{
  if (name.empty() || name == "/") {
    return "participant";
  }

  while (!name.empty() && name.front() == '/') {
    name.erase(name.begin());
  }
  for (auto & c : name) {
    if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_' && c != '-') {
      c = '_';
    }
  }
  return name.empty() ? "participant" : name;
}

}  // namespace

TimeParticipant::TimeParticipant(
  rclcpp::Node & node,
  std::string participant_id,
  double max_speed_ratio,
  std::chrono::milliseconds lease_timeout,
  bool publish_clock)
: node_(node),
  participant_id_(sanitize_helics_name(std::move(participant_id))),
  lease_timeout_(lease_timeout),
  publish_clock_(publish_clock),
  steady_start_(std::chrono::steady_clock::now()),
  coordinator_(std::make_unique<HelicsTimeCoordinator>(make_helics_options(max_speed_ratio)))
{
  if (publish_clock_) {
    clock_pub_ = node_.create_publisher<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());
  }
}

void TimeParticipant::start()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    coordinator_->start();
    coordinator_->set_idle();
  }
  tick_timer_ = node_.create_wall_timer(std::chrono::milliseconds(5), [this]() { tick(); });
  RCLCPP_INFO(
    node_.get_logger(),
    "fss_time participant '%s' uses HELICS time coordination",
    participant_id_.c_str());
}

void TimeParticipant::announce_next_safe_time(const rclcpp::Time & next_safe_time)
{
  std::lock_guard<std::mutex> lock(mutex_);
  coordinator_->set_next_safe_time(next_safe_time.nanoseconds());
}

void TimeParticipant::announce_idle()
{
  std::lock_guard<std::mutex> lock(mutex_);
  coordinator_->set_idle();
}

void TimeParticipant::announce_leaving()
{
  std::lock_guard<std::mutex> lock(mutex_);
  coordinator_->finalize();
}

rclcpp::Time TimeParticipant::now() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return rclcpp::Time(coordinator_->current_time_ns(), RCL_ROS_TIME);
}

int64_t TimeParticipant::steady_now_ns() const
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now() - steady_start_).count();
}

HelicsTimeOptions TimeParticipant::make_helics_options(double max_speed_ratio) const
{
  HelicsTimeOptions options;
  options.participant_id = participant_id_;
  options.core_type = declare_or_get_parameter<std::string>(
    node_, "helics_core_type", options.core_type);
  options.broker_address = declare_or_get_parameter<std::string>(
    node_, "helics_broker_address", options.broker_address);
  options.broker_port = declare_or_get_parameter<int>(
    node_, "helics_broker_port", options.broker_port);
  options.time_delta_ns = declare_or_get_parameter<int64_t>(
    node_, "helics_time_delta_ns", options.time_delta_ns);
  options.max_speed_ratio = max_speed_ratio;
  return options;
}

void TimeParticipant::tick()
{
  HelicsGrantResult result;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    result = coordinator_->request_grant(steady_now_ns());
  }
  if (publish_clock_ && result.advanced) {
    rosgraph_msgs::msg::Clock clock;
    clock.clock = from_ns(result.grant_time_ns);
    clock_pub_->publish(clock);
  }
}

void TimeParticipant::publish_control(const fss_time_interfaces::msg::TimeControl & control)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto steady_now = steady_now_ns();
  switch (control.command) {
    case fss_time_interfaces::msg::TimeControl::COMMAND_PAUSE:
      coordinator_->set_paused(true);
      break;
    case fss_time_interfaces::msg::TimeControl::COMMAND_RESUME:
      coordinator_->set_paused(false);
      break;
    case fss_time_interfaces::msg::TimeControl::COMMAND_SET_SPEED:
      coordinator_->set_speed(control.max_speed_ratio, steady_now);
      break;
    case fss_time_interfaces::msg::TimeControl::COMMAND_RESET:
      coordinator_->reset(to_ns(control.reset_time), steady_now);
      break;
    default:
      RCLCPP_WARN(node_.get_logger(), "Ignoring unknown fss_time control command %u", control.command);
      break;
  }
}

}  // namespace fss_time
