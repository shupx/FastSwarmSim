#include "fss_time/rate.hpp"

#include "fss_time/thread_time_participant.hpp"
#include "fss_time/tools.hpp"

#include <chrono>
#include <stdexcept>

namespace fss_time
{

Rate::Rate(
  rclcpp::Node & node,
  const double rate)
: node_(node),
  clock_(node.get_clock()),
  period_(0, 0),
  last_interval_(0, 0, RCL_ROS_TIME)
{
  if (!clock_) {
    throw std::invalid_argument{"clock cannot be null"};
  }
  initialize_fss_sim_time();
  if (rate <= 0.0) {
    throw std::invalid_argument{"rate must be greater than 0"};
  }
  period_ = rclcpp::Duration::from_seconds(1.0 / rate);
  last_interval_ = clock_->now();
}

Rate::Rate(
  rclcpp::Node & node,
  const rclcpp::Duration & period)
: node_(node),
  clock_(node.get_clock()),
  period_(period),
  last_interval_(0, 0, RCL_ROS_TIME)
{
  if (!clock_) {
    throw std::invalid_argument{"clock cannot be null"};
  }
  initialize_fss_sim_time();
  if (period <= rclcpp::Duration(0, 0)) {
    throw std::invalid_argument{"period must be greater than 0"};
  }
  last_interval_ = clock_->now();
}

void Rate::initialize_fss_sim_time()
{
  use_fss_sim_time_ =
    fss_time_tools::declare_or_get_parameter<bool>(node_, "use_fss_sim_time", false);
  if (use_fss_sim_time_) {
    fss_time_tools::ensure_use_sim_time_enabled(node_);
  }
}

bool Rate::utils_sleep_until(const rclcpp::Time & until)
{
  if (use_fss_sim_time_) {
    thread_time_participant::for_current_thread(node_).announce_next_safe_time(until);
  }
  return clock_->sleep_until(until);
}

bool Rate::sleep()
{
  auto now = clock_->now();
  auto next_interval = last_interval_ + period_;
  if (now < last_interval_) {
    next_interval = now + period_;
  }
  last_interval_ += period_;
  if (next_interval <= now) {
    if (now > next_interval + period_) {
      last_interval_ = now + period_;
    }
    return false;
  }
  try {
    utils_sleep_until(next_interval);
  } catch (const std::runtime_error &) {
    return false;
  }
  return true;
}

rcl_clock_type_t Rate::get_type() const
{
  return clock_->get_clock_type();
}

bool Rate::is_steady() const
{
  return clock_->get_clock_type() == RCL_STEADY_TIME;
}

void Rate::reset()
{
  last_interval_ = clock_->now();
}

std::chrono::nanoseconds Rate::period() const
{
  return std::chrono::nanoseconds(period_.nanoseconds());
}

}  // namespace fss_time
