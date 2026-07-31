#include "fss_time/rate.hpp"

#include "fss_time/sleep.hpp"

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
  last_interval_(clock_->now())
{
  if (!clock_) {
    throw std::invalid_argument{"clock cannot be null"};
  }
  if (rate <= 0.0) {
    throw std::invalid_argument{"rate must be greater than 0"};
  }
  period_ = rclcpp::Duration::from_seconds(1.0 / rate);
}

Rate::Rate(
  rclcpp::Node & node,
  const rclcpp::Duration & period)
: node_(node),
  clock_(node.get_clock()),
  period_(period),
  last_interval_(clock_->now())
{
  if (!clock_) {
    throw std::invalid_argument{"clock cannot be null"};
  }
  if (period <= rclcpp::Duration(0, 0)) {
    throw std::invalid_argument{"period must be greater than 0"};
  }
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
  auto time_to_sleep = next_interval - now;
  try {
    fss_time::sleep_for(node_, time_to_sleep);
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
