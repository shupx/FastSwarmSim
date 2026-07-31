#ifndef FSS_TIME_RATE_HPP_
#define FSS_TIME_RATE_HPP_

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace fss_time
{

class Rate
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Rate)

  explicit Rate(
    rclcpp::Node & node,
    const double rate);

  explicit Rate(
    rclcpp::Node & node,
    const rclcpp::Duration & period);

  bool sleep();

  rcl_clock_type_t get_type() const;

  bool is_steady() const;

  void reset();

  std::chrono::nanoseconds period() const;

private:
  RCLCPP_DISABLE_COPY(Rate)

  rclcpp::Node & node_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Duration period_;
  rclcpp::Time last_interval_;
};

}  // namespace fss_time

#endif  // FSS_TIME_RATE_HPP_
