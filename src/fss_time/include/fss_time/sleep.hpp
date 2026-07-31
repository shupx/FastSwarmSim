#ifndef FSS_TIME_SLEEP_HPP_
#define FSS_TIME_SLEEP_HPP_

#include "rclcpp/rclcpp.hpp"

namespace fss_time
{

bool sleep_until(
  rclcpp::Node & node,
  const rclcpp::Time & until,
  const rclcpp::Context::SharedPtr & context = rclcpp::contexts::get_global_default_context());

bool sleep_for(
  rclcpp::Node & node,
  const rclcpp::Duration & rel_time,
  const rclcpp::Context::SharedPtr & context = rclcpp::contexts::get_global_default_context());

}  // namespace fss_time

#endif  // FSS_TIME_SLEEP_HPP_
