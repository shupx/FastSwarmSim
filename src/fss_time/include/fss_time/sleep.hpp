#ifndef FSS_TIME_SLEEP_HPP_
#define FSS_TIME_SLEEP_HPP_

#include "rclcpp/rclcpp.hpp"

namespace fss_time
{

/**
 * @brief Sleep until an absolute time using the node's clock.
 *
 * If the node parameter `use_fss_sim_time` is true, the current thread's
 * fss_time participant announces `until` before delegating to the node clock.
 *
 * @param node Node whose clock and fss_time parameters are used.
 * @param until Absolute time to sleep until.
 * @param context rclcpp context used by the underlying clock sleep.
 * @return true if the target time was reached, false if sleep was interrupted.
 */
bool sleep_until(
  rclcpp::Node & node,
  const rclcpp::Time & until,
  const rclcpp::Context::SharedPtr & context = rclcpp::contexts::get_global_default_context());

/**
 * @brief Sleep for a relative duration using the node's clock.
 *
 * If the node parameter `use_fss_sim_time` is true, the current thread's
 * fss_time participant announces the computed end time before sleeping.
 *
 * @param node Node whose clock and fss_time parameters are used.
 * @param rel_time Relative duration to sleep for.
 * @param context rclcpp context used by the underlying clock sleep.
 * @return true if the end time was reached, false if sleep was interrupted.
 */
bool sleep_for(
  rclcpp::Node & node,
  const rclcpp::Duration & rel_time,
  const rclcpp::Context::SharedPtr & context = rclcpp::contexts::get_global_default_context());

}  // namespace fss_time

#endif  // FSS_TIME_SLEEP_HPP_
