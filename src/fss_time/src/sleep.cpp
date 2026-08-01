#include "fss_time/sleep.hpp"

#include "fss_time/thread_time_participant.hpp"
#include "fss_time/tools.hpp"

#include <mutex>
#include <string>

namespace fss_time
{

bool sleep_until(
  rclcpp::Node & node,
  const rclcpp::Time & until,
  const rclcpp::Context::SharedPtr & context)
{
  bool use_fss_sim_time = fss_time_tools::declare_or_get_parameter<bool>(node, "use_fss_sim_time", false);

  if (use_fss_sim_time) {
    // wait for the use_sim_time parameter to take effect.
    fss_time_tools::wait_until_ros_time_is_active(node); 

    thread_time_participant::for_current_thread(node).announce_next_safe_time(until);
  }
  return node.get_clock()->sleep_until(until, context);
}

bool sleep_for(
  rclcpp::Node & node,
  const rclcpp::Duration & rel_time,
  const rclcpp::Context::SharedPtr & context)
{
  return sleep_until(node, node.get_clock()->now() + rel_time, context);
}

}  // namespace fss_time
