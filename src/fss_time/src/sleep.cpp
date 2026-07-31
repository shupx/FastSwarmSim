#include "fss_time/sleep.hpp"

#include "fss_time/thread_time_participant.hpp"

#include <mutex>
#include <string>

namespace fss_time
{

namespace
{

template<typename T>
T declare_or_get_parameter_locked(
  rclcpp::Node & node,
  const std::string & name,
  const T & default_value)
{
  static std::mutex parameter_mutex;
  std::lock_guard<std::mutex> lock(parameter_mutex);
  if (!node.has_parameter(name)) {
    return node.declare_parameter<T>(name, default_value);
  }

  T value{};
  node.get_parameter(name, value);
  return value;
}

bool use_fss_sim_time(rclcpp::Node & node)
{
  return declare_or_get_parameter_locked<bool>(node, "use_fss_sim_time", false);
}

}  // namespace

bool sleep_until(
  rclcpp::Node & node,
  const rclcpp::Time & until,
  const rclcpp::Context::SharedPtr & context)
{
  if (use_fss_sim_time(node)) {
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
