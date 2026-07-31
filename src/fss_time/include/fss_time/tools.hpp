#ifndef FSS_TIME_TOOLS_HPP_
#define FSS_TIME_TOOLS_HPP_

#include <stdexcept>
#include <string>

#include "fss_time/thread_time_participant.hpp"
#include "fss_time/time_types.hpp"

#include "rclcpp/rclcpp.hpp"

namespace fss_time
{

/// @brief fss_time_tools namespace contains utility functions for working with fss_time and rclcpp nodes.
namespace fss_time_tools
{

/**
 * @brief Declare a node parameter if missing, otherwise read its current value.
 * @tparam T Parameter value type.
 * @param node Node that owns the parameter.
 * @param name Parameter name.
 * @param default_value Value used when declaring a missing parameter.
 * @return Declared or existing parameter value.
 */
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

/**
 * @brief Ensure a node has ROS simulated time enabled.
 *
 * If `use_sim_time` is missing, it is declared as true. If it exists and is
 * false, this function attempts to set it to true.
 *
 * @param node Node whose use_sim_time parameter is checked.
 * @throws std::runtime_error if setting use_sim_time fails.
 */
inline void ensure_use_sim_time_enabled(rclcpp::Node & node)
{
  bool use_sim_time = false;
  if (!node.has_parameter("use_sim_time")) {
    use_sim_time = node.declare_parameter<bool>("use_sim_time", true);
  } else {
    node.get_parameter("use_sim_time", use_sim_time);
  }

  if (use_sim_time) {
    return;
  }

  RCLCPP_WARN(
    node.get_logger(),
    "use_fss_sim_time is true, but use_sim_time is false. Setting use_sim_time to true.");
  const auto result = node.set_parameter(rclcpp::Parameter("use_sim_time", true));
  if (!result.successful) {
    throw std::runtime_error(
            "failed to set use_sim_time to true: " + result.reason);
  }
}

/**
 * @brief Announce unconstrained safe time for a participant.
 * @param participant Participant to update.
 */
inline void announce_next_safe_time_infinite(thread_time_participant & participant)
{
  participant.announce_next_safe_time(rclcpp::Time(fss_time::kInfiniteTimeNs, RCL_ROS_TIME));
}

/**
 * @brief Announce the participant's current broker simulation time as its safe time.
 * @param participant Participant to update.
 */
inline void announce_current_time(thread_time_participant & participant)
{
  participant.announce_next_safe_time(participant.get_sim_time());
}

}  // namespace fss_time_tools
}  // namespace fss_time

#endif  // FSS_TIME_TOOLS_HPP_
