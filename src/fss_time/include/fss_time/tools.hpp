#ifndef FSS_TIME_TOOLS_HPP_
#define FSS_TIME_TOOLS_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <iomanip>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include "fss_time/thread_time_participant.hpp"
#include "fss_time/time_types.hpp"

#include "rclcpp/rclcpp.hpp"

namespace fss_time
{

/// @brief fss_time_tools namespace contains utility functions for working with fss_time and rclcpp nodes.
namespace fss_time_tools
{

/**
 * @brief Declare a node parameter if missing, otherwise read its current value. Thread-safe.
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
    try {
      return node.declare_parameter<T>(name, default_value);
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & e) {
      RCLCPP_WARN(
        node.get_logger(),
        "Parameter %s was declared by another thread before this thread could declare it. "
        "Using the existing value.",
        name.c_str()); // in case of race condition, another thread may have declared the parameter and node.has_parameter(）does not take effect yet, so we catch the exception and use the existing value.
    }
  }

  T value{};
  node.get_parameter(name, value);
  return value;
}

/**
 * @brief Ensure a node has a boolean parameter enabled.
 *
 * If the parameter is missing, it is declared as true. If it exists and is
 * false, this function attempts to set it to true.
 *
 * @param node Node whose boolean parameter is checked.
 * @param parameter_name Name of the boolean parameter.
 */
inline void ensure_bool_parameter_enabled(rclcpp::Node & node, const std::string & parameter_name)
{
  bool param_value = declare_or_get_parameter<bool>(node, parameter_name, false);
  if (param_value) {
    return;
  }

  RCLCPP_INFO(
    node.get_logger(),
    "Setting %s to true.", parameter_name.c_str());
  const auto result = node.set_parameter(rclcpp::Parameter(parameter_name, true));
  if (!result.successful) {
    throw std::runtime_error(
            "failed to set " + parameter_name + " to true: " + result.reason);
  }
}

inline void wait_until_ros_time_is_active(rclcpp::Node & node, bool verbose = false)
{
  int repeat = 0;
  while(rclcpp::ok() && !node.get_clock()->ros_time_is_active()) {
    if (repeat % 5 == 0) { // warn every 1s
      RCLCPP_WARN(
        node.get_logger(),
        "Waiting for ROS time to be active... (Please ensure the use_sim_time parameter be true on start of the node if use_fss_sim_time is true)");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    repeat++;
  }
  if (verbose) {
    RCLCPP_INFO(
      node.get_logger(),
      "ros time is active.");
  }
}

/**
 * @brief Generate a random UUID string.
 * @return Random UUID string.
 */
inline std::string make_uuid()
{
  std::random_device random_device;
  std::uniform_int_distribution<int> byte_distribution(0, 255);
  std::array<unsigned char, 16> bytes{};
  for (auto & byte : bytes) {
    byte = static_cast<unsigned char>(byte_distribution(random_device));
  }
  bytes[6] = static_cast<unsigned char>((bytes[6] & 0x0f) | 0x40);
  bytes[8] = static_cast<unsigned char>((bytes[8] & 0x3f) | 0x80);

  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (std::size_t i = 0; i < bytes.size(); ++i) {
    if (i == 4 || i == 6 || i == 8 || i == 10) {
      output << '-';
    }
    output << std::setw(2) << static_cast<int>(bytes[i]);
  }
  return output.str();
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
 * @brief Announce the participant's current coordinator simulation time as its safe time.
 * @param participant Participant to update.
 */
inline void announce_current_time(thread_time_participant & participant)
{
  participant.announce_next_safe_time(participant.get_sim_time());
}

}  // namespace fss_time_tools
}  // namespace fss_time

#endif  // FSS_TIME_TOOLS_HPP_
