#ifndef FASTSWARM_TIME_TIME_TYPES_HPP_
#define FASTSWARM_TIME_TIME_TYPES_HPP_

#include <algorithm>
#include <cstdint>

#include "builtin_interfaces/msg/duration.hpp"
#include "builtin_interfaces/msg/time.hpp"

namespace fss_time
{

/// Number of nanoseconds in one second.
constexpr int64_t kNanosecondsPerSecond = 1000000000LL;

/// Sentinel safe time used to represent an unconstrained participant.
constexpr int64_t kInfiniteTimeNs = 2147480000LL * kNanosecondsPerSecond;

/**
 * @brief Convert a builtin_interfaces time message to nanoseconds.
 * @param time Time message to convert.
 * @return Time in nanoseconds.
 */
inline int64_t to_ns(const builtin_interfaces::msg::Time & time)
{
  return static_cast<int64_t>(time.sec) * kNanosecondsPerSecond + time.nanosec;
}

/**
 * @brief Convert nanoseconds to a builtin_interfaces time message.
 * @param ns Time in nanoseconds. Negative values are clamped to zero.
 * @return Time message.
 */
inline builtin_interfaces::msg::Time from_ns(int64_t ns)
{
  ns = std::max<int64_t>(0, ns);
  builtin_interfaces::msg::Time time;
  time.sec = static_cast<int32_t>(ns / kNanosecondsPerSecond);
  time.nanosec = static_cast<uint32_t>(ns % kNanosecondsPerSecond);
  return time;
}

/**
 * @brief Convert a builtin_interfaces duration message to nanoseconds.
 * @param duration Duration message to convert.
 * @return Duration in nanoseconds.
 */
inline int64_t duration_to_ns(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<int64_t>(duration.sec) * kNanosecondsPerSecond + duration.nanosec;
}

/**
 * @brief Convert nanoseconds to a builtin_interfaces duration message.
 * @param ns Duration in nanoseconds.
 * @return Duration message.
 */
inline builtin_interfaces::msg::Duration duration_from_ns(int64_t ns)
{
  builtin_interfaces::msg::Duration duration;
  duration.sec = static_cast<int32_t>(ns / kNanosecondsPerSecond);
  duration.nanosec = static_cast<uint32_t>(ns % kNanosecondsPerSecond);
  return duration;
}

}  // namespace fss_time

#endif  // FASTSWARM_TIME_TIME_TYPES_HPP_
