#ifndef FASTSWARM_TIME_TIME_TYPES_HPP_
#define FASTSWARM_TIME_TIME_TYPES_HPP_

#include <algorithm>
#include <cstdint>

#include "builtin_interfaces/msg/duration.hpp"
#include "builtin_interfaces/msg/time.hpp"

namespace fastswarm_time
{

constexpr int64_t kNanosecondsPerSecond = 1000000000LL;
constexpr int64_t kInfiniteTimeNs = 2147480000LL * kNanosecondsPerSecond;

inline int64_t to_ns(const builtin_interfaces::msg::Time & time)
{
  return static_cast<int64_t>(time.sec) * kNanosecondsPerSecond + time.nanosec;
}

inline builtin_interfaces::msg::Time from_ns(int64_t ns)
{
  ns = std::max<int64_t>(0, ns);
  builtin_interfaces::msg::Time time;
  time.sec = static_cast<int32_t>(ns / kNanosecondsPerSecond);
  time.nanosec = static_cast<uint32_t>(ns % kNanosecondsPerSecond);
  return time;
}

inline int64_t duration_to_ns(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<int64_t>(duration.sec) * kNanosecondsPerSecond + duration.nanosec;
}

inline builtin_interfaces::msg::Duration duration_from_ns(int64_t ns)
{
  builtin_interfaces::msg::Duration duration;
  duration.sec = static_cast<int32_t>(ns / kNanosecondsPerSecond);
  duration.nanosec = static_cast<uint32_t>(ns % kNanosecondsPerSecond);
  return duration;
}

}  // namespace fastswarm_time

#endif  // FASTSWARM_TIME_TIME_TYPES_HPP_
