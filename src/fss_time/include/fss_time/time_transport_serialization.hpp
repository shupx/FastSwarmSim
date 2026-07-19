#ifndef FSS_TIME_TIME_TRANSPORT_SERIALIZATION_HPP_
#define FSS_TIME_TIME_TRANSPORT_SERIALIZATION_HPP_

#include <string>

#include "fss_time_interfaces/msg/time_control.hpp"
#include "fss_time_interfaces/msg/time_intent.hpp"

namespace fss_time
{

std::string serialize_time_intent(const fss_time_interfaces::msg::TimeIntent & intent);
bool deserialize_time_intent(
  const void * data,
  std::size_t size,
  fss_time_interfaces::msg::TimeIntent & intent);

std::string serialize_time_control(const fss_time_interfaces::msg::TimeControl & control);
bool deserialize_time_control(
  const void * data,
  std::size_t size,
  fss_time_interfaces::msg::TimeControl & control);

}  // namespace fss_time

#endif  // FSS_TIME_TIME_TRANSPORT_SERIALIZATION_HPP_
