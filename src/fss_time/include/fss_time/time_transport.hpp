#ifndef FSS_TIME_TIME_TRANSPORT_HPP_
#define FSS_TIME_TIME_TRANSPORT_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "fss_time_interfaces/msg/time_control.hpp"
#include "fss_time_interfaces/msg/time_intent.hpp"

namespace fss_time
{

struct TimeTransportOptions
{
  std::string intent_topic{"/fss/time_intent"};
  std::string control_topic{"/fss/time_control"};
  std::string participant_id;
};

class TimeTransport
{
public:
  using IntentCallback =
    std::function<void(const fss_time_interfaces::msg::TimeIntent &)>;
  using ControlCallback =
    std::function<void(const fss_time_interfaces::msg::TimeControl &)>;

  virtual ~TimeTransport() = default;

  virtual void start(IntentCallback intent_callback, ControlCallback control_callback) = 0;
  virtual void publish_intent(const fss_time_interfaces::msg::TimeIntent & intent) = 0;
  virtual void publish_control(const fss_time_interfaces::msg::TimeControl & control) = 0;
  virtual std::string name() const = 0;
};

std::unique_ptr<TimeTransport> create_time_transport(
  const std::string & transport_name,
  const TimeTransportOptions & options);

}  // namespace fss_time

#endif  // FSS_TIME_TIME_TRANSPORT_HPP_
