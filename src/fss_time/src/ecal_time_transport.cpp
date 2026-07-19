#include "fss_time/time_transport.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "fss_time/time_transport_serialization.hpp"

#include <ecal/ecal.h>

namespace fss_time
{

namespace
{

std::string normalize_ecal_topic(std::string topic)
{
  while (!topic.empty() && topic.front() == '/') {
    topic.erase(topic.begin());
  }
  for (auto & c : topic) {
    if (c == '/') {
      c = '.';
    }
  }
  return topic;
}

void ensure_ecal_initialized()
{
  static std::once_flag once;
  std::call_once(once, []() {
    if (!eCAL::IsInitialized()) {
      eCAL::Initialize("fss_time");
    }
  });
}

class EcalTimeTransport : public TimeTransport
{
public:
  explicit EcalTimeTransport(TimeTransportOptions options)
  : options_(std::move(options))
  {
    ensure_ecal_initialized();
  }

  void start(IntentCallback intent_callback, ControlCallback control_callback) override
  {
    intent_callback_ = std::move(intent_callback);
    control_callback_ = std::move(control_callback);

    intent_pub_ = std::make_unique<eCAL::CPublisher>(
      normalize_ecal_topic(options_.intent_topic));
    control_pub_ = std::make_unique<eCAL::CPublisher>(
      normalize_ecal_topic(options_.control_topic));
    intent_sub_ = std::make_unique<eCAL::CSubscriber>(
      normalize_ecal_topic(options_.intent_topic));
    control_sub_ = std::make_unique<eCAL::CSubscriber>(
      normalize_ecal_topic(options_.control_topic));

    intent_sub_->SetReceiveCallback(
      [this](
        const eCAL::STopicId &,
        const eCAL::SDataTypeInformation &,
        const eCAL::SReceiveCallbackData & data) {
        if (data.buffer == nullptr || data.buffer_size == 0 || intent_callback_ == nullptr) {
          return;
        }
        fss_time_interfaces::msg::TimeIntent intent;
        if (deserialize_time_intent(data.buffer, data.buffer_size, intent)) {
          intent_callback_(intent);
        }
      });

    control_sub_->SetReceiveCallback(
      [this](
        const eCAL::STopicId &,
        const eCAL::SDataTypeInformation &,
        const eCAL::SReceiveCallbackData & data) {
        if (data.buffer == nullptr || data.buffer_size == 0 || control_callback_ == nullptr) {
          return;
        }
        fss_time_interfaces::msg::TimeControl control;
        if (deserialize_time_control(data.buffer, data.buffer_size, control)) {
          control_callback_(control);
        }
      });
  }

  void publish_intent(const fss_time_interfaces::msg::TimeIntent & intent) override
  {
    const auto payload = serialize_time_intent(intent);
    intent_pub_->Send(payload.data(), payload.size());
  }

  void publish_control(const fss_time_interfaces::msg::TimeControl & control) override
  {
    const auto payload = serialize_time_control(control);
    control_pub_->Send(payload.data(), payload.size());
  }

  std::string name() const override { return "ecal"; }

private:
  TimeTransportOptions options_;
  IntentCallback intent_callback_;
  ControlCallback control_callback_;
  std::unique_ptr<eCAL::CPublisher> intent_pub_;
  std::unique_ptr<eCAL::CPublisher> control_pub_;
  std::unique_ptr<eCAL::CSubscriber> intent_sub_;
  std::unique_ptr<eCAL::CSubscriber> control_sub_;
};

}  // namespace

std::unique_ptr<TimeTransport> create_ecal_time_transport(
  const TimeTransportOptions & options)
{
  return std::make_unique<EcalTimeTransport>(options);
}

}  // namespace fss_time
