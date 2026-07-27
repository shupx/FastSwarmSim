#include "fss_time/thread_time_participant.hpp"

#include <cctype>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <unistd.h>

namespace fss_time
{

namespace
{

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

std::string sanitize_name(std::string name)
{
  if (name.empty() || name == "/") {
    return "participant";
  }

  while (!name.empty() && name.front() == '/') {
    name.erase(name.begin());
  }
  for (auto & c : name) {
    if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_' && c != '-') {
      c = '_';
    }
  }
  return name.empty() ? "participant" : name;
}

ZeroMqTimeParticipantOptions make_options(
  rclcpp::Node & node,
  const std::string & participant_id_hint)
{
  ZeroMqTimeParticipantOptions options;
  auto base_name = participant_id_hint;
  if (base_name.empty()) {
    base_name = std::string(node.get_namespace());
    if (base_name.empty() || base_name == "/") {
      base_name = node.get_name();
    }
  }

  options.participant_id = sanitize_name(
    base_name + "_" +
    std::to_string(static_cast<long long>(getpid())) + "_" +
    std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id())));
  options.ipc_endpoint =
    declare_or_get_parameter<std::string>(node, "sim_time_broker_endpoint", options.ipc_endpoint);
  return options;
}

thread_local std::unique_ptr<thread_time_participant> tls_participant;

}  // namespace

thread_time_participant::thread_time_participant(
  std::shared_ptr<ZeroMqTimeParticipantBackend> backend)
: backend_(std::move(backend))
{
  backend_->register_participant();
}

thread_time_participant::~thread_time_participant()
{
  unregister_participant();
}

thread_time_participant & thread_time_participant::for_current_thread(
  rclcpp::Node & node,
  const std::string & participant_id_hint)
{
  if (!tls_participant) {
    tls_participant = std::unique_ptr<thread_time_participant>(
      new thread_time_participant(std::make_shared<ZeroMqTimeParticipantBackend>(
        make_options(node, participant_id_hint),
        node.get_clock())));
  }
  return *tls_participant;
}

void thread_time_participant::reset_current_thread_for_testing()
{
  tls_participant.reset();
}

void thread_time_participant::announce_next_safe_time(const rclcpp::Time & next_safe_time)
{
  backend_->announce_next_safe_time(next_safe_time.nanoseconds());
}

void thread_time_participant::unregister_participant()
{
  if (backend_) {
    backend_->unregister_participant();
  }
}

rclcpp::Time thread_time_participant::get_sim_time() const
{
  return rclcpp::Time(backend_->current_time_ns(), RCL_ROS_TIME);
}

}  // namespace fss_time
