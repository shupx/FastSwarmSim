#include "fss_time/sim_time_broker.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <regex>
#include <string>

#include "fss_time/thread_time_participant.hpp"
#include "fss_time/time_types.hpp"

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

uint32_t count_federates_in_query_result(
  const std::string & query_result,
  const std::string & excluded_federate_name)
{
  uint32_t count = 0;
  static const std::regex quoted_entry_regex("\"([^\"]+)\"");
  for (std::sregex_iterator it(query_result.begin(), query_result.end(), quoted_entry_regex), end;
    it != end; ++it)
  {
    const auto federate_name = (*it)[1].str();
    if (federate_name != excluded_federate_name) {
      ++count;
    }
  }
  return count;
}

}  // namespace

SimTimeBroker::SimTimeBroker(rclcpp::Node & node)
: node_(node)
{
  const auto core_type = declare_or_get_parameter<std::string>(node_, "helics_core_type", "zmq");
  const auto broker_address = declare_or_get_parameter<std::string>(node_, "broker_address", "127.0.0.1");
  const auto broker_port = declare_or_get_parameter<int>(node_, "broker_port", 23404);
  const auto start_broker = declare_or_get_parameter<bool>(node_, "start_broker", true);
  const auto broker_federates = declare_or_get_parameter<int>(node_, "helics_broker_federates", 0);
  helics_time_delta_ns_ = declare_or_get_parameter<int64_t>(node_, "helics_time_delta_ns", 1000000);
  speed_regulator_tick_ns_ = declare_or_get_parameter<int64_t>(node_, "speed_regulator_tick_ns", 1000000);
  participant_query_period_ns_ =
    declare_or_get_parameter<int64_t>(node_, "participant_query_period_ns", 500000000);

  const auto max_rtf = declare_or_get_parameter<double>(node_, "max_real_time_factor", 1.0);
  running_ = declare_or_get_parameter<bool>(node_, "running", true);
  max_real_time_factor_ = max_rtf;

  HelicsBrokerOptions broker_options;
  broker_options.core_type = core_type;
  broker_options.broker_address = broker_address;
  broker_options.broker_port = broker_port;
  broker_options.start_broker = start_broker;
  broker_options.federates = broker_federates;
  broker_backend_ = std::make_unique<HelicsBrokerBackend>(broker_options);

  HelicsThreadParticipantOptions regulator_options;
  regulator_options.participant_id = "sim_time_regulator";
  regulator_options.core_type = core_type;
  regulator_options.broker_address = broker_address;
  regulator_options.broker_port = broker_port;
  regulator_options.time_delta_ns = helics_time_delta_ns_;
  regulator_options.count_for_participant_metrics = false;
  regulator_backend_ = std::make_shared<HelicsThreadParticipantBackend>(regulator_options);

  status_pub_ = node_.create_publisher<fss_time_interfaces::msg::SimClockStatus>(
    "/fss/sim_clock_status", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());
  control_srv_ = node_.create_service<fss_time_interfaces::srv::SimClockControl>(
    "/fss/clock_control",
    [this](
      const std::shared_ptr<fss_time_interfaces::srv::SimClockControl::Request> request,
      std::shared_ptr<fss_time_interfaces::srv::SimClockControl::Response> response) {
      set_max_real_time_factor(request->max_real_time_factor);
      set_running(request->running);
      response->success = true;
      response->message = "sim_time_broker updated";
    });
}

SimTimeBroker::~SimTimeBroker()
{
  regulator_timer_.reset();
  if (regulator_backend_) {
    regulator_backend_->finalize();
  }
  if (broker_backend_) {
    broker_backend_->finalize();
  }
}

void SimTimeBroker::start()
{
  broker_backend_->start();
  regulator_backend_->start();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    reset_wall_anchor_locked(regulator_backend_->current_time_ns());
    last_participant_query_steady_ = std::chrono::steady_clock::time_point{};
  }
  refresh_participant_count();
  regulator_timer_ = node_.create_wall_timer(std::chrono::nanoseconds(speed_regulator_tick_ns_), [this]() {
    on_regulator_tick();
  });
  publish_status();
}

void SimTimeBroker::set_running(bool running)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (running_ == running) {
    return;
  }
  running_ = running;
  reset_wall_anchor_locked(regulator_backend_->current_time_ns());
}

void SimTimeBroker::set_max_real_time_factor(double max_real_time_factor)
{
  std::lock_guard<std::mutex> lock(mutex_);
  max_real_time_factor_ = max_real_time_factor;
  reset_wall_anchor_locked(regulator_backend_->current_time_ns());
}

fss_time_interfaces::msg::SimClockStatus SimTimeBroker::status_message() const
{
  fss_time_interfaces::msg::SimClockStatus status;
  status.sim_time = from_ns(regulator_backend_->current_time_ns());
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status.running = running_;
    status.max_real_time_factor = max_real_time_factor_;
    status.participant_count = cached_participant_count_;
  }
  status.regulator_active = regulator_backend_->is_request_in_flight();
  return status;
}

void SimTimeBroker::on_regulator_tick()
{
  refresh_participant_count();

  if (regulator_backend_->poll()) {
    publish_status();
  }

  const auto target_ns = compute_regulator_target_ns();
  if (target_ns > regulator_backend_->current_time_ns()) {
    regulator_backend_->announce_next_safe_time(target_ns);
  }
  publish_status();
}

void SimTimeBroker::publish_status()
{
  status_pub_->publish(status_message());
}

void SimTimeBroker::refresh_participant_count()
{
  const auto now = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (participant_query_period_ns_ > 0 &&
      last_participant_query_steady_ != std::chrono::steady_clock::time_point{} &&
      now - last_participant_query_steady_ < std::chrono::nanoseconds(participant_query_period_ns_))
    {
      return;
    }
  }

  try {
    const auto participant_count = count_federates_in_query_result(
      regulator_backend_->query("root", "federates"),
      "sim_time_regulator");
    std::lock_guard<std::mutex> lock(mutex_);
    cached_participant_count_ = participant_count;
    last_participant_query_steady_ = now;
  } catch (const std::exception &) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (last_participant_query_steady_ == std::chrono::steady_clock::time_point{}) {
      cached_participant_count_ = thread_time_participant::participant_count();
      last_participant_query_steady_ = now;
    }
  }
}

int64_t SimTimeBroker::compute_regulator_target_ns() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!running_) {
    return regulator_backend_->current_time_ns();
  }

  if (max_real_time_factor_ <= 0.0) {
    return kInfiniteTimeNs;
  }

  const auto elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now() - wall_anchor_steady_).count();
  const auto capped_sim_ns = sim_anchor_ns_ + static_cast<int64_t>(elapsed_ns * max_real_time_factor_);
  return std::max(capped_sim_ns, regulator_backend_->current_time_ns());
}

void SimTimeBroker::reset_wall_anchor_locked(int64_t sim_time_ns)
{
  sim_anchor_ns_ = sim_time_ns;
  wall_anchor_steady_ = std::chrono::steady_clock::now();
}

}  // namespace fss_time
