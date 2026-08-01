#include "fss_time/sim_time_broker.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <zmq.hpp>

#include "fss_time/time_types.hpp"

namespace fss_time
{

namespace
{

constexpr int64_t kMinOperationWalltime_ns = 100000;

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

bool ipc_endpoint_path(const std::string & endpoint, std::string & path)
{
  constexpr char prefix[] = "ipc://";
  if (endpoint.rfind(prefix, 0) != 0) {
    return false;
  }
  path = endpoint.substr(sizeof(prefix) - 1);
  return !path.empty() && path.front() == '/';
}

}  // namespace

struct SimTimeBroker::Impl
{
  zmq::context_t context{1};
  zmq::socket_t socket{context, zmq::socket_type::router};
};

SimTimeBroker::SimTimeBroker(rclcpp::Node & node)
: node_(node), impl_(std::make_unique<Impl>())
{
  endpoint_ = normalize_zmq_endpoint(
    declare_or_get_parameter<std::string>(node_, "fss_time_broker_endpoint", "ipc:///tmp/fss_time_broker.ipc"));
  min_operation_walltime_ = kMinOperationWalltime_ns;
  const auto configured_speed_regulator_step_ns =
    declare_or_get_parameter<int64_t>(node_, "speed_regulator_step_ns", 1000000);
  if (configured_speed_regulator_step_ns < min_operation_walltime_) {
    RCLCPP_WARN(
      node_.get_logger(),
      "speed_regulator_step_ns=%lld is smaller than min_operation_walltime=%lld ns. "
      "Clamping speed_regulator_step_ns to %lld ns.",
      static_cast<long long>(configured_speed_regulator_step_ns),
      static_cast<long long>(min_operation_walltime_),
      static_cast<long long>(min_operation_walltime_));
  }
  speed_regulator_step_ns_ =
    std::max<int64_t>(min_operation_walltime_, configured_speed_regulator_step_ns);
  const auto max_rtf = declare_or_get_parameter<double>(node_, "max_real_time_factor", 1.0);
  running_ = declare_or_get_parameter<bool>(node_, "auto_start", true);
  max_real_time_factor_ = max_rtf;

  clock_pub_ = node_.create_publisher<rosgraph_msgs::msg::Clock>(
    "/clock", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());
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
  clock_status_timer_.reset();
  regulator_timer_.reset();
  stop_receive_.store(true);
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
  if (impl_) {
    try {
      impl_->socket.close();
      impl_->context.close();
    } catch (const zmq::error_t &) {
    }
  }

  std::string ipc_path;
  if (ipc_endpoint_path(endpoint_, ipc_path)) {
    std::remove(ipc_path.c_str());
  }
}

void SimTimeBroker::start()
{
  std::string ipc_path;
  if (ipc_endpoint_path(endpoint_, ipc_path)) {
    std::remove(ipc_path.c_str());
  }

  impl_->socket.set(zmq::sockopt::linger, 0);
  impl_->socket.set(zmq::sockopt::rcvhwm, 100000);
  impl_->socket.set(zmq::sockopt::rcvtimeo, 100);
  impl_->socket.bind(endpoint_);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    regulator_request_ns_ = 0;
    observed_rtf_last_sim_time_ns_ = sim_time_ns_;
    observed_rtf_last_wall_time_ = std::chrono::steady_clock::now();
    publish_clock_locked();
  }

  receive_thread_ = std::thread([this]() { receive_loop(); });
  reset_regulator_timer();
  clock_status_timer_ = node_.create_wall_timer(std::chrono::milliseconds(100), [this]() {
    on_clock_status_tick();
  });
  publish_status();
}

void SimTimeBroker::set_running(bool running)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_ == running) {
      return;
    }
    running_ = running;
    if (!running_) {
      regulator_request_ns_ = sim_time_ns_;
    } else {
      regulator_request_ns_ = sim_time_ns_;
    }
  }
  publish_status();
}

void SimTimeBroker::set_max_real_time_factor(double max_real_time_factor)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    max_real_time_factor_ = max_real_time_factor;
    if (!running_) {
      regulator_request_ns_ = sim_time_ns_;
    } else {
      regulator_request_ns_ = sim_time_ns_;
    }
  }
  reset_regulator_timer();
  publish_status();
}

fss_time_interfaces::msg::SimClockStatus SimTimeBroker::status_message() const
{
  fss_time_interfaces::msg::SimClockStatus status;
  std::lock_guard<std::mutex> lock(mutex_);
  status.sim_time = from_ns(sim_time_ns_);
  status.running = running_;
  status.max_real_time_factor = max_real_time_factor_;
  status.observed_real_time_factor = observed_real_time_factor_;
  status.participant_count = static_cast<uint32_t>(participants_.size());
  status.new_request_participant_count = static_cast<uint32_t>(std::count_if(
      participants_.begin(),
      participants_.end(),
      [](const auto & entry) {
        return entry.second.has_new_request;
      }));
  status.regulator_active = running_ && max_real_time_factor_ > 0.0;
  status.debug_msg = debug_msg_;
  status.speed_regulator_step_ns = speed_regulator_step_ns_;
  status.min_operation_walltime = min_operation_walltime_;
  return status;
}

void SimTimeBroker::receive_loop()
{
  while (!stop_receive_.load()) {
    zmq::message_t identity_frame;
    zmq::message_t message_frame;
    try {
      const auto received = impl_->socket.recv(identity_frame, zmq::recv_flags::none);
      if (!received) {
        continue;
      }
      const auto received_message = impl_->socket.recv(message_frame, zmq::recv_flags::none);
      if (!received_message) {
        continue;
      }

      const auto reply_text = handle_message(identity_frame.to_string(), message_frame.to_string());
      zmq::message_t identity(identity_frame.data(), identity_frame.size());
      zmq::message_t reply(reply_text.begin(), reply_text.end());
      impl_->socket.send(identity, zmq::send_flags::sndmore);
      impl_->socket.send(reply, zmq::send_flags::none);
    } catch (const zmq::error_t &) {
      if (!stop_receive_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      continue;
    }
  }
}

std::string SimTimeBroker::handle_message(const std::string & identity, const std::string & message)
{
  std::istringstream input(message);
  std::string command;
  input >> command;

  std::lock_guard<std::mutex> lock(mutex_);
  if (command == "REGISTER") {
    participants_.try_emplace(identity);
    return "OK";
  }

  if (command == "ANNOUNCE") {
    int64_t request_ns = 0;
    input >> request_ns;
    auto & participant = participants_[identity];
    participant.request_time_ns = request_ns;
    participant.has_new_request = true;
    try_update_clock_locked();
    return "OK";
  }

  if (command == "UNREGISTER") {
    participants_.erase(identity);
    try_update_clock_locked();
    return "OK";
  }

  return "ERROR unknown command";
}

void SimTimeBroker::on_regulator_tick()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    regulator_request_ns_ = compute_regulator_target_ns_locked();
    try_update_clock_locked();
  }
}

void SimTimeBroker::on_clock_status_tick()
{
  {
    /* Publish /clock here, only for telling the lately added participants about the current time */
    std::lock_guard<std::mutex> lock(mutex_);
    update_observed_real_time_factor_locked();
    publish_clock_locked();
  }
  publish_status();
}

void SimTimeBroker::update_observed_real_time_factor_locked()
{
  const auto now = std::chrono::steady_clock::now();
  if (observed_rtf_last_wall_time_.time_since_epoch().count() == 0) {
    observed_rtf_last_wall_time_ = now;
    observed_rtf_last_sim_time_ns_ = sim_time_ns_;
    observed_real_time_factor_ = 0.0;
    return;
  }

  const auto wall_dt = now - observed_rtf_last_wall_time_;
  const auto wall_dt_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(wall_dt).count();
  if (wall_dt_ns <= 0) {
    return;
  }

  const auto sim_dt_ns = sim_time_ns_ - observed_rtf_last_sim_time_ns_;
  observed_real_time_factor_ =
    std::max(0.0, static_cast<double>(sim_dt_ns) / static_cast<double>(wall_dt_ns));
  observed_rtf_last_wall_time_ = now;
  observed_rtf_last_sim_time_ns_ = sim_time_ns_;
}

void SimTimeBroker::try_update_clock_locked()
{
  if (participants_.empty()) {
    debug_msg_ = "try_update_clock_locked: publish_clock_locked skipped: no participants";
    return;
  }

  bool all_have_new_request = true;
  int64_t min_request_ns = regulator_request_ns_;
  std::string waiting_participant;
  for (const auto & entry : participants_) {
    const auto & participant = entry.second;
    if (!participant.has_new_request) {
      all_have_new_request = false;
      waiting_participant = entry.first;
      break;
    }
    min_request_ns = std::min(min_request_ns, participant.request_time_ns);
  }

  if (!all_have_new_request) {
    debug_msg_ =
      "try_update_clock_locked: publish_clock_locked skipped: waiting for participant " +
      waiting_participant + " to announce next safe time";
    return;
  }

  if (min_request_ns >= kInfiniteTimeNs - speed_regulator_step_ns_) {
    debug_msg_ =
      "try_update_clock_locked: publish_clock_locked skipped: minimum request is infinite or near infinite";
    return;
  }

  if (min_request_ns <= sim_time_ns_) {
    debug_msg_ =
      "try_update_clock_locked: publish_clock_locked skipped: minimum request " +
      std::to_string(min_request_ns) + " ns is not greater than current sim time " +
      std::to_string(sim_time_ns_) + " ns";
    return;
  }

  const auto previous_sim_time_ns = sim_time_ns_;
  sim_time_ns_ = min_request_ns;
  publish_clock_locked();
  debug_msg_ =
    "try_update_clock_locked: publish_clock_locked published: sim time advanced from " +
    std::to_string(previous_sim_time_ns) + " ns to " + std::to_string(sim_time_ns_) + " ns";

  for (auto & entry : participants_) {
    auto & participant = entry.second;
    if (participant.request_time_ns <= sim_time_ns_) {
      participant.has_new_request = false;
    }
  }
}

void SimTimeBroker::publish_clock_locked()
{
  rosgraph_msgs::msg::Clock clock;
  clock.clock = from_ns(sim_time_ns_);
  clock_pub_->publish(clock);
}

void SimTimeBroker::publish_status()
{
  status_pub_->publish(status_message());
}

int64_t SimTimeBroker::compute_regulator_target_ns_locked() const
{
  if (!running_) {
    return sim_time_ns_;
  }

  if (max_real_time_factor_ <= 0.0) {
    return sim_time_ns_;
  }

  if (sim_time_ns_ < regulator_request_ns_) {
    return regulator_request_ns_;
  }

  if (regulator_request_ns_ >= kInfiniteTimeNs - speed_regulator_step_ns_) {
    return kInfiniteTimeNs;
  }
  return regulator_request_ns_ + speed_regulator_step_ns_;
}

void SimTimeBroker::reset_regulator_timer()
{
  regulator_timer_.reset();  // Cancel the previous timer if it exists
  const auto period = regulator_wall_period();
  if (period == std::chrono::nanoseconds::max()) {
    return; // regulator has already been cancelled, no need to create a timer
  }
  regulator_timer_ = node_.create_wall_timer(period, [this]() {
    on_regulator_tick();
  });
}

std::chrono::nanoseconds SimTimeBroker::regulator_wall_period() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (max_real_time_factor_ <= 0.0) {
    return std::chrono::nanoseconds::max();
  }

  const auto period_ns = static_cast<int64_t>(std::ceil(
      static_cast<double>(speed_regulator_step_ns_) / max_real_time_factor_));
  return std::chrono::nanoseconds(std::max<int64_t>(min_operation_walltime_, period_ns));
}

std::string SimTimeBroker::normalize_zmq_endpoint(const std::string & endpoint) const
{
  if (!endpoint.empty()) {
    return endpoint;
  }
  return "ipc:///tmp/fss_time_broker.ipc";
}

}  // namespace fss_time
