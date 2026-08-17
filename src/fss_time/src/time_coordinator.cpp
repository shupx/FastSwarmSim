#include "fss_time/time_coordinator.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <zmq.hpp>

#include "fss_time/time_types.hpp"
#include "fss_time/tools.hpp"

namespace fss_time
{

namespace
{

constexpr int64_t kMinOperationWalltime_ns = 100000LL; // 100us, a clock publish may take 100us at least (from publlish to subscriber callback)

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
 * @brief Check whether an endpoint is an absolute IPC path and extract that path.
 * @param endpoint Endpoint to inspect.
 * @param[out] path Extracted IPC filesystem path.
 * @return true when endpoint is an absolute IPC endpoint.
 */
bool is_ipc_endpoint_path(const std::string & endpoint, std::string & path)
{
  constexpr char prefix[] = "ipc://";
  if (endpoint.rfind(prefix, 0) != 0) {
    return false;
  }
  path = endpoint.substr(sizeof(prefix) - 1);
  return !path.empty() && path.front() == '/';
}

/** @brief Components of a parsed ZeroMQ TCP endpoint. */
struct TcpEndpointParts
{
  std::string host;
  std::string port;
  std::size_t port_offset{};
};

/**
 * @brief Parse a ZeroMQ TCP endpoint into its host, port, and port offset.
 * @param endpoint Endpoint to parse.
 * @return Parsed endpoint parts, or std::nullopt for a non-TCP/malformed endpoint.
 */
std::optional<TcpEndpointParts> parse_tcp_endpoint(const std::string & endpoint)
{
  constexpr char prefix[] = "tcp://";
  if (endpoint.rfind(prefix, 0) != 0) {
    return std::nullopt;
  }
  const auto address = endpoint.substr(sizeof(prefix) - 1);
  std::size_t separator = std::string::npos;
  if (!address.empty() && address.front() == '[') {
    const auto closing_bracket = address.find(']');
    if (closing_bracket != std::string::npos && closing_bracket + 1 < address.size() &&
      address[closing_bracket + 1] == ':') {
      separator = closing_bracket + 1;
    }
  } else {
    separator = address.rfind(':');
  }
  if (separator == std::string::npos || separator + 1 >= address.size()) {
    return std::nullopt;
  }
  return TcpEndpointParts{address.substr(0, separator), address.substr(separator + 1),
    sizeof(prefix) - 1 + separator + 1};
}

/**
 * @brief Replace the port portion of a TCP endpoint.
 * @param endpoint Endpoint whose port should be replaced.
 * @param port New port text.
 * @return Endpoint with the new port, or the original endpoint if it is not TCP.
 */
std::string replace_tcp_endpoint_port(const std::string & endpoint, const std::string & port)
{
  const auto parts = parse_tcp_endpoint(endpoint);
  if (!parts) {
    return endpoint;
  }
  return endpoint.substr(0, parts->port_offset) + port;
}

/**
 * @brief Replace a wildcard TCP host with the host from a parent endpoint.
 * @param endpoint Endpoint potentially containing a wildcard host.
 * @param parent_endpoint Endpoint supplying the concrete host.
 * @return Endpoint with its wildcard host replaced when both endpoints are TCP.
 */
std::string replace_tcp_endpoint_wildcard_host(
  const std::string & endpoint, const std::string & parent_endpoint)
{
  const auto endpoint_parts = parse_tcp_endpoint(endpoint);
  const auto parent_parts = parse_tcp_endpoint(parent_endpoint);
  if (!endpoint_parts || !parent_parts ||
    (endpoint_parts->host != "*" && endpoint_parts->host != "0.0.0.0" &&
    endpoint_parts->host != "[::]")) {
    return endpoint;
  }
  return "tcp://" + parent_parts->host + ":" + endpoint_parts->port;
}

}  // namespace

struct TimeCoordinator::Impl
{
  zmq::context_t context{1};
  zmq::socket_t socket{context, zmq::socket_type::router};
  zmq::socket_t pub_socket{context, zmq::socket_type::pub};
  zmq::socket_t parent_sub_socket{context, zmq::socket_type::sub};
};

TimeCoordinator::TimeCoordinator(rclcpp::Node & node)
: node_(node), impl_(std::make_unique<Impl>())
{
  endpoint_ =
    declare_or_get_parameter<std::string>(
      node_, "fss_time_coordinator_endpoint", "ipc:///tmp/fss_time_coordinator.ipc");
  pub_endpoint_ =
    declare_or_get_parameter<std::string>(
      node_, "fss_time_coordinator_pub_endpoint", "");
  parent_endpoint_ =
    declare_or_get_parameter<std::string>(node_, "fss_time_parent_coordinator_endpoint", "");
  has_parent_coordinator_ = !parent_endpoint_.empty();
  min_operation_walltime_ = kMinOperationWalltime_ns;
  const auto configured_speed_regulator_step_ns =
    declare_or_get_parameter<int64_t>(node_, "speed_regulator_step_ns", 5000000);
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
  const auto configured_zombie_participant_timeout_ms =
    declare_or_get_parameter<int64_t>(node_, "zombie_participant_timeout_ms", 500);
  if (configured_zombie_participant_timeout_ms < 0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "zombie_participant_timeout_ms=%lld is negative. Clamping it to 0 ms.",
      static_cast<long long>(configured_zombie_participant_timeout_ms));
  }
  zombie_participant_timeout_ = std::chrono::milliseconds(
    std::max<int64_t>(0, configured_zombie_participant_timeout_ms));
  running_ = declare_or_get_parameter<bool>(node_, "auto_start", true);
  follows_real_time_ = declare_or_get_parameter<bool>(node_, "follows_real_time", true);
  publish_clock_ = declare_or_get_parameter<bool>(node_, "publish_clock", true);
  max_real_time_factor_ = max_rtf;

  if (publish_clock_) {
    // Use rclcpp::ClockQoS() to ensure that the /clock topic is best-effort and volatile, which is the standard for /clock in ROS 2. Do not use a reliable QoS for /clock, as it will increase the clock publishing time significantly as subscribers are added and is not necessary for clock use cases.
    clock_pub_ = node_.create_publisher<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::ClockQoS()); 
  }
  status_pub_ = node_.create_publisher<fss_time_interfaces::msg::SimClockStatus>(
    "fss/sim_clock_status", rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile().transient_local());
  control_srv_ = node_.create_service<fss_time_interfaces::srv::SimClockControl>(
    "fss/clock_control",
    [this](
      const std::shared_ptr<fss_time_interfaces::srv::SimClockControl::Request> request,
      std::shared_ptr<fss_time_interfaces::srv::SimClockControl::Response> response) {
      set_max_real_time_factor(request->max_real_time_factor);
      set_running(request->running);
      response->success = true;
      response->message = "time_coordinator updated";
    });
  clear_zombie_participants_srv_ = node_.create_service<std_srvs::srv::Trigger>(
    "fss/clear_zombie_participants",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      const auto removed_count = clear_zombie_participants();
      response->success = true;
      response->message = removed_count == 0 ?
        "No zombie participants found." :
        "Removed " + std::to_string(removed_count) + " zombie participant(s).";
    });
}

TimeCoordinator::~TimeCoordinator()
{
  clock_status_timer_.reset();
  real_time_timer_.reset();
  regulator_timer_.reset();
  stop_receive_.store(true);
  if (receive_router_thread_.joinable()) {
    receive_router_thread_.join();
  }
  if (receive_parent_thread_.joinable()) {
    receive_parent_thread_.join();
  }
  stop_async_worker();
  if (impl_) {
    try {
      impl_->socket.close();
      impl_->pub_socket.close();
      impl_->parent_sub_socket.close();
      impl_->context.close();
    } catch (const zmq::error_t &) {
    }
  }

  std::string ipc_path;
  if (is_ipc_endpoint_path(endpoint_, ipc_path)) {
    std::remove(ipc_path.c_str());
  }
  if (is_ipc_endpoint_path(pub_endpoint_, ipc_path)) {
    std::remove(ipc_path.c_str());
  }
}

void TimeCoordinator::start()
{
  std::string ipc_path;
  if (is_ipc_endpoint_path(endpoint_, ipc_path)) {
    std::remove(ipc_path.c_str());
  }
  if (is_ipc_endpoint_path(pub_endpoint_, ipc_path)) {
    std::remove(ipc_path.c_str());
  }

  // bind the ROUTER socket for receiving requests from participants
  impl_->socket.set(zmq::sockopt::linger, 0);
  impl_->socket.set(zmq::sockopt::rcvhwm, 100000);
  impl_->socket.set(zmq::sockopt::rcvtimeo, 0);
  impl_->socket.bind(endpoint_);

  // The granted-time PUB channel is optional for non-cascaded coordinators.
  if (!pub_endpoint_.empty()) {
    impl_->pub_socket.set(zmq::sockopt::linger, 0);
    impl_->pub_socket.set(zmq::sockopt::sndhwm, 100000);
    impl_->pub_socket.bind(pub_endpoint_);
    // A TCP port of zero requests an ephemeral port. Advertise the port selected by bind().
    if (parse_tcp_endpoint(pub_endpoint_)) {
      if (const auto bound_pub = parse_tcp_endpoint(
          impl_->pub_socket.get(zmq::sockopt::last_endpoint))) {
        pub_endpoint_ = replace_tcp_endpoint_port(pub_endpoint_, bound_pub->port);
      }
    }
  }

  // connect to the parent coordinator if specified
  if (has_parent_coordinator_) {
    // Create a participant backend to communicate with the parent coordinator (It auto connects to the parent coordinator's ROUTER socket when request_coordinator or register_participant or announce_next_safe_time is called).
    ZeroMqTimeParticipantOptions parent_options;
    parent_options.participant_id =
      std::string(node_.get_name()) + "_coordinator_participant_" +
      fss_time_tools::make_uuid();
    parent_options.coordinator_endpoint = parent_endpoint_;
    parent_participant_ =
      std::make_unique<ZeroMqTimeParticipantBackend>(std::move(parent_options), node_.get_clock());
    parent_participant_->register_participant();
    parent_participant_->set_follows_real_time(follows_real_time_);

    // Get the parent coordinator's PUB endpoint so we can subscribe to it (for granted clock messages).
    const auto parent_pub_reply =
      parent_participant_->request_coordinator("GET_PUB_ENDPOINT", true);
    std::istringstream input(parent_pub_reply);
    std::string status;
    input >> status >> parent_pub_endpoint_;
    if (status != "PUB_ENDPOINT" || parent_pub_endpoint_.empty()) {
      throw std::runtime_error(
              "parent coordinator does not provide a PUB endpoint: " + parent_endpoint_ +
              ": " + parent_pub_reply);
    }
    if (const auto parent_pub = parse_tcp_endpoint(parent_pub_endpoint_)) {
      if (!parent_pub->port.empty() &&
        parent_pub->port.find_first_not_of('0') == std::string::npos) {
        throw std::runtime_error(
                "parent coordinator PUB endpoint has invalid port 0: " + parent_pub_endpoint_);
      }
      parent_pub_endpoint_ =
        replace_tcp_endpoint_wildcard_host(parent_pub_endpoint_, parent_endpoint_);
    }
    impl_->parent_sub_socket.set(zmq::sockopt::linger, 0);
    impl_->parent_sub_socket.set(zmq::sockopt::subscribe, "");
    impl_->parent_sub_socket.connect(parent_pub_endpoint_);
  }

  start_async_worker(); // heavy work is moved to this separate thread, like clock publishing (200us) and zeromq message sending (5us), so that the coordinator can respond to participants quickly.

  {
    std::lock_guard<std::mutex> lock(mutex_);
    regulator_request_ns_ = 0;
    real_time_request_ns_ = sim_time_ns_;
    real_time_last_wall_time_ = std::chrono::steady_clock::now();
    observed_rtf_last_sim_time_ns_ = sim_time_ns_;
    observed_rtf_last_wall_time_ = std::chrono::steady_clock::now();
    advance_time_locked(sim_time_ns_);
  }

  // Keep the participant ROUTER and parent SUB receive paths independent.
  receive_router_thread_ = std::thread([this]() { receive_router_loop(); });
  if (has_parent_coordinator_) {
    receive_parent_thread_ = std::thread([this]() { receive_parent_loop(); });
  }

  // create a wall timer for regulating the simulation clock
  reset_regulator_timer();

  // create a wall timer for real time pacing
  if (follows_real_time_) {
    reset_real_time_timer();
  }

  // create a wall timer for publishing the clock status
  clock_status_timer_ = node_.create_wall_timer(std::chrono::milliseconds(200), [this]() {
    on_clock_status_tick();
  });
  publish_status();
}

void TimeCoordinator::set_running(bool running)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_ == running) {
      return;
    }
    running_ = running;
    regulator_request_ns_ = sim_time_ns_;
    real_time_request_ns_ = sim_time_ns_;
    real_time_last_wall_time_ = std::chrono::steady_clock::now();
  }
  publish_status();
}

void TimeCoordinator::set_max_real_time_factor(double max_real_time_factor)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    max_real_time_factor_ = max_real_time_factor;
    regulator_request_ns_ = sim_time_ns_;
    real_time_request_ns_ = sim_time_ns_;
    real_time_last_wall_time_ = std::chrono::steady_clock::now();
  }
  reset_regulator_timer();
  publish_status();
}

fss_time_interfaces::msg::SimClockStatus TimeCoordinator::status_message() const
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
  switch (debug_state_) {
    case DebugState::Initializing: status.debug_msg = "Initializing..."; break;
    case DebugState::NoParticipants:
      status.debug_msg = "try_update_clock_locked: publish_clock_locked skipped: no participants";
      break;
    case DebugState::WaitingForParticipant:
      status.debug_msg = "try_update_clock_locked: publish_clock_locked skipped: waiting for participant";
      break;
    case DebugState::InfiniteRequest:
      status.debug_msg = "try_update_clock_locked: publish_clock_locked skipped: minimum request is infinite or near infinite";
      break;
    case DebugState::RequestNotAdvanced:
      status.debug_msg = "try_update_clock_locked: publish_clock_locked skipped: minimum request " +
        std::to_string(debug_min_request_ns_) + " ns is not greater than current sim time " +
        std::to_string(sim_time_ns_) + " ns";
      break;
    case DebugState::Advanced: status.debug_msg = "try_update_clock_locked: clock advanced"; break;
    case DebugState::ParentGrantPublished: status.debug_msg = "receive_parent_grant: clock advanced"; break;
  }
  status.speed_regulator_step_ns = speed_regulator_step_ns_;
  status.min_operation_walltime = min_operation_walltime_;
  return status;
}

std::size_t TimeCoordinator::clear_zombie_participants()
{
  std::size_t removed_count = 0;
  const auto now = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto participant = participants_.begin(); participant != participants_.end();) {
      const auto & state = participant->second;
      if (!state.has_new_request &&
        now - state.last_request_walltime >= zombie_participant_timeout_) {
        participant = participants_.erase(participant);
        ++removed_count;
      } else {
        ++participant;
      }
    }
    if (removed_count > 0) {
      try_update_clock_locked();
    }
  }
  if (removed_count > 0) {
    publish_status();
  }
  return removed_count;
}

void TimeCoordinator::receive_router_loop()
{
  while (!stop_receive_.load() && rclcpp::ok()) {
    try {
      zmq::pollitem_t poll_item{static_cast<void *>(impl_->socket), 0, ZMQ_POLLIN, 0};
      zmq::poll(&poll_item, 1, std::chrono::milliseconds(100));
      if ((poll_item.revents & ZMQ_POLLIN) != 0) {
        receive_router_message();
      }
    } catch (const zmq::error_t &) {
      if (!stop_receive_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      continue;
    }
  }
}

void TimeCoordinator::receive_parent_loop()
{
  while (!stop_receive_.load() && rclcpp::ok()) {
    try {
      zmq::pollitem_t poll_item{static_cast<void *>(impl_->parent_sub_socket), 0, ZMQ_POLLIN, 0};
      zmq::poll(&poll_item, 1, std::chrono::milliseconds(100));
      if ((poll_item.revents & ZMQ_POLLIN) != 0) {
        receive_parent_grant();
      }
    } catch (const zmq::error_t &) {
      if (!stop_receive_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
  }
}

void TimeCoordinator::receive_router_message()
{
  zmq::message_t identity_frame;
  zmq::message_t message_frame;
  const auto received = impl_->socket.recv(identity_frame, zmq::recv_flags::dontwait);
  if (!received) {
    return;
  }
  const auto received_message = impl_->socket.recv(message_frame, zmq::recv_flags::none);
  if (!received_message) {
    return;
  }

  const auto reply_text = handle_message(identity_frame.to_string(), message_frame.to_string());
  if (!reply_text.empty()) {
    // send() may take 2us, which is heavy for a time coordinator that is expected to run at 1kHz and with multiple participants. 
    zmq::message_t identity(identity_frame.data(), identity_frame.size());
    zmq::message_t reply(reply_text.begin(), reply_text.end());
    impl_->socket.send(identity, zmq::send_flags::sndmore);
    impl_->socket.send(reply, zmq::send_flags::none);
  }
}

void TimeCoordinator::receive_parent_grant()
{
  zmq::message_t grant_frame;
  const auto received = impl_->parent_sub_socket.recv(grant_frame, zmq::recv_flags::dontwait);
  if (!received) {
    return;
  }

  std::istringstream input(grant_frame.to_string());
  std::string command;
  int64_t granted_time_ns = 0;
  input >> command >> granted_time_ns;
  if (command != "GRANT" || !input) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (granted_time_ns <= sim_time_ns_) {
    return;
  }
  sim_time_ns_ = granted_time_ns;
  publish_clock_locked();
  debug_state_ = DebugState::ParentGrantPublished;

  for (auto & entry : participants_) {
    auto & participant = entry.second;
    if (participant.request_time_ns <= sim_time_ns_) {
      participant.has_new_request = false;
    }
  }
}

std::string TimeCoordinator::handle_message(const std::string & identity, const std::string & message)
{
  std::istringstream input(message);
  std::string command;
  input >> command;

  std::lock_guard<std::mutex> lock(mutex_);
  
  if (command == "REGISTER") {
    participants_[identity].last_request_walltime = std::chrono::steady_clock::now();
    return "OK";
  }

  if (command == "GET_PUB_ENDPOINT") {
    return "PUB_ENDPOINT " + pub_endpoint_;
  }

  if (command == "ANNOUNCE") {
    int64_t request_ns = 0;
    input >> request_ns;
    auto & participant = participants_[identity];
    participant.request_time_ns = request_ns;
    participant.has_new_request = true;
    participant.last_request_walltime = std::chrono::steady_clock::now();
    try_update_clock_locked();
    return ""; // no reply needed for ANNOUNCE
  }

  if (command == "SET_FOLLOWS_REAL_TIME") {
    int follows_real_time = 0;
    std::string extra;
    if (!(input >> follows_real_time) || (follows_real_time != 0 && follows_real_time != 1) ||
      (input >> extra)) {
      return "ERROR invalid follows_real_time setting";
    }
    const auto participant = participants_.find(identity);
    if (participant == participants_.end()) {
      return "ERROR participant is not registered";
    }
    participant->second.follows_real_time = follows_real_time == 1;
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

void TimeCoordinator::on_regulator_tick()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    regulator_request_ns_ = compute_regulator_target_ns_locked();
    try_update_clock_locked();
  }
}

void TimeCoordinator::on_real_time_tick()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!follows_real_time_) {
      return;
    }
    update_real_time_request_locked();
    try_update_clock_locked();
  }
}

void TimeCoordinator::on_clock_status_tick()
{
  {
    /* Publish /clock here, only for telling the lately added participants about the current time */
    std::lock_guard<std::mutex> lock(mutex_);
    update_observed_real_time_factor_locked();
    publish_clock_locked();
  }
  publish_status();
}

void TimeCoordinator::update_real_time_request_locked()
{
  const auto now = std::chrono::steady_clock::now();
  const auto real_time_paused =
    !follows_real_time_ || !running_ || max_real_time_factor_ <= 0.0;
  if (real_time_paused || real_time_last_wall_time_.time_since_epoch().count() == 0) {
    real_time_request_ns_ = sim_time_ns_;
    real_time_last_wall_time_ = now;
    return;
  }

  const auto wall_dt = now - real_time_last_wall_time_;
  const auto wall_dt_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(wall_dt).count();
  real_time_last_wall_time_ = now;
  if (wall_dt_ns <= 0) {
    return;
  }

  const auto base_ns = real_time_request_ns_; // last real time request 
  if (base_ns >= kInfiniteTimeNs - wall_dt_ns) {
    real_time_request_ns_ = kInfiniteTimeNs;
    return;
  }
  real_time_request_ns_ = base_ns + wall_dt_ns; // new real time request 
}

void TimeCoordinator::update_observed_real_time_factor_locked()
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

void TimeCoordinator::try_update_clock_locked()
{
  int64_t request_ns = 0;
  if (should_advance(request_ns)){
    advance_time_locked(request_ns);
    debug_state_ = DebugState::Advanced;

    // Clear the new request flags for participants whose requests have been satisfied.
    if (!has_parent_coordinator_) {
      for (auto & entry : participants_) {
        auto & participant = entry.second;
        if (participant.request_time_ns <= sim_time_ns_) {
          participant.has_new_request = false;
        }
      }
    }
  }
}

/**
 * @brief Determine whether the simulation time should be advanced based on participant requests and regulator/real-time constraints.
 * @param[out] output_request_time_ns The target simulation time to advance to if advancement is needed.
 * @return true if the simulation time should be advanced, false otherwise.
 */
bool TimeCoordinator::should_advance(int64_t & output_request_time_ns)
{
  if (!running_) {
    debug_state_ = DebugState::RequestNotAdvanced;
    return false; // do not advance if the coordinator is stopped
  }

  if (participants_.empty()) {
    debug_state_ = DebugState::NoParticipants;
    return false; // do not advance if there are no participants
  }

  // if follows real time, advance to the real time if it is greater than the current sim time and the regulator request time.
  if (follows_real_time_) {
    int64_t max_time_ns_follow_real_time = real_time_request_ns_;
    if (max_real_time_factor_ < 1.0) {
      // if max_real_time_factor_ < 1, the request should follow regulator instead of real time, as the sim time is limited by the regulator now.
      max_time_ns_follow_real_time = std::min(max_time_ns_follow_real_time, regulator_request_ns_);
    }
    if (max_time_ns_follow_real_time > sim_time_ns_) {
      // prepare to advance to real time
      for (const auto & entry : participants_) {
        const auto & participant = entry.second;
        if (!participant.follows_real_time && !participant.has_new_request) {
          debug_state_ = DebugState::WaitingForParticipant;
          return false; // do not advance if any participant that does not follow real time has not made a new request
        }
      }
      output_request_time_ns = max_time_ns_follow_real_time;
      return true; // advance to real time
    }
  }

  // get the minimum request time from all participants and the regulator
  int64_t min_request_ns = regulator_request_ns_;
  bool all_have_new_request = true;
  for (const auto & entry : participants_) {
    const auto & participant = entry.second;
    if (!participant.has_new_request) {
      all_have_new_request = false;
      break; // break if any participant has not made a new request
    }
    min_request_ns = std::min(min_request_ns, participant.request_time_ns);
  }

  if (!all_have_new_request) {
    debug_state_ = DebugState::WaitingForParticipant;
    return false; // do not advance if any participant has not made a new request
  }

  if (min_request_ns >= kInfiniteTimeNs - speed_regulator_step_ns_) {
    debug_state_ = DebugState::InfiniteRequest;
    return false; // do not advance if the minimum request is infinite or near infinite
  }

  if (min_request_ns <= sim_time_ns_) {
    debug_state_ = DebugState::RequestNotAdvanced;
    debug_min_request_ns_ = min_request_ns;
    return false; // do not advance if the minimum request is not greater than the current sim time
  }

  output_request_time_ns = min_request_ns;
  return true;
}

void TimeCoordinator::advance_time_locked(int64_t target_time_ns)
{
  if (has_parent_coordinator_) {
    // If we have a parent coordinator, we don't advance the time directly. Instead, we announce the next safe time to the parent coordinator and wait for a grant.
    if (parent_participant_) {
      enqueue_announce_task([this, target_time_ns]() {
        parent_participant_->announce_next_safe_time(target_time_ns);
      });
    }
  }
  else {
    // If we don't have a parent coordinator, we can advance the time directly.
    sim_time_ns_ = target_time_ns;
    publish_clock_locked();
  }
}

void TimeCoordinator::publish_clock_locked()
{
  const auto sim_time_ns = sim_time_ns_;
  enqueue_publish_task([this, sim_time_ns]() {
    publish_clock(sim_time_ns);
  });
}

void TimeCoordinator::publish_clock(int64_t sim_time_ns)
{
  if (publish_clock_ && clock_pub_) {
    rosgraph_msgs::msg::Clock clock;
    clock.clock = from_ns(sim_time_ns);
    clock_pub_->publish(clock);
  }
  publish_granted_time(sim_time_ns);
}

void TimeCoordinator::publish_granted_time(int64_t sim_time_ns)
{
  if (!impl_ || pub_endpoint_.empty()) {
    return;
  }
  std::ostringstream message;
  message << "GRANT " << sim_time_ns;
  try {
    impl_->pub_socket.send(zmq::buffer(message.str()), zmq::send_flags::dontwait);
  } catch (const zmq::error_t &) {
  }
}

void TimeCoordinator::enqueue_publish_task(std::function<void()> task)
{
  {
    std::lock_guard<std::mutex> lock(publish_mutex_);
    publish_tasks_.push(std::move(task));
  }
  publish_cv_.notify_one();
}

void TimeCoordinator::enqueue_announce_task(std::function<void()> task)
{
  {
    std::lock_guard<std::mutex> lock(announce_mutex_);
    announce_tasks_.push(std::move(task));
  }
  announce_cv_.notify_one();
}

void TimeCoordinator::start_async_worker()
{
  {
    std::lock_guard<std::mutex> lock(publish_mutex_);
    if (publish_worker_thread_.joinable() || announce_worker_thread_.joinable()) {
      return;
    }
    stop_async_worker_ = false;
  }
  publish_worker_thread_ = std::thread([this]() { publish_worker_loop(); });
  announce_worker_thread_ = std::thread([this]() { announce_worker_loop(); });
}

void TimeCoordinator::stop_async_worker()
{
  {
    std::lock_guard<std::mutex> lock(publish_mutex_);
    stop_async_worker_ = true;
  }
  publish_cv_.notify_one();
  announce_cv_.notify_one();
  if (publish_worker_thread_.joinable()) {
    publish_worker_thread_.join();
  }
  if (announce_worker_thread_.joinable()) {
    announce_worker_thread_.join();
  }
}

void TimeCoordinator::publish_worker_loop()
{
  while (rclcpp::ok()) {
    std::function<void()> task;
    {
      std::unique_lock<std::mutex> lock(publish_mutex_);
      publish_cv_.wait(lock, [this]() {
        return stop_async_worker_ || !publish_tasks_.empty();
      });
      if (stop_async_worker_ && publish_tasks_.empty()) {
        return;
      }
      task = std::move(publish_tasks_.front());
      publish_tasks_.pop();
    }

    if (task) {
      try {
        task();
      } catch (const std::exception & error) {
        RCLCPP_WARN(
          node_.get_logger(),
          "failed to execute asynchronous time coordinator task: %s",
          error.what());
      }
    }
  }
}

void TimeCoordinator::announce_worker_loop()
{
  while (rclcpp::ok()) {
    std::function<void()> task;
    {
      std::unique_lock<std::mutex> lock(announce_mutex_);
      announce_cv_.wait(lock, [this]() {
        return stop_async_worker_ || !announce_tasks_.empty();
      });
      if (stop_async_worker_ && announce_tasks_.empty()) {
        return;
      }
      task = std::move(announce_tasks_.front());
      announce_tasks_.pop();
    }
    if (task) {
      try {
        task();
      } catch (const std::exception & error) {
        RCLCPP_WARN(
          node_.get_logger(),
          "failed to execute asynchronous time coordinator task: %s",
          error.what());
      }
    }
  }
}

void TimeCoordinator::publish_status()
{
  status_pub_->publish(status_message());
}

int64_t TimeCoordinator::compute_regulator_target_ns_locked() const
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

void TimeCoordinator::reset_regulator_timer()
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

void TimeCoordinator::reset_real_time_timer()
{
  real_time_timer_.reset();
  if (!follows_real_time_) {
    return;
  }
  int64_t real_time_timer_period_ns = std::max(int64_t(speed_regulator_step_ns_ / 2), int64_t(1e6));  // minimum 1 ms
  real_time_timer_ = node_.create_wall_timer(
    std::chrono::nanoseconds(real_time_timer_period_ns),
    [this]() {
      on_real_time_tick();
    });
}

std::chrono::nanoseconds TimeCoordinator::regulator_wall_period() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (max_real_time_factor_ <= 0.0) {
    return std::chrono::nanoseconds::max();
  }

  const auto period_ns = static_cast<int64_t>(std::ceil(
      static_cast<double>(speed_regulator_step_ns_) / max_real_time_factor_));
  return std::chrono::nanoseconds(std::max<int64_t>(min_operation_walltime_, period_ns));
}

}  // namespace fss_time
