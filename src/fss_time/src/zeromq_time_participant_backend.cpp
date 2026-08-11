#include "fss_time/zeromq_time_participant_backend.hpp"

#include <chrono>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

#include <zmq.hpp>

namespace fss_time
{

struct ZeroMqTimeParticipantBackend::Impl
{
  zmq::context_t context{1};
  zmq::socket_t socket{context, zmq::socket_type::dealer};
};

ZeroMqTimeParticipantBackend::ZeroMqTimeParticipantBackend(
  ZeroMqTimeParticipantOptions options,
  rclcpp::Clock::SharedPtr clock)
: options_(std::move(options)), clock_(std::move(clock)), impl_(std::make_unique<Impl>())
{
}

ZeroMqTimeParticipantBackend::~ZeroMqTimeParticipantBackend()
{
  unregister_participant();
}

void ZeroMqTimeParticipantBackend::announce_next_safe_time(int64_t next_safe_time_ns)
{
  std::lock_guard<std::mutex> lock(mutex_);
  start_locked();
  if (!registered_) {
    if (request_coordinator_locked("REGISTER", true) != "OK") {
      throw std::runtime_error("failed to register fss_time participant");
    }
    registered_ = true;
  }
  last_requested_time_ns_ = next_safe_time_ns;

  std::ostringstream message;
  message << "ANNOUNCE " << last_requested_time_ns_;
  if (!send_coordinator_locked(message.str(), true)) {
    throw std::runtime_error("failed to announce fss_time participant request");
  }
}

void ZeroMqTimeParticipantBackend::register_participant()
{
  std::lock_guard<std::mutex> lock(mutex_);
  start_locked();
  if (registered_) {
    return;
  }
  if (request_coordinator_locked("REGISTER", true) != "OK") {
    throw std::runtime_error("failed to register fss_time participant");
  }
  registered_ = true;
}

void ZeroMqTimeParticipantBackend::unregister_participant()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!registered_) {
    return;
  }

  start_locked();
  // ROS shutdown makes rclcpp::ok() false before thread-local destructors run.
  // Send UNREGISTER best-effort without waiting for a reply in that case.
  send_coordinator_locked("UNREGISTER", false, true);
  registered_ = false;
}

void ZeroMqTimeParticipantBackend::set_follows_real_time(bool follows_real_time)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!registered_) {
    start_locked();
    if (request_coordinator_locked("REGISTER", true) != "OK") {
      throw std::runtime_error("failed to register fss_time participant");
    }
    registered_ = true;
  }

  const std::string message =
    std::string("SET_FOLLOWS_REAL_TIME ") + (follows_real_time ? "1" : "0");
  if (request_coordinator_locked(message, true) != "OK") {
    throw std::runtime_error("failed to update fss_time participant follows_real_time setting");
  }
}

int64_t ZeroMqTimeParticipantBackend::current_time_ns() const
{
  if (clock_) {
    return clock_->now().nanoseconds();
  }
  return 0;
}

int64_t ZeroMqTimeParticipantBackend::last_requested_time_ns() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_requested_time_ns_;
}

bool ZeroMqTimeParticipantBackend::is_registered() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return registered_;
}

const std::string & ZeroMqTimeParticipantBackend::participant_id() const
{
  return options_.participant_id;
}

std::string ZeroMqTimeParticipantBackend::request_coordinator(
  const std::string & message,
  bool wait_forever)
{
  std::lock_guard<std::mutex> lock(mutex_);
  start_locked();
  return request_coordinator_locked(message, wait_forever);
}

void ZeroMqTimeParticipantBackend::start_locked()
{
  if (connected_) {
    return;
  }

  impl_->socket.set(zmq::sockopt::linger, 0);
  impl_->socket.set(zmq::sockopt::sndtimeo, 100);
  impl_->socket.set(zmq::sockopt::rcvtimeo, 100);
  impl_->socket.set(zmq::sockopt::routing_id, options_.participant_id);
  impl_->socket.connect(options_.coordinator_endpoint);
  connected_ = true;
}

// only send, no receive, because the coordinator may not respond to all requests
// TODO: only send() does not guarantee that the message is actually received by the coordinator (e.g. if the network is down temporarily). Consider adding a hearbeat mechanism to detect if the coordinator is alive and connected, and keep resending if the coordinator is offline temporarily. 
bool ZeroMqTimeParticipantBackend::send_coordinator_locked(
  const std::string & message,
  bool wait_forever,
  bool allow_after_shutdown)
{
  constexpr auto retry_period = std::chrono::milliseconds(200);
  int attempts = 0;
  while ((wait_forever || attempts < 3) && (allow_after_shutdown || rclcpp::ok())) {
    ++attempts;
    try {
      zmq::message_t request(message.begin(), message.end());
      const auto sent = impl_->socket.send(request, zmq::send_flags::none);
      if (sent) {
        return true;
      }
    } catch (const zmq::error_t &) {
    }
    if (attempts >= 5 && attempts % 5 == 0) {
      std::cerr
        << "\033[33m[fss_time::ZeroMqTimeParticipantBackend] waiting "
        << attempts * retry_period.count() / 1000 << " s to send coordinator request \""
        << message << "\" to " << options_.coordinator_endpoint << ".\033[0m" << std::endl;
    }
    std::this_thread::sleep_for(retry_period);
  }

  return false;
}

std::string ZeroMqTimeParticipantBackend::request_coordinator_locked(
  const std::string & message,
  bool wait_forever)
{
  constexpr auto retry_period = std::chrono::milliseconds(200);
  int attempts = 0;
  bool request_sent = false;
  while ((wait_forever || attempts < 3) && rclcpp::ok()) {
    ++attempts;
    try {
      if (!request_sent) {
        // DEALER replies are not correlated with requests; resending here can leave a stale reply
        // that the next request would consume.
        zmq::message_t request(message.begin(), message.end());
        const auto sent = impl_->socket.send(request, zmq::send_flags::none);
        if (!sent) {
          std::this_thread::sleep_for(retry_period);
          continue;
        }
        request_sent = true;
      }

      zmq::message_t reply;
      const auto received = impl_->socket.recv(reply, zmq::recv_flags::none);
      if (received) {
        return reply.to_string();
      }
    } catch (const zmq::error_t &) {
    }
    if (attempts >= 5 && attempts % 5 == 0) {
      std::cerr
        << "\033[33m[fss_time::ZeroMqTimeParticipantBackend] waiting "
        << attempts * retry_period.count() / 1000 << " s for coordinator response to \"" << message
        << "\" from " << options_.coordinator_endpoint << ".\033[0m" << std::endl;
    }
    std::this_thread::sleep_for(retry_period);
  }

  return "";
}

}  // namespace fss_time
