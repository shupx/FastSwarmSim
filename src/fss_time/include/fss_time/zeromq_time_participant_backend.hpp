#ifndef FSS_TIME_ZEROMQ_TIME_PARTICIPANT_BACKEND_HPP_
#define FSS_TIME_ZEROMQ_TIME_PARTICIPANT_BACKEND_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace fss_time
{

struct ZeroMqTimeParticipantOptions
{
  std::string participant_id;
  std::string ipc_endpoint{"ipc:///tmp/fss_time_broker.ipc"};
};

class ZeroMqTimeParticipantBackend
{
public:
  ZeroMqTimeParticipantBackend(
    ZeroMqTimeParticipantOptions options,
    rclcpp::Clock::SharedPtr clock);
  ~ZeroMqTimeParticipantBackend();

  ZeroMqTimeParticipantBackend(const ZeroMqTimeParticipantBackend &) = delete;
  ZeroMqTimeParticipantBackend & operator=(const ZeroMqTimeParticipantBackend &) = delete;

  void announce_next_safe_time(int64_t next_safe_time_ns);
  void register_participant();
  void unregister_participant();

  int64_t current_time_ns() const;
  int64_t last_requested_time_ns() const;
  bool is_registered() const;
  const std::string & participant_id() const;

private:
  void start_locked();
  bool request_broker_locked(const std::string & message, bool wait_forever);

  struct Impl;

  ZeroMqTimeParticipantOptions options_;
  rclcpp::Clock::SharedPtr clock_;
  std::unique_ptr<Impl> impl_;
  mutable std::mutex mutex_;
  int64_t last_requested_time_ns_{0};
  bool connected_{false};
  bool registered_{false};
};

}  // namespace fss_time

#endif  // FSS_TIME_ZEROMQ_TIME_PARTICIPANT_BACKEND_HPP_
