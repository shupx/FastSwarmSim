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
  /// Unique participant id sent to the broker.
  std::string participant_id;

  /// ZeroMQ endpoint for the broker. Supports any endpoint accepted by zmq::socket_t::connect().
  std::string ipc_endpoint{"ipc:///tmp/fss_time_broker.ipc"};
};

/**
 * @brief ZeroMQ client backend for a thread time participant.
 */
class ZeroMqTimeParticipantBackend
{
public:
  /**
   * @brief Construct a participant backend.
   * @param options Participant id and broker endpoint options.
   * @param clock Clock used to track broker time.
   */
  ZeroMqTimeParticipantBackend(
    ZeroMqTimeParticipantOptions options,
    rclcpp::Clock::SharedPtr clock);

  /**
   * @brief Unregister and release backend resources.
   */
  ~ZeroMqTimeParticipantBackend();

  /**
   * @brief Copy construction is disabled.
   */
  ZeroMqTimeParticipantBackend(const ZeroMqTimeParticipantBackend &) = delete;

  /**
   * @brief Copy assignment is disabled.
   */
  ZeroMqTimeParticipantBackend & operator=(const ZeroMqTimeParticipantBackend &) = delete;

  /**
   * @brief Send the next safe simulation time request to the broker.
   * @param next_safe_time_ns Requested next safe time in nanoseconds.
   */
  void announce_next_safe_time(int64_t next_safe_time_ns);

  /**
   * @brief Register this participant with the broker.
   */
  void register_participant();

  /**
   * @brief Unregister this participant from the broker.
   */
  void unregister_participant();

  /**
   * @brief Return the latest broker time observed by this backend.
   * @return Simulation time in nanoseconds.
   */
  int64_t current_time_ns() const;

  /**
   * @brief Return the last safe time request sent to the broker.
   * @return Last requested simulation time in nanoseconds.
   */
  int64_t last_requested_time_ns() const;

  /**
   * @brief Check whether this backend is registered with the broker.
   * @return true if registered.
   */
  bool is_registered() const;

  /**
   * @brief Return this backend's participant id.
   * @return Participant id string.
   */
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
