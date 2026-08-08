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
  /// Unique participant id sent to the coordinator.
  std::string participant_id;

  /// ZeroMQ endpoint for the coordinator. Supports any endpoint accepted by zmq::socket_t::connect().
  std::string coordinator_endpoint{"ipc:///tmp/fss_time_coordinator.ipc"};

};

/**
 * @brief ZeroMQ client backend for a thread time participant.
 */
class ZeroMqTimeParticipantBackend
{
public:
  /**
   * @brief Construct a participant backend.
   * @param options Participant id and coordinator endpoint options.
   * @param clock Clock used to track coordinator time.
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
   * @brief Send the next safe simulation time request to the coordinator.
   * @param next_safe_time_ns Requested next safe time in nanoseconds.
   */
  void announce_next_safe_time(int64_t next_safe_time_ns);

  /**
   * @brief Register this participant with the coordinator.
   */
  void register_participant();

  /**
   * @brief Unregister this participant from the coordinator.
   */
  void unregister_participant();

  /**
   * @brief Set whether this participant follows real time.
   *
   * Registers this participant with the coordinator when needed, then updates
   * the setting immediately.
   * @param follows_real_time true to apply the coordinator real-time floor.
   * @throws std::runtime_error if the coordinator rejects the update.
   */
  void set_follows_real_time(bool follows_real_time);

  /**
   * @brief Return the latest coordinator time observed by this backend.
   * @return Simulation time in nanoseconds.
   */
  int64_t current_time_ns() const;

  /**
   * @brief Return the last safe time request sent to the coordinator.
   * @return Last requested simulation time in nanoseconds.
   */
  int64_t last_requested_time_ns() const;

  /**
   * @brief Check whether this backend is registered with the coordinator.
   * @return true if registered.
   */
  bool is_registered() const;

  /**
   * @brief Return this backend's participant id.
   * @return Participant id string.
   */
  const std::string & participant_id() const;

  /**
   * @brief Send a raw coordinator request and return its reply.
   * @param message Request message.
   * @param wait_forever true to retry until a reply is received.
   * @return Coordinator reply text, or an empty string if no reply was received.
   */
  std::string request_coordinator(const std::string & message, bool wait_forever);

private:
  void start_locked();
  bool send_coordinator_locked(const std::string & message, bool wait_forever);
  std::string request_coordinator_locked(const std::string & message, bool wait_forever);

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
