#ifndef FSS_TIME_THREAD_TIME_PARTICIPANT_HPP_
#define FSS_TIME_THREAD_TIME_PARTICIPANT_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "fss_time/zeromq_time_participant_backend.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fss_time
{

class thread_time_participant
{
public:
  static thread_time_participant & for_current_thread(
    rclcpp::Node & node,
    const std::string & participant_id_hint = "");
#ifdef BUILD_TESTING
  static void reset_current_thread_for_testing();
#endif

  ~thread_time_participant();

  void announce_next_safe_time(const rclcpp::Time & next_safe_time);
  void unregister_participant();
  rclcpp::Time get_sim_time() const;
  rclcpp::Time get_last_safe_time() const;

private:
  explicit thread_time_participant(std::shared_ptr<ZeroMqTimeParticipantBackend> backend);

  std::shared_ptr<ZeroMqTimeParticipantBackend> backend_;
  mutable std::mutex mutex_;
  int64_t last_safe_time_ns_{0};
};

}  // namespace fss_time

#endif  // FSS_TIME_THREAD_TIME_PARTICIPANT_HPP_
