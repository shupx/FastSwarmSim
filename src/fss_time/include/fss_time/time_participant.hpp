#ifndef FSS_TIME_TIME_PARTICIPANT_HPP_
#define FSS_TIME_TIME_PARTICIPANT_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "fss_time/thread_time_participant.hpp"

namespace fss_time
{

class TimeParticipant
{
public:
  TimeParticipant(
    rclcpp::Node & node,
    std::string participant_id,
    double,
    std::chrono::milliseconds,
    bool)
  : node_(node), participant_id_(std::move(participant_id))
  {
  }

  void start()
  {
    participant_ = &thread_time_participant::for_current_thread(node_, participant_id_);
  }

  void announce_next_safe_time(const rclcpp::Time & next_safe_time)
  {
    ensure_started();
    participant_->announce_next_safe_time(next_safe_time);
  }

  rclcpp::Time now() const
  {
    ensure_started();
    return participant_->get_sim_time();
  }

private:
  void ensure_started() const
  {
    if (participant_ == nullptr) {
      const_cast<TimeParticipant *>(this)->participant_ =
        &thread_time_participant::for_current_thread(node_, participant_id_);
    }
  }

  rclcpp::Node & node_;
  std::string participant_id_;
  thread_time_participant * participant_{nullptr};
};

}  // namespace fss_time

#endif  // FSS_TIME_TIME_PARTICIPANT_HPP_
