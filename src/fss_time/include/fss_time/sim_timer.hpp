#ifndef FASTSWARM_TIME_SIM_TIMER_HPP_
#define FASTSWARM_TIME_SIM_TIMER_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include "fss_time/time_participant.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fss_time
{

class SimRate
{
public:
  SimRate(TimeParticipant & participant, rclcpp::Duration period)
  : participant_(participant), period_(period), next_time_(participant.now() + period)
  {
  }

  void sleep()
  {
    participant_.announce_next_safe_time(next_time_);
    while (rclcpp::ok() && participant_.now() < next_time_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    next_time_ = next_time_ + period_;
  }

private:
  TimeParticipant & participant_;
  rclcpp::Duration period_;
  rclcpp::Time next_time_;
};

class SimSleep
{
public:
  static void sleep_for(TimeParticipant & participant, const rclcpp::Duration & duration)
  {
    const auto end = participant.now() + duration;
    participant.announce_next_safe_time(end);
    while (rclcpp::ok() && participant.now() < end) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
};

}  // namespace fss_time

#endif  // FASTSWARM_TIME_SIM_TIMER_HPP_
