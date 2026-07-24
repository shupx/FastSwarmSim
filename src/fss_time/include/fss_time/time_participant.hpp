#ifndef FSS_TIME_TIME_PARTICIPANT_HPP_
#define FSS_TIME_TIME_PARTICIPANT_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "fss_time/helics_time_coordinator.hpp"
#include "fss_time_interfaces/msg/time_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

namespace fss_time
{

class TimeParticipant
{
public:
  TimeParticipant(
    rclcpp::Node & node,
    std::string participant_id,
    double max_speed_ratio,
    std::chrono::milliseconds lease_timeout,
    bool publish_clock);

  void start();
  void announce_next_safe_time(const rclcpp::Time & next_safe_time);
  void announce_idle();
  void announce_leaving();
  void publish_control(const fss_time_interfaces::msg::TimeControl & control);
  rclcpp::Time now() const;

private:
  int64_t steady_now_ns() const;
  HelicsTimeOptions make_helics_options(double max_speed_ratio) const;
  void tick();

  rclcpp::Node & node_;
  std::string participant_id_;
  std::chrono::milliseconds lease_timeout_;
  bool publish_clock_{false};
  std::chrono::steady_clock::time_point steady_start_;

  mutable std::mutex mutex_;
  std::unique_ptr<HelicsTimeCoordinator> coordinator_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
};

}  // namespace fss_time

#endif  // FSS_TIME_TIME_PARTICIPANT_HPP_
