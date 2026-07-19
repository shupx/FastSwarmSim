#ifndef FSS_TIME_TIME_PARTICIPANT_HPP_
#define FSS_TIME_TIME_PARTICIPANT_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "fss_time/distributed_time_core.hpp"
#include "fss_time/time_transport.hpp"
#include "fss_time_interfaces/msg/time_control.hpp"
#include "fss_time_interfaces/msg/time_intent.hpp"
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
  void publish_intent(uint8_t state, int64_t next_safe_time_ns);
  void tick();
  void on_intent(const fss_time_interfaces::msg::TimeIntent & msg);
  void on_control(const fss_time_interfaces::msg::TimeControl & msg);

  rclcpp::Node & node_;
  std::string participant_id_;
  uint64_t epoch_{0};
  int64_t next_safe_time_ns_{0};
  uint8_t intent_state_{fss_time_interfaces::msg::TimeIntent::STATE_IDLE};
  std::chrono::milliseconds lease_timeout_;
  bool publish_clock_{false};
  std::chrono::steady_clock::time_point steady_start_;

  mutable std::mutex mutex_;
  DistributedTimeCore core_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  std::unique_ptr<TimeTransport> transport_;
};

}  // namespace fss_time

#endif  // FSS_TIME_TIME_PARTICIPANT_HPP_
