#include <chrono>
#include <memory>
#include <string>

#include "fastswarm_time/time_participant.hpp"
#include "fastswarm_time/time_types.hpp"
#include "fastswarm_time_interfaces/msg/time_control.hpp"
#include "fastswarm_time_interfaces/srv/sim_clock_control.hpp"
#include "rclcpp/rclcpp.hpp"

class ClockBridgeNode : public rclcpp::Node
{
public:
  ClockBridgeNode()
  : Node("fastswarm_clock_bridge")
  {
    const auto participant_id = declare_parameter<std::string>("participant_id", "clock_bridge");
    const auto max_speed_ratio = declare_parameter<double>("max_speed_ratio", 1.0);
    const auto lease_timeout_ms = declare_parameter<int>("lease_timeout_ms", 1000);
    participant_ = std::make_unique<fastswarm_time::TimeParticipant>(
      *this,
      participant_id,
      max_speed_ratio,
      std::chrono::milliseconds(lease_timeout_ms),
      true);
    participant_->start();
    control_pub_ = create_publisher<fastswarm_time_interfaces::msg::TimeControl>(
      "/fastswarm/time_control", rclcpp::QoS(rclcpp::KeepLast(20)).reliable().transient_local());
    clock_control_srv_ = create_service<fastswarm_time_interfaces::srv::SimClockControl>(
      "/fastswarm/clock_control",
      [this](
        const std::shared_ptr<fastswarm_time_interfaces::srv::SimClockControl::Request> request,
        std::shared_ptr<fastswarm_time_interfaces::srv::SimClockControl::Response> response) {
        fastswarm_time_interfaces::msg::TimeControl control;
        control.epoch = ++control_epoch_;
        if (request->max_sim_speed != 0.0f) {
          control.command = fastswarm_time_interfaces::msg::TimeControl::COMMAND_SET_SPEED;
          control.max_speed_ratio = request->max_sim_speed;
          control_pub_->publish(control);
          control.epoch = ++control_epoch_;
        }
        control.command = request->proceed ?
          fastswarm_time_interfaces::msg::TimeControl::COMMAND_RESUME :
          fastswarm_time_interfaces::msg::TimeControl::COMMAND_PAUSE;
        control_pub_->publish(control);
        response->success = true;
      });

    RCLCPP_INFO(
      get_logger(),
      "FastSwarm distributed clock bridge started as '%s' with max_speed_ratio %.3f",
      participant_id.c_str(),
      max_speed_ratio);
  }

  ~ClockBridgeNode() override
  {
    if (participant_) {
      participant_->announce_leaving();
    }
  }

private:
  std::unique_ptr<fastswarm_time::TimeParticipant> participant_;
  rclcpp::Publisher<fastswarm_time_interfaces::msg::TimeControl>::SharedPtr control_pub_;
  rclcpp::Service<fastswarm_time_interfaces::srv::SimClockControl>::SharedPtr clock_control_srv_;
  uint64_t control_epoch_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClockBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
