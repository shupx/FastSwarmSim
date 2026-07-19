#include <chrono>
#include <memory>
#include <string>

#include "fss_time/time_participant.hpp"
#include "fss_time/time_types.hpp"
#include "fss_time_interfaces/msg/time_control.hpp"
#include "fss_time_interfaces/srv/sim_clock_control.hpp"
#include "rclcpp/rclcpp.hpp"

class ClockBridgeNode : public rclcpp::Node
{
public:
  ClockBridgeNode()
  : Node("fss_clock_bridge")
  {
    const auto participant_id = declare_parameter<std::string>("participant_id", "clock_bridge");
    const auto max_speed_ratio = declare_parameter<double>("max_speed_ratio", 1.0);
    const auto lease_timeout_ms = declare_parameter<int>("lease_timeout_ms", 1000);
    participant_ = std::make_unique<fss_time::TimeParticipant>(
      *this,
      participant_id,
      max_speed_ratio,
      std::chrono::milliseconds(lease_timeout_ms),
      true);
    participant_->start();
    clock_control_srv_ = create_service<fss_time_interfaces::srv::SimClockControl>(
      "/fss/clock_control",
      [this](
        const std::shared_ptr<fss_time_interfaces::srv::SimClockControl::Request> request,
        std::shared_ptr<fss_time_interfaces::srv::SimClockControl::Response> response) {
        fss_time_interfaces::msg::TimeControl control;
        control.epoch = ++control_epoch_;
        if (request->max_sim_speed != 0.0f) {
          control.command = fss_time_interfaces::msg::TimeControl::COMMAND_SET_SPEED;
          control.max_speed_ratio = request->max_sim_speed;
          participant_->publish_control(control);
          control.epoch = ++control_epoch_;
        }
        control.command = request->proceed ?
          fss_time_interfaces::msg::TimeControl::COMMAND_RESUME :
          fss_time_interfaces::msg::TimeControl::COMMAND_PAUSE;
        participant_->publish_control(control);
        response->success = true;
      });

    RCLCPP_INFO(
      get_logger(),
      "FastSwarm distributed clock bridge started as '%s' with max_speed_ratio %.3f using eCAL transport",
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
  std::unique_ptr<fss_time::TimeParticipant> participant_;
  rclcpp::Service<fss_time_interfaces::srv::SimClockControl>::SharedPtr clock_control_srv_;
  uint64_t control_epoch_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClockBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
