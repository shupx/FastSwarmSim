#include <memory>

#include "fss_time_interfaces/msg/sim_clock_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

class RosClockPublisherNode : public rclcpp::Node
{
public:
  RosClockPublisherNode()
  : Node("fss_ros_clock_publisher")
  {
    clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());
    status_sub_ = create_subscription<fss_time_interfaces::msg::SimClockStatus>(
      "/fss/sim_clock_status",
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
      [this](const fss_time_interfaces::msg::SimClockStatus::SharedPtr msg) {
        rosgraph_msgs::msg::Clock clock;
        clock.clock = msg->sim_time;
        clock_pub_->publish(clock);
      });
  }

private:
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Subscription<fss_time_interfaces::msg::SimClockStatus>::SharedPtr status_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RosClockPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
