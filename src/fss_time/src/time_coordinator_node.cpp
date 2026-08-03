#include <memory>

#include "fss_time/time_coordinator.hpp"
#include "rclcpp/rclcpp.hpp"

class TimeCoordinatorNode : public rclcpp::Node
{
public:
  TimeCoordinatorNode()
  : Node("fss_time_coordinator"), coordinator_(*this)
  {
    coordinator_.start();
  }

private:
  fss_time::TimeCoordinator coordinator_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimeCoordinatorNode>());
  rclcpp::shutdown();
  return 0;
}
