#include <memory>

#include "fss_time/sim_time_broker.hpp"
#include "rclcpp/rclcpp.hpp"

class SimTimeBrokerNode : public rclcpp::Node
{
public:
  SimTimeBrokerNode()
  : Node("fss_sim_time_broker"), broker_(*this)
  {
    broker_.start();
  }

private:
  fss_time::SimTimeBroker broker_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimTimeBrokerNode>());
  rclcpp::shutdown();
  return 0;
}
