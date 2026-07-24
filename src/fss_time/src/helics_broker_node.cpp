#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <helics/helics.h>
#include "rclcpp/rclcpp.hpp"

class HelicsBrokerNode : public rclcpp::Node
{
public:
  HelicsBrokerNode()
  : Node("fss_helics_broker")
  {
    const auto core_type = declare_parameter<std::string>("helics_core_type", "zmq");
    const auto broker_address = declare_parameter<std::string>("helics_broker_address", "127.0.0.1");
    const auto broker_port = declare_parameter<int>("helics_broker_port", 23404);
    const auto federates = declare_parameter<int>("helics_broker_federates", 0);

    std::ostringstream init;
    init << "--broker_address=" << broker_address
         << " --port=" << broker_port;
    if (federates > 0) {
      init << " --federates=" << federates;
    }

    HelicsError error = helicsErrorInitialize();
    broker_ = helicsCreateBroker(core_type.c_str(), "fss_time_broker", init.str().c_str(), &error);
    if (error.error_code != 0 || broker_ == nullptr) {
      const auto message = error.message == nullptr ? "unknown HELICS broker error" : error.message;
      throw std::runtime_error(message);
    }

    RCLCPP_INFO(
      get_logger(),
      "HELICS broker started with core_type=%s broker_address=%s broker_port=%d",
      core_type.c_str(),
      broker_address.c_str(),
      static_cast<int>(broker_port));
  }

  ~HelicsBrokerNode() override
  {
    if (broker_ != nullptr) {
      HelicsError error = helicsErrorInitialize();
      helicsBrokerDisconnect(broker_, &error);
      helicsBrokerFree(broker_);
      broker_ = nullptr;
    }
  }

private:
  HelicsBroker broker_{nullptr};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HelicsBrokerNode>());
  rclcpp::shutdown();
  return 0;
}
