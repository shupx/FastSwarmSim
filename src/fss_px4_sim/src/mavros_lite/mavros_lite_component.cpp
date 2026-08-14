#include "fss_px4_sim/mavros_lite/core.hpp"
#include "fss_px4_sim/mavros_lite/udp_transport.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace fss_px4_sim::mavros_lite
{

class MavrosLiteUdp final : public rclcpp::Node
{
public:
  explicit MavrosLiteUdp(const rclcpp::NodeOptions & options)
  : Node("mavros_lite_node", options)
  {
    lite_ = std::make_shared<MavrosLite>(*this);
    MavlinkUdpTransport::Config config;
    config.bind_port = static_cast<uint16_t>(
      declare_parameter<int>("udp.bind_port", config.bind_port));
    config.remote_host = declare_parameter<std::string>(
      "udp.remote_host", config.remote_host);
    config.remote_port = static_cast<uint16_t>(
      declare_parameter<int>("udp.remote_port", config.remote_port));

    transport_ = std::make_shared<MavlinkUdpTransport>(
      config, get_logger(), get_clock());
    const std::weak_ptr<MavrosLite> lite_weak = lite_;
    const std::weak_ptr<MavlinkUdpTransport> transport_weak = transport_;
    transport_->set_receive_callback([lite_weak](const mavlink_message_t & message) {
      if (const auto lite = lite_weak.lock()) {
        lite->receive_message(message);
      }
    });
    lite_->set_send_callback([transport_weak](const mavlink_message_t & message) {
      if (const auto transport = transport_weak.lock()) {
        transport->send_message(message);
      }
    });
    transport_->start();
  }

  ~MavrosLiteUdp() override
  {
    if (transport_) {
      transport_->stop();
    }
  }

private:
  std::shared_ptr<MavrosLite> lite_;
  std::shared_ptr<MavlinkUdpTransport> transport_;
};

}  // namespace fss_px4_sim::mavros_lite

RCLCPP_COMPONENTS_REGISTER_NODE(fss_px4_sim::mavros_lite::MavrosLiteUdp)
