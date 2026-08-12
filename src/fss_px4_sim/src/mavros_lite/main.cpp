#include "fss_px4_sim/mavros_lite/core.hpp"
#include "fss_px4_sim/mavros_lite/udp_transport.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    using fss_px4_sim::mavros_lite::MavlinkUdpTransport;
    using fss_px4_sim::mavros_lite::MavrosLite;

    auto node = std::make_shared<rclcpp::Node>("mavros_lite_node");
    auto lite = std::make_shared<MavrosLite>(*node);
    MavlinkUdpTransport::Config config;
    config.bind_port = static_cast<uint16_t>(
      node->declare_parameter<int>("udp.bind_port", config.bind_port));
    config.remote_host = node->declare_parameter<std::string>(
      "udp.remote_host", config.remote_host);
    config.remote_port = static_cast<uint16_t>(
      node->declare_parameter<int>("udp.remote_port", config.remote_port));

    auto transport = std::make_shared<MavlinkUdpTransport>(
      config, node->get_logger(), node->get_clock());
    const std::weak_ptr<MavrosLite> lite_weak = lite;
    const std::weak_ptr<MavlinkUdpTransport> transport_weak = transport;
    transport->set_receive_callback([lite_weak](const mavlink_message_t & message) {
      if (const auto core = lite_weak.lock()) {
        core->receive_message(message);
      }
    });
    lite->set_send_callback([transport_weak](const mavlink_message_t & message) {
      if (const auto udp = transport_weak.lock()) {
        udp->send_message(message);
      }
    });
    transport->start();
    rclcpp::spin(node);
    transport->stop();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("mavros_lite"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
