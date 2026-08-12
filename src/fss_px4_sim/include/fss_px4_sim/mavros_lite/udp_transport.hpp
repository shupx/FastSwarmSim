#pragma once

#include <arpa/inet.h>
#include <mavlink/v2.0/common/mavlink.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

namespace fss_px4_sim::mavros_lite
{

class MavlinkUdpTransport
{
public:
  using ReceiveCallback = std::function<void(const mavlink_message_t &)>;

  struct Config
  {
    uint16_t bind_port{24540};
    std::string remote_host{"127.0.0.1"};
    uint16_t remote_port{0};
  };

  MavlinkUdpTransport(
    const Config & config, rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock);
  ~MavlinkUdpTransport();

  MavlinkUdpTransport(const MavlinkUdpTransport &) = delete;
  MavlinkUdpTransport & operator=(const MavlinkUdpTransport &) = delete;

  void start();
  void stop();
  void set_receive_callback(ReceiveCallback callback);
  void send_message(const mavlink_message_t & message);

private:
  void receive_loop();

  Config config_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  int socket_fd_{-1};
  sockaddr_in remote_address_{};
  bool remote_learned_{false};
  std::atomic<bool> running_{false};
  std::thread receive_thread_;
  std::mutex mutex_;
  ReceiveCallback receive_callback_;
};

}  // namespace fss_px4_sim::mavros_lite
