#include "fss_px4_sim/mavros_lite/core.hpp"

#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace fss_px4_sim::mavros_lite
{

Core::Core()
: Node("mavros_lite_node")
{
  bind_port_ = static_cast<uint16_t>(declare_parameter<int>("udp.bind_port", 24540));
  remote_host_ = declare_parameter<std::string>("udp.remote_host", "127.0.0.1");
  remote_port_ = static_cast<uint16_t>(declare_parameter<int>("udp.remote_port", 0));
  target_system_ = static_cast<uint8_t>(declare_parameter<int>("target_system", 1));
  target_component_ = static_cast<uint8_t>(declare_parameter<int>("target_component", 1));
  system_id_ = static_cast<uint8_t>(declare_parameter<int>("system_id", 1));
  component_id_ = static_cast<uint8_t>(
    declare_parameter<int>("component_id", MAV_COMP_ID_ONBOARD_COMPUTER));

  add_module(make_imu(*this));
  add_module(make_local_position(*this));
  add_module(make_global_position(*this));
  add_module(make_setpoint_raw(*this));
  add_module(make_command(*this));
  add_module(make_sys_status(*this));
  start_transport();
}

Core::~Core()
{
  running_ = false;
  if (socket_fd_ >= 0) {
    shutdown(socket_fd_, SHUT_RDWR);
  }
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
  if (socket_fd_ >= 0) {
    close(socket_fd_);
  }
}

void Core::add_module(std::unique_ptr<Module> module)
{
  modules_.push_back(std::move(module));
}

void Core::start_transport()
{
  socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    throw std::runtime_error("unable to create MAVLink UDP socket");
  }

  sockaddr_in local{};
  local.sin_family = AF_INET;
  local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  local.sin_port = htons(bind_port_);
  if (bind(socket_fd_, reinterpret_cast<const sockaddr *>(&local), sizeof(local)) != 0) {
    throw std::runtime_error("unable to bind MAVLink UDP port " + std::to_string(bind_port_) +
      ": " + std::strerror(errno));
  }

  remote_address_.sin_family = AF_INET;
  remote_address_.sin_port = htons(remote_port_);
  if (inet_pton(AF_INET, remote_host_.c_str(), &remote_address_.sin_addr) != 1) {
    throw std::runtime_error("udp.remote_host must be an IPv4 address");
  }
  remote_learned_ = remote_port_ != 0;
  running_ = true;
  receive_thread_ = std::thread(&Core::receive_loop, this);
  RCLCPP_INFO(get_logger(), "MAVLink UDP listening on 127.0.0.1:%u", bind_port_);
}

void Core::send_message(mavlink_message_t & message)
{
  std::lock_guard<std::mutex> lock(send_mutex_);
  if (!remote_learned_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Waiting for PX4 MAVLink endpoint before sending");
    return;
  }
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  const uint16_t length = mavlink_msg_to_send_buffer(buffer, &message);
  const auto sent = sendto(
    socket_fd_, buffer, length, 0, reinterpret_cast<const sockaddr *>(&remote_address_),
    sizeof(remote_address_));
  if (sent != length) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "MAVLink UDP send failed");
  }
}

void Core::receive_loop()
{
  mavlink_status_t parse_status{};
  while (running_) {
    pollfd descriptor{socket_fd_, POLLIN, 0};
    const int ready = poll(&descriptor, 1, 200);
    if (ready <= 0) {
      continue;
    }
    uint8_t buffer[2048];
    sockaddr_in source{};
    socklen_t source_length = sizeof(source);
    const auto length = recvfrom(
      socket_fd_, buffer, sizeof(buffer), 0, reinterpret_cast<sockaddr *>(&source), &source_length);
    if (length <= 0) {
      continue;
    }
    for (ssize_t index = 0; index < length; ++index) {
      mavlink_message_t message{};
      if (!mavlink_parse_char(MAVLINK_COMM_0, buffer[index], &message, &parse_status)) {
        continue;
      }
      if (message.sysid != target_system_) {
        continue;
      }
      if (!remote_learned_) {
        std::lock_guard<std::mutex> lock(send_mutex_);
        remote_address_ = source;
        remote_learned_ = true;
        RCLCPP_INFO(
          get_logger(), "Learned PX4 MAVLink endpoint 127.0.0.1:%u", ntohs(source.sin_port));
      }
      for (auto & module : modules_) {
        module->handle_message(message);
      }
    }
  }
}

}  // namespace fss_px4_sim::mavros_lite
