#include "fss_px4_sim/mavros_lite/udp_transport.hpp"

#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace fss_px4_sim::mavros_lite
{

MavlinkUdpTransport::MavlinkUdpTransport(
  const Config & config, rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
: config_(config), logger_(std::move(logger)), clock_(std::move(clock))
{
}

MavlinkUdpTransport::~MavlinkUdpTransport()
{
  stop();
}

void MavlinkUdpTransport::start()
{
  if (running_) {
    return;
  }
  socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    throw std::runtime_error("unable to create MAVLink UDP socket");
  }

  try {
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    local.sin_port = htons(config_.bind_port);
    if (bind(socket_fd_, reinterpret_cast<const sockaddr *>(&local), sizeof(local)) != 0) {
      throw std::runtime_error(
              "unable to bind MAVLink UDP port " + std::to_string(config_.bind_port) + ": " +
              std::strerror(errno));
    }

    remote_address_.sin_family = AF_INET;
    remote_address_.sin_port = htons(config_.remote_port);
    if (inet_pton(AF_INET, config_.remote_host.c_str(), &remote_address_.sin_addr) != 1) {
      throw std::runtime_error("udp.remote_host must be an IPv4 address");
    }
    remote_learned_ = config_.remote_port != 0;
    running_ = true;
    receive_thread_ = std::thread(&MavlinkUdpTransport::receive_loop, this);
    RCLCPP_INFO(logger_, "MAVLink UDP listening on 127.0.0.1:%u", config_.bind_port);
  } catch (...) {
    close(socket_fd_);
    socket_fd_ = -1;
    throw;
  }
}

void MavlinkUdpTransport::stop()
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
    socket_fd_ = -1;
  }
}

void MavlinkUdpTransport::set_receive_callback(ReceiveCallback callback)
{
  std::lock_guard<std::mutex> lock(mutex_);
  receive_callback_ = std::move(callback);
}

void MavlinkUdpTransport::send_message(const mavlink_message_t & message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!remote_learned_) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000, "Waiting for PX4 MAVLink endpoint before sending");
    return;
  }
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  auto mutable_message = message;
  const uint16_t length = mavlink_msg_to_send_buffer(buffer, &mutable_message);
  const auto sent = sendto(
    socket_fd_, buffer, length, 0, reinterpret_cast<const sockaddr *>(&remote_address_),
    sizeof(remote_address_));
  if (sent != length) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "MAVLink UDP send failed");
  }
}

void MavlinkUdpTransport::receive_loop()
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
      ReceiveCallback callback;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!remote_learned_) {
          remote_address_ = source;
          remote_learned_ = true;
          RCLCPP_INFO(
            logger_, "Learned PX4 MAVLink endpoint 127.0.0.1:%u", ntohs(source.sin_port));
        }
        callback = receive_callback_;
      }
      if (callback) {
        callback(message);
      }
    }
  }
}

}  // namespace fss_px4_sim::mavros_lite
