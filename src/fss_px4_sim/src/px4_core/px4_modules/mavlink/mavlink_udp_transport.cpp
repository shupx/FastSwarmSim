#include "mavlink_udp_transport.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <netinet/in.h>
#include <poll.h>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>

Px4MavlinkUdpTransport::Px4MavlinkUdpTransport(const Config &config)
	: config_(config)
{
}

Px4MavlinkUdpTransport::~Px4MavlinkUdpTransport()
{
	stop();
}

void Px4MavlinkUdpTransport::start()
{
	if (socket_fd_ >= 0) {
		return;
	}
	socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
	if (socket_fd_ < 0) {
		throw std::runtime_error("failed to create MAVLink UDP socket");
	}

	try {
		sockaddr_in local{};
		local.sin_family = AF_INET;
		local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
		local.sin_port = htons(config_.local_port);
		if (::bind(socket_fd_, reinterpret_cast<const sockaddr *>(&local), sizeof(local)) < 0) {
			throw std::runtime_error("failed to bind MAVLink UDP socket");
		}

		sockaddr_in remote{};
		remote.sin_family = AF_INET;
		remote.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
		remote.sin_port = htons(config_.remote_port);
		if (::connect(socket_fd_, reinterpret_cast<const sockaddr *>(&remote), sizeof(remote)) < 0) {
			throw std::runtime_error("failed to connect MAVLink UDP socket");
		}

		receiving_ = true;
		receive_thread_ = std::thread(&Px4MavlinkUdpTransport::receive_loop, this);
	} catch (...) {
		::close(socket_fd_);
		socket_fd_ = -1;
		throw;
	}
}

void Px4MavlinkUdpTransport::stop()
{
	receiving_ = false;
	if (socket_fd_ >= 0) {
		(void)::shutdown(socket_fd_, SHUT_RDWR);
	}
	if (receive_thread_.joinable()) {
		receive_thread_.join();
	}
	if (socket_fd_ >= 0) {
		::close(socket_fd_);
		socket_fd_ = -1;
	}
}

void Px4MavlinkUdpTransport::send_message(const mavlink_message_t &message) const
{
	if (socket_fd_ < 0) {
		return;
	}
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	auto mutable_message = message;
	const auto length = mavlink_msg_to_send_buffer(buffer, &mutable_message);
	(void)::send(socket_fd_, buffer, length, MSG_DONTWAIT);
}

void Px4MavlinkUdpTransport::set_receive_callback(ReceiveCallback callback)
{
	receive_callback_ = std::move(callback);
}

void Px4MavlinkUdpTransport::receive_loop()
{
	mavlink_status_t status{};
	uint8_t buffer[2048];
	pollfd poll_fd{socket_fd_, POLLIN, 0};

	while (receiving_) {
		const int poll_result = ::poll(&poll_fd, 1, 100);
		if (poll_result == 0) {
			continue;
		}
		if (poll_result < 0) {
			if (errno == EINTR) {
				continue;
			}
			break;
		}
		if ((poll_fd.revents & POLLIN) == 0) {
			continue;
		}

		const auto length = ::recv(socket_fd_, buffer, sizeof(buffer), 0);
		if (length < 0) {
			if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
				continue;
			}
			break;
		}

		for (ssize_t index = 0; index < length; ++index) {
			mavlink_message_t message{};
			if (mavlink_parse_char(MAVLINK_COMM_0, buffer[index], &message, &status) &&
				receive_callback_) {
				receive_callback_(message);
			}
		}
	}
}
