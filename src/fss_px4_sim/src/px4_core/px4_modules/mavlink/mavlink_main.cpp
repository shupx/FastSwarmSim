/**
 * @file mavlink_main.cpp
 * @brief UDP-based MAVLink module entry point for the PX4 simulation.
 *
 * The module owns the simulated MAVLink receiver and streamer. It sends
 * outgoing messages through a connected loopback UDP socket and processes
 * incoming messages on a dedicated thread.
 *
 * @author Peixuan Shu
 * @date 2026-08-08
 * Modified by Peixuan Shu (2026-08-09): scope the MAVLink module and receiver
 * thread to their owning PX4 instance context.
 */

#include "mavlink_main.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <netinet/in.h>
#include <poll.h>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>

MAVLINK::MAVLINK(MavrosQuadSimulator::Px4InstanceContext &context,
	int local_port, int remote_port)
	: context_(context), local_port_(local_port), remote_port_(remote_port)
{
}

MAVLINK::~MAVLINK()
{
	stop();
}

void MAVLINK::start()
{
	// Modified by Peixuan Shu: construct MAVLink components in their owning context.
	MavrosQuadSimulator::Px4InstanceContext::Scope context_scope(context_);
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
		local.sin_port = htons(static_cast<uint16_t>(local_port_));
		if (::bind(socket_fd_, reinterpret_cast<const sockaddr *>(&local), sizeof(local)) < 0) {
			throw std::runtime_error("failed to bind MAVLink UDP socket");
		}

		sockaddr_in remote{};
		remote.sin_family = AF_INET;
		remote.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
		remote.sin_port = htons(static_cast<uint16_t>(remote_port_));
		if (::connect(socket_fd_, reinterpret_cast<const sockaddr *>(&remote), sizeof(remote)) < 0) {
			throw std::runtime_error("failed to connect MAVLink UDP socket");
		}

		receiver_ = std::make_unique<MavlinkReceiver>();
		streamer_ = std::make_unique<MavlinkStreamer>([this](const mavlink_message_t &message) {
			send_message(message);
		});
		receiving_.store(true);
		receive_thread_ = std::thread(&MAVLINK::receive_loop, this);
	} catch (...) {
		receiving_.store(false);
		streamer_.reset();
		receiver_.reset();
		::close(socket_fd_);
		socket_fd_ = -1;
		throw;
	}
}

void MAVLINK::stop()
{
	receiving_.store(false);
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
	streamer_.reset();
	receiver_.reset();
}

void MAVLINK::Stream(const uint64_t &time_us)
{
	// Added by Peixuan Shu: keep direct MAVLINK callers in the owning context.
	MavrosQuadSimulator::Px4InstanceContext::Scope context_scope(context_);
	if (streamer_) {
		streamer_->Stream(time_us);
	}
}

void MAVLINK::send_message(const mavlink_message_t &message) const
{
	if (socket_fd_ < 0) {
		return;
	}

	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	auto mutable_message = message;
	const auto length = mavlink_msg_to_send_buffer(buffer, &mutable_message);
	(void)::send(socket_fd_, buffer, length, MSG_DONTWAIT);
}

void MAVLINK::receive_loop()
{
	// Modified by Peixuan Shu: the receiver executes on another thread, so it
	// must select the complete owning context explicitly.
	MavrosQuadSimulator::Px4InstanceContext::Scope context_scope(context_);
	mavlink_status_t status{};
	uint8_t buffer[2048];
	pollfd poll_fd{socket_fd_, POLLIN, 0};

	while (receiving_.load()) {
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
			if (mavlink_parse_char(MAVLINK_COMM_0, buffer[index], &message, &status)) {
				receiver_->handle_message(&message);
			}
		}
	}
}
