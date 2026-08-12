#pragma once

#include <mavlink/v2.0/common/mavlink.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <thread>

class Px4MavlinkUdpTransport
{
public:
	using ReceiveCallback = std::function<void(const mavlink_message_t &)>;

	struct Config
	{
		uint16_t local_port{0};
		uint16_t remote_port{24540};
	};

	explicit Px4MavlinkUdpTransport(const Config &config);
	~Px4MavlinkUdpTransport();

	Px4MavlinkUdpTransport(const Px4MavlinkUdpTransport &) = delete;
	Px4MavlinkUdpTransport &operator=(const Px4MavlinkUdpTransport &) = delete;

	void start();
	void stop();
	void send_message(const mavlink_message_t &message) const;
	void set_receive_callback(ReceiveCallback callback);

private:
	void receive_loop();

	Config config_;
	ReceiveCallback receive_callback_;
	int socket_fd_{-1};
	std::atomic<bool> receiving_{false};
	std::thread receive_thread_;
};
