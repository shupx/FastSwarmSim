#pragma once

#include <functional>
#include <utility>

#include <mavlink/v2.0/common/mavlink.h>

class MavlinkSender
{
public:
	// added or modified by Peixuan Shu: keep the MAVLink source IDs with the
	// sender so each stream can encode packets for its owning PX4 instance.
	MavlinkSender(std::function<void(const mavlink_message_t &)> send = {},
		uint8_t system_id = 1, uint8_t component_id = 1)
		: send_(std::move(send)), system_id_(system_id), component_id_(component_id) {}

	void operator()(const mavlink_message_t &message) const
	{
		if (send_) {
			send_(message);
		}
	}

	explicit operator bool() const { return static_cast<bool>(send_); }
	uint8_t system_id() const { return system_id_; }
	uint8_t component_id() const { return component_id_; }

private:
	std::function<void(const mavlink_message_t &)> send_;
	uint8_t system_id_;
	uint8_t component_id_;
};
