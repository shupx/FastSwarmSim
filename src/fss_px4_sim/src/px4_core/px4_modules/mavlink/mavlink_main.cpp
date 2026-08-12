/**
 * @file mavlink_main.cpp
 * @brief UDP or direct ROS MAVLink entry point for the PX4 simulation.
 *
 * The module owns the simulated MAVLink receiver and streamer. It sends
 * outgoing messages through a selected transport. Socket and byte-stream
 * details are isolated in Px4MavlinkUdpTransport.
 *
 * @author Peixuan Shu
 * @date 2026-08-08
 * Modified by Peixuan Shu (2026-08-09): scope the MAVLink module and receiver
 * thread to their owning PX4 instance context.
 */

#include "mavlink_main.hpp"

MAVLINK::MAVLINK(MavrosQuadSimulator::Px4InstanceContext &context,
	int local_port, int remote_port, uint8_t system_id, uint8_t component_id, Transport transport)
	: context_(context),
	  udp_config_{static_cast<uint16_t>(local_port), static_cast<uint16_t>(remote_port)},
	  system_id_(system_id), component_id_(component_id), transport_(transport)
{
}

MAVLINK::~MAVLINK()
{
	stop();
}

void MAVLINK::start()
{
	//construct MAVLink components in their owning context.
	MavrosQuadSimulator::Px4InstanceContext::Scope context_scope(context_);
	if (receiver_ || streamer_) {
		return;
	}
	receiver_ = std::make_unique<MavlinkReceiver>(system_id_, component_id_);
	streamer_ = std::make_unique<MavlinkStreamer>(MavlinkSender(
		[this](const mavlink_message_t &message) { send_message(message); },
		system_id_, component_id_));
	if (transport_ == Transport::DirectRos) {
		return;
	}

	try {
		udp_transport_ = std::make_unique<Px4MavlinkUdpTransport>(udp_config_);
		udp_transport_->set_receive_callback([this](const mavlink_message_t &message) {
			receive_message(message);
		});
		udp_transport_->start();
	} catch (...) {
		udp_transport_.reset();
		streamer_.reset();
		receiver_.reset();
		throw;
	}
}

void MAVLINK::stop()
{
	udp_transport_.reset();
	streamer_.reset();
	receiver_.reset();
}

void MAVLINK::Stream(const uint64_t &time_us)
{
	// keep direct MAVLINK callers in the owning context.
	MavrosQuadSimulator::Px4InstanceContext::Scope context_scope(context_);
	if (streamer_) {
		streamer_->Stream(time_us);
	}
}

void MAVLINK::send_message(const mavlink_message_t &message) const
{
	if (transport_ == Transport::DirectRos) {
		if (send_callback_) {
			send_callback_(message);
		}
		return;
	}
	if (udp_transport_) {
		udp_transport_->send_message(message);
	}
}

void MAVLINK::receive_message(const mavlink_message_t &message)
{
	MavrosQuadSimulator::Px4InstanceContext::Scope context_scope(context_);
	if (receiver_) {
		auto mutable_message = message;
		receiver_->handle_message(&mutable_message);
	}
}

void MAVLINK::set_send_callback(SendCallback callback)
{
	send_callback_ = std::move(callback);
}
