/**
 * @file mavlink_main.hpp
 * @brief UDP or direct ROS MAVLink entry point for the PX4 simulation.
 *
 * The module owns the simulated MAVLink receiver and streamer. It sends
 * outgoing messages either through loopback UDP or a direct callback. PX4SITL
 * does not construct this class for the empty transport.
 *
 * @author Peixuan Shu
 * @date 2026-08-08
 * Modified by Peixuan Shu (2026-08-09): bind MAVLink work to the owning PX4
 * instance context and remove agent-indexed state routing.
 */

#pragma once

#include <cstdint>
#include <functional>
#include <memory>

#include <mavlink/v2.0/common/mavlink.h>
#include <fss_px4_sim/px4_instance_context.hpp>
#include "mavlink_receiver.h"
#include "mavlink_streamer.hpp"
#include "mavlink_udp_transport.hpp"

/**
 * @brief Simulated PX4 MAVLink transport module.
 *
 * MAVLINK coordinates the selected transport, receiver, and scheduled message
 * streamer for one simulated vehicle.
 */
class MAVLINK
{
public:
	using SendCallback = std::function<void(const mavlink_message_t &)>;
	enum class Transport { Udp, DirectRos };

	/**
	 * @brief Create a simulated MAVLink module.
	 *
	 * Transport construction is deferred until start() is called. The UDP
	 * implementation owns its socket and worker thread separately.
	 *
	 * @param context runtime context owned by the PX4SITL instance.
	 * @param local_port local UDP port on which MAVLink messages are received.
	 * @param remote_port remote UDP port to which MAVLink messages are sent.
	 */
	// Modified by Peixuan Shu: receive the complete context used by this module
	// and its receiver thread.
	MAVLINK(MavrosQuadSimulator::Px4InstanceContext &context,
		int local_port, int remote_port, uint8_t system_id = 1, uint8_t component_id = 1,
		Transport transport = Transport::Udp);

	/**
	 * @brief Stop the module and release its selected transport.
	 */
	~MAVLINK();

	/** @brief Disable copying because the module owns transport state. */
	MAVLINK(const MAVLINK &) = delete;

	/** @brief Disable copy assignment because the module owns transport state. */
	MAVLINK &operator=(const MAVLINK &) = delete;

	/**
	 * @brief Construct and start the selected MAVLink transport.
	 *
	 * Calling start() while the module is already running has no effect.
	 * Socket setup failures are reported by throwing std::runtime_error.
	 */
	void start();

	/**
	 * @brief Stop receiving messages and release all runtime resources.
	 *
	 * Calling stop() when the module is already stopped has no effect.
	 */
	void stop();

	/**
	 * @brief Stream scheduled MAVLink messages.
	 *
	 * The timestamp is forwarded to MavlinkStreamer, which decides which
	 * messages are due according to their configured rates.
	 *
	 * @param time_us current simulation time in microseconds.
	 */
	void Stream(const uint64_t &time_us);

	/** Deliver one decoded MAVLink message without UDP serialization. */
	void receive_message(const mavlink_message_t &message);

	/** Set the destination used by Direct transport for outgoing messages. */
	void set_send_callback(SendCallback callback);

	Transport transport() const { return transport_; }
	uint8_t system_id() const { return system_id_; }
	uint8_t component_id() const { return component_id_; }

private:
	/** Dispatch one outgoing message to the selected transport. */
	void send_message(const mavlink_message_t &message) const;

	// Modified by Peixuan Shu: one context isolates all MAVLink-facing PX4 state.
	MavrosQuadSimulator::Px4InstanceContext &context_;
	Px4MavlinkUdpTransport::Config udp_config_;
	uint8_t system_id_; // added or modified by Peixuan Shu: source and target system ID for this PX4 instance.
	uint8_t component_id_; // added or modified by Peixuan Shu: source and target component ID for this PX4 instance.
	Transport transport_;
	SendCallback send_callback_;
	std::unique_ptr<Px4MavlinkUdpTransport> udp_transport_;
	std::unique_ptr<MavlinkReceiver> receiver_; ///< Handler for parsed incoming messages.
	std::unique_ptr<MavlinkStreamer> streamer_; ///< Producer of scheduled outgoing messages.
};
