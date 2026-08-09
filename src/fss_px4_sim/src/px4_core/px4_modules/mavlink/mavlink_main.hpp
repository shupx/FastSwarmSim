/**
 * @file mavlink_main.hpp
 * @brief UDP-based MAVLink module entry point for the PX4 simulation.
 *
 * The module owns the simulated MAVLink receiver and streamer. It sends
 * outgoing messages through a connected loopback UDP socket and processes
 * incoming messages on a dedicated thread.
 *
 * @author Peixuan Shu
 * @date 2026-08-08
 * Modified by Peixuan Shu (2026-08-09): bind MAVLink work to the owning PX4
 * instance context and remove agent-indexed state routing.
 */

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <thread>

#include <mavlink/v2.0/common/mavlink.h>
#include <fss_px4_sim/px4_instance_context.hpp>

#include "mavlink_receiver.h"
#include "mavlink_streamer.hpp"

/**
 * @brief Simulated PX4 MAVLink transport module.
 *
 * MAVLINK coordinates the UDP transport, the background receiver, and the
 * scheduled message streamer for one simulated vehicle.
 */
class MAVLINK
{
public:
	/**
	 * @brief Create a simulated MAVLink module.
	 *
	 * Socket creation and thread startup are deferred until start() is called.
	 * Both UDP endpoints use the loopback interface.
	 *
	 * @param context runtime context owned by the PX4SITL instance.
	 * @param local_port local UDP port on which MAVLink messages are received.
	 * @param remote_port remote UDP port to which MAVLink messages are sent.
	 */
	// Modified by Peixuan Shu: receive the complete context used by this module
	// and its receiver thread.
	MAVLINK(MavrosQuadSimulator::Px4InstanceContext &context,
		int local_port, int remote_port);

	/**
	 * @brief Stop the module and release its socket and worker thread.
	 */
	~MAVLINK();

	/** @brief Disable copying because the module owns a socket and thread. */
	MAVLINK(const MAVLINK &) = delete;

	/** @brief Disable copy assignment because the module owns a socket and thread. */
	MAVLINK &operator=(const MAVLINK &) = delete;

	/**
	 * @brief Open the UDP socket and start receiving MAVLink messages.
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

private:
	/**
	 * @brief Receive and parse MAVLink bytes from the UDP socket.
	 *
	 * This function runs on receive_thread_ until receiving_ is cleared or the
	 * socket encounters an unrecoverable error.
	 */
	void receive_loop();

	/**
	 * @brief Serialize and send one MAVLink message through the UDP socket.
	 *
	 * The message is discarded when the module is stopped. Sending is
	 * non-blocking, so a transient send failure does not stop the receiver.
	 *
	 * @param message MAVLink message to serialize and send.
	 */
	void send_message(const mavlink_message_t &message) const;

	// Modified by Peixuan Shu: one context isolates all MAVLink-facing PX4 state.
	MavrosQuadSimulator::Px4InstanceContext &context_;
	int local_port_; ///< Local UDP port used to receive MAVLink messages.
	int remote_port_; ///< Remote UDP port used to send MAVLink messages.
	int socket_fd_{-1}; ///< UDP socket descriptor, or -1 when stopped.
	std::atomic<bool> receiving_{false}; ///< Whether the receive loop should continue.
	std::thread receive_thread_; ///< Worker thread that reads and parses incoming data.
	std::unique_ptr<MavlinkReceiver> receiver_; ///< Handler for parsed incoming messages.
	std::unique_ptr<MavlinkStreamer> streamer_; ///< Producer of scheduled outgoing messages.
};
