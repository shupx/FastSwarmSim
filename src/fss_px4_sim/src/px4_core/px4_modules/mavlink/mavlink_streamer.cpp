/**
 * @file mavlink_streamer.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Simulate the streaming of mavlink messages in PX4. 
 * 
 * Note: This program relies on mavlink
 * 
 * @version 1.0
 * @date 2023-12-17
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */

#include "mavlink_streamer.hpp"

// Modified by Peixuan Shu: agent_id no longer selects global PX4 state.
MavlinkStreamer::MavlinkStreamer(MavlinkSender sender) : sender_(std::move(sender))
{
    mavlink_stream_AttitudeQuaternion_ = STREAM_MAKE_PTR(MavlinkStreamAttitudeQuaternion)(50, this); // set streaming rate
    mavlink_stream_LocalPositionNED_ = STREAM_MAKE_PTR(MavlinkStreamLocalPositionNED)(30, this); // set streaming rate
    mavlink_stream_AttitudeTarget_ = STREAM_MAKE_PTR(MavlinkStreamAttitudeTarget)(50, this); // set streaming rate
    mavlink_stream_PositionTargetLocalNed_ = STREAM_MAKE_PTR(MavlinkStreamPositionTargetLocalNed)(30, this); // set streaming rate
    mavlink_stream_Heartbeat_ = STREAM_MAKE_PTR(MavlinkStreamHeartbeat)(10, this); // set streaming rate
    mavlink_stream_SysStatus_ = STREAM_MAKE_PTR(MavlinkStreamSysStatus)(2, this); // set streaming rate
    mavlink_stream_GlobalPositionInt_ = STREAM_MAKE_PTR(MavlinkStreamGlobalPositionInt)(30, this); // set streaming rate
    mavlink_stream_GpsGlobalOrigin_ = STREAM_MAKE_PTR(MavlinkStreamGpsGlobalOrigin)(100, this); // set streaming rate (Actually it will only send the mavlink stream when the gps origin is updated. Refer to streams/GPS_GLOBAL_ORIGIN.hpp)

    //@TODO extended_sys_state for extended_state

}


MavlinkStreamer::~MavlinkStreamer()
{
    stream_signal_.disconnect_all_slots();
}

void MavlinkStreamer::Stream(const uint64_t &time_us)
{
	while (_vehicle_command_ack_sub.updated()) {
		vehicle_command_ack_s command_ack{};
		if (!_vehicle_command_ack_sub.update(&command_ack)) {
			break;
		}

		if (!command_ack.from_external
		    && command_ack.command < vehicle_command_s::VEHICLE_CMD_PX4_INTERNAL_START) {
			mavlink_command_ack_t ack{};
			ack.command = command_ack.command;
			ack.result = command_ack.result;
			ack.progress = command_ack.result_param1;
			ack.result_param2 = command_ack.result_param2;
			ack.target_system = command_ack.target_system;
			ack.target_component = command_ack.target_component;

			mavlink_message_t encoded{};
			mavlink_msg_command_ack_encode(sender_.system_id(), sender_.component_id(), &encoded, &ack); // added or modified by Peixuan Shu
			if (sender_) {
				sender_(encoded);
			}
		}
	}

    // stream all mavlink messages
    stream_signal_(time_us); 
}
