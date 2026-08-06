/**
 * @file mavlink_msg_list.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Store the mavlink messages to simulate the send and receive of mavlink messages. 
 * 
 * Note: This program relies on 
 * 
 * @version 1.0
 * @date 2023-12-17
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */

#ifndef __MAVLINK_MSG_LIST_HPP__
#define __MAVLINK_MSG_LIST_HPP__

#include <mavlink/v2.0/common/mavlink.h> // mavlink c headers from ros-noetic-mavlink

#include<vector> // for std::vector
#include<array> // for std::array


struct mavlink_info_s
{
	bool updated;
	mavlink_message_t msg;
};


namespace px4 
{ 

enum class mavlink_stream_handle : uint16_t {
	ATTITUDE_QUATERNION,
	ATTITUDE_TARGET,
	HEARTBEAT,
	LOCAL_POSITION_NED,
	POSITION_TARGET_LOCAL_NED,
	SYS_STATUS,
	GLOBAL_POSITION_INT,
	GPS_GLOBAL_ORIGIN,
	//@TODO COMMAND_ACK for vehicle_command_ack
	ENUM_NUM  // number of mavlink_stream
};
#define MAVLINK_STREAM_NUM (int)px4::mavlink_stream_handle::ENUM_NUM // number of mavlink_receive
/* Store the mavlink stream messages for arbitrary number of UAVs */
extern std::vector<std::array<mavlink_info_s, MAVLINK_STREAM_NUM>> mavlink_stream_lists; // declare global
// // Store the streaming mavlink messages (declaring global)
// extern mavlink_info_s mavlink_stream_list[MAVLINK_STREAM_NUM]; // declare global

enum class mavlink_receive_handle : uint16_t {
	SET_POSITION_TARGET_LOCAL_NED,
	SET_POSITION_TARGET_GLOBAL_INT,
	SET_ATTITUDE_TARGET,
	SET_MODE,
	COMMAND_LONG,
	COMMAND_INT,
	SET_GPS_GLOBAL_ORIGIN,
	ENUM_NUM // number of mavlink_receive
};
#define MAVLINK_RECEIVE_NUM (int)px4::mavlink_receive_handle::ENUM_NUM // number of mavlink_receive
/* Store the mavlink stream messages for arbitrary number of UAVs */
extern std::vector<std::array<mavlink_info_s, MAVLINK_RECEIVE_NUM>> mavlink_receive_lists; // declare global
// // Store the receiving mavlink messages
// extern mavlink_info_s mavlink_receive_list[MAVLINK_RECEIVE_NUM]; // declare global

void allocate_mavlink_message_storage(int expected_agent_num);

}

#endif