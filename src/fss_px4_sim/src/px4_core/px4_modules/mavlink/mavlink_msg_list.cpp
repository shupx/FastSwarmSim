/**
 * @file mavlink_msg_list.cpp
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


#include "mavlink_msg_list.hpp"


namespace px4 
{ 

// Init mavlink lists here (they are global/extern!)

// initialize with at least one element for UAV 1
std::vector<std::array<mavlink_info_s, MAVLINK_STREAM_NUM>> mavlink_stream_lists{std::array<mavlink_info_s, MAVLINK_STREAM_NUM>{}}; // define (allocate storage)

// initialize with at least one element for UAV 1
std::vector<std::array<mavlink_info_s, MAVLINK_RECEIVE_NUM>> mavlink_receive_lists {std::array<mavlink_info_s, MAVLINK_RECEIVE_NUM>{}}; // define (allocate storage)


/* allocate global storage for messages of agent i */
void allocate_mavlink_message_storage(int expected_agent_num)
{
    while(mavlink_stream_lists.size() < expected_agent_num)
    {
        mavlink_stream_lists.emplace_back(std::array<mavlink_info_s, MAVLINK_STREAM_NUM>{});
    }
    while(mavlink_receive_lists.size() < expected_agent_num)
    {
        mavlink_receive_lists.emplace_back(std::array<mavlink_info_s, MAVLINK_RECEIVE_NUM>{});
    }
}

}