/**
 * Copied from PX4-Autopilot/src/modules/mavlink/streams
 * Comment out useless defines.
 * @author Peixuan Shu
 */

/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef SYS_STATUS_HPP
#define SYS_STATUS_HPP

#include <uORB/topics/battery_status.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/vehicle_status.h>

#include <uORB/uORB_sim.hpp> // Added by Peixuan Shu
#include "../mavlink_msg_list.hpp"  // Added by Peixuan Shu

class MavlinkStreamSysStatus
{
private:

    int agent_id_ = -1; // agent id. 
	
	uORB_sim::Subscription<vehicle_status_s> _status_sub{ORB_ID(vehicle_status)};
	uORB_sim::Subscription<cpuload_s> _cpuload_sub{ORB_ID(cpuload)};
	uORB_sim::Subscription<battery_status_s> _battery_status_subs{ORB_ID(battery_status)};

public:
	void set_agent_id(int id)
	{
		agent_id_ = id;
	}

	bool send()
	{
		if (_status_sub.updated() || _cpuload_sub.updated() || _battery_status_subs.updated()) {
			vehicle_status_s status{};
			_status_sub.copy(&status);

			cpuload_s cpuload{};
			_cpuload_sub.copy(&cpuload);

			battery_status_s battery_status[battery_status_s::MAX_INSTANCES] {};

			// for (int i = 0; i < _battery_status_subs.size(); i++) {
			// 	_battery_status_subs[i].copy(&battery_status[i]);
			// }
			_battery_status_subs.copy(&battery_status[0]); // simply use the first battery. Added by Peixuan Shu

			int lowest_battery_index = 0;

			// // No battery is connected, select the first group
			// // Low battery judgment is performed only when the current battery is connected
			// // When the last cached battery is not connected or the current battery level is lower than the cached battery level,
			// // the current battery status is replaced with the cached value
			// for (int i = 0; i < _battery_status_subs.size(); i++) {
			// 	if (battery_status[i].connected && ((!battery_status[lowest_battery_index].connected)
			// 					    || (battery_status[i].remaining < battery_status[lowest_battery_index].remaining))) {
			// 		lowest_battery_index = i;
			// 	}
			// }

			mavlink_sys_status_t msg{};

			msg.onboard_control_sensors_present = static_cast<uint32_t>(status.onboard_control_sensors_present & 0xFFFFFFFF) |
							      MAV_SYS_STATUS_EXTENSION_USED;
			msg.onboard_control_sensors_enabled = static_cast<uint32_t>(status.onboard_control_sensors_enabled & 0xFFFFFFFF) |
							      MAV_SYS_STATUS_EXTENSION_USED;
			msg.onboard_control_sensors_health = static_cast<uint32_t>(status.onboard_control_sensors_health & 0xFFFFFFFF) |
							     MAV_SYS_STATUS_EXTENSION_USED;

			msg.onboard_control_sensors_present_extended = static_cast<uint32_t>((status.onboard_control_sensors_present >> 32u) &
					0xFFFFFFFF);
			msg.onboard_control_sensors_enabled_extended = static_cast<uint32_t>((status.onboard_control_sensors_enabled >> 32u) &
					0xFFFFFFFF);
			msg.onboard_control_sensors_health_extended = static_cast<uint32_t>((status.onboard_control_sensors_health >> 32u) &
					0xFFFFFFFF);

			msg.load = cpuload.load * 1000.0f;

			// TODO: Determine what data should be put here when there are multiple batteries.
			//  Right now, it uses the lowest battery. This is a safety decision, because if a client is only checking
			//  one battery using this message, it should be the lowest.
			//  In the future, this should somehow determine the "main" battery, or use the "type" field of BATTERY_STATUS
			//  to determine which battery is more important at a given time.
			const battery_status_s &lowest_battery = battery_status[lowest_battery_index];

			if (lowest_battery.connected) {
				msg.voltage_battery = lowest_battery.voltage_filtered_v * 1000.0f;
				msg.current_battery = lowest_battery.current_filtered_a * 100.0f;
				msg.battery_remaining = ceilf(lowest_battery.remaining * 100.0f);

			} else {
				msg.voltage_battery = UINT16_MAX;
				msg.current_battery = -1;
				msg.battery_remaining = -1;
			}

			// mavlink_msg_sys_status_send_struct(_mavlink->get_channel(), &msg);

			/*  Added by Peixuan Shu. Write mavlink messages into "px4_modules/mavlink/mavlink_msg_list.hpp" */
			int handle = (int) px4::mavlink_stream_handle::SYS_STATUS;
			mavlink_msg_sys_status_encode(1, 1, &px4::mavlink_stream_lists.at(agent_id_)[handle].msg, &msg); 
			px4::mavlink_stream_lists.at(agent_id_)[handle].updated = true;

			return true;
		}

		return false;
	}
};

#endif // SYS_STATUS_HPP
