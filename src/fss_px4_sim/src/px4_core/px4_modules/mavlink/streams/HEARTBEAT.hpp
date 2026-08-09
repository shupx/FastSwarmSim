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

#ifndef HEARTBEAT_HPP
#define HEARTBEAT_HPP

#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status_flags.h>

#include <uORB/uORB_sim.hpp> // Added by Peixuan Shu
#include "../mavlink_sender.hpp"

#include <commander/px4_custom_mode.h> // Added by Peixuan Shu

/* Added by Peixuan Shu. Originally in https://github.com/PX4/PX4-Autopilot/blob/v1.13.3/src/modules/mavlink/mavlink_messages.cpp */
#ifndef MAVLINK_MESSAGES_H_
static inline union px4_custom_mode get_px4_custom_mode(uint8_t nav_state)
{
	union px4_custom_mode custom_mode;
	custom_mode.data = 0;

	switch (nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_MANUAL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_ALTCTL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_POSCTL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_MISSION;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_RTL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL:
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LAND;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ACRO:
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_ACRO;
		break;

	case vehicle_status_s::NAVIGATION_STATE_DESCEND:
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LAND;
		break;

	case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_MANUAL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_OFFBOARD;
		break;

	case vehicle_status_s::NAVIGATION_STATE_STAB:
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_STABILIZED;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LAND;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ORBIT:
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_POSCTL;
		custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_POSCTL_ORBIT;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF:
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_VTOL_TAKEOFF;
		break;
	}

	return custom_mode;
}
#endif //MAVLINK_MESSAGES_H_


class MavlinkStreamHeartbeat
{
private:

	// Modified by Peixuan Shu: subscriptions bind to the active PX4 context.
	MavlinkSender sender_;
	
	uORB_sim::Subscription<actuator_armed_s> _acturator_armed_sub{ORB_ID(actuator_armed)};
	uORB_sim::Subscription<vehicle_control_mode_s> _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB_sim::Subscription<vehicle_status_s> _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB_sim::Subscription<vehicle_status_flags_s> _vehicle_status_flags_sub{ORB_ID(vehicle_status_flags)};

public:
	void set_sender(MavlinkSender sender) { sender_ = std::move(sender); }

	bool send()
	{
		if (/*_mavlink->get_free_tx_buf() >= get_size()*/ true) {
			// always send the heartbeat, independent of the update status of the topics
			vehicle_status_s vehicle_status{};
			_vehicle_status_sub.copy(&vehicle_status);

			vehicle_status_flags_s vehicle_status_flags{};
			_vehicle_status_flags_sub.copy(&vehicle_status_flags);

			vehicle_control_mode_s vehicle_control_mode{};
			_vehicle_control_mode_sub.copy(&vehicle_control_mode);

			actuator_armed_s actuator_armed{};
			_acturator_armed_sub.copy(&actuator_armed);

			// uint8_t base_mode (MAV_MODE_FLAG) - System mode bitmap.
			uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

			if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
				base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
			}

			if (vehicle_status.hil_state == vehicle_status_s::HIL_STATE_ON) {
				base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
			}

			if (vehicle_control_mode.flag_control_manual_enabled) {
				base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
			}

			if (vehicle_control_mode.flag_control_attitude_enabled) {
				base_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
			}

			if (vehicle_control_mode.flag_control_auto_enabled) {
				base_mode |= MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED;
			}


			// uint32_t custom_mode - A bitfield for use for autopilot-specific flags
			union px4_custom_mode custom_mode {get_px4_custom_mode(vehicle_status.nav_state)};


			// uint8_t system_status (MAV_STATE) - System status flag.
			uint8_t system_status = MAV_STATE_UNINIT;

			switch (vehicle_status.arming_state) {
			case vehicle_status_s::ARMING_STATE_ARMED:
				system_status = vehicle_status.failsafe ? MAV_STATE_CRITICAL : MAV_STATE_ACTIVE;
				break;

			case vehicle_status_s::ARMING_STATE_STANDBY:
				system_status = MAV_STATE_STANDBY;
				break;

			case vehicle_status_s::ARMING_STATE_SHUTDOWN:
				system_status = MAV_STATE_POWEROFF;
				break;
			}

			// system_status overrides
			if (actuator_armed.force_failsafe || actuator_armed.lockdown || actuator_armed.manual_lockdown
			    || vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_TERMINATION) {
				system_status = MAV_STATE_FLIGHT_TERMINATION;

			} else if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL) {
				system_status = MAV_STATE_EMERGENCY;

			} else if (vehicle_status_flags.calibration_enabled) {
				system_status = MAV_STATE_CALIBRATING;
			}


			// mavlink_msg_heartbeat_send(_mavlink->get_channel(), _mavlink->get_system_type(), MAV_AUTOPILOT_PX4,
			// 			   base_mode, custom_mode.data, system_status);

			/* Encode and send the stream directly over the PX4 UDP endpoint. */
			mavlink_heartbeat_t msg{};
			msg.type = 0; // equal to _mavlink->get_system_type(), which is _param_mav_type.get()
			msg.autopilot = MAV_AUTOPILOT_PX4;
			msg.base_mode = base_mode;
			msg.custom_mode = custom_mode.data;
			msg.system_status = system_status;
			mavlink_message_t encoded{};
			mavlink_msg_heartbeat_encode(sender_.system_id(), sender_.component_id(), &encoded, &msg); // added or modified by Peixuan Shu
			if (sender_) sender_(encoded);

			return true;
		}

		return false;
	}
};


#endif // HEARTBEAT_HPP
