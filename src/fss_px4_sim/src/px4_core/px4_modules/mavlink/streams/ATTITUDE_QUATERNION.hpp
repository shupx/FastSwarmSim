/**
 * Copied from PX4-Autopilot/src/modules/mavlink/streams
 * Comment out useless defines.
 * @author Peixuan Shu
 */

/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#ifndef ATTITUDE_QUATERNION_HPP
#define ATTITUDE_QUATERNION_HPP

#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_status.h>

#include <uORB/uORB_sim.hpp> // Added by Peixuan Shu
#include "../mavlink_msg_list.hpp"  // Added by Peixuan Shu

class MavlinkStreamAttitudeQuaternion
{
private:

    int agent_id_ = -1; // agent id.
	
	uORB_sim::Subscription<vehicle_attitude_s> _att_sub{ORB_ID(vehicle_attitude)};
	uORB_sim::Subscription<vehicle_angular_velocity_s> _angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB_sim::Subscription<vehicle_status_s> _status_sub{ORB_ID(vehicle_status)};

public:
	void set_agent_id(int id)
	{
		agent_id_ = id;
	}

	bool send()
	{
		vehicle_attitude_s att;

		if (_att_sub.update(&att)) {
			vehicle_angular_velocity_s angular_velocity{};
			_angular_velocity_sub.copy(&angular_velocity);

			vehicle_status_s status{};
			_status_sub.copy(&status);

			mavlink_attitude_quaternion_t msg{};

			msg.time_boot_ms = att.timestamp / 1000;
			msg.q1 = att.q[0];
			msg.q2 = att.q[1];
			msg.q3 = att.q[2];
			msg.q4 = att.q[3];
			msg.rollspeed = angular_velocity.xyz[0];
			msg.pitchspeed = angular_velocity.xyz[1];
			msg.yawspeed = angular_velocity.xyz[2];

			if (status.is_vtol && status.is_vtol_tailsitter && (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING)) {
				// // This is a tailsitter VTOL flying in fixed wing mode:
				// // indicate that reported attitude should be rotated by
				// // 90 degrees upward pitch for user display
				// get_rot_quaternion(ROTATION_PITCH_90).copyTo(msg.repr_offset_q);

				std::cout << "Unsupported vehicle_type" << std::endl; //Added by Peixuan Shu
			} else {
				// Normal case
				// zero rotation should be [1 0 0 0]:
				// `get_rot_quaternion(ROTATION_NONE).copyTo(msg.repr_offset_q);`
				// but to save bandwidth, we instead send [0, 0, 0, 0].
				msg.repr_offset_q[0] = 0.0f;
				msg.repr_offset_q[1] = 0.0f;
				msg.repr_offset_q[2] = 0.0f;
				msg.repr_offset_q[3] = 0.0f;
			}

			// mavlink_msg_attitude_quaternion_send_struct(_mavlink->get_channel(), &msg);

			/*  Added by Peixuan Shu. Write mavlink messages into "px4_modules/mavlink/mavlink_msg_list.hpp" */
			int handle = (int) px4::mavlink_stream_handle::ATTITUDE_QUATERNION;
			mavlink_msg_attitude_quaternion_encode(1, 1, &px4::mavlink_stream_lists.at(agent_id_)[handle].msg, &msg); 
			px4::mavlink_stream_lists.at(agent_id_)[handle].updated = true;

			return true;
		}

		return false;
	}
};

#endif // ATTITUDE_QUATERNION_HPP
