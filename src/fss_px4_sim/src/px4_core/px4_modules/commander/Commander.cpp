/**
 * Modified from https://github.com/PX4/PX4-Autopilot/blob/v1.13.3/src/modules/commander
 * Modified by Peixuan Shu
 * 
 * Simulate the command handle and mode switching process
 * 
 * 2023-12-23
 * 
 * @author Peixuan Shu
 */

/****************************************************************************
 *
 *   Copyright (c) 2013-2022 PX4 Development Team. All rights reserved.
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

/**
 * @file commander.cpp
 *
 * Main state machine / business logic
 *
 * @TODO This application is currently in a rewrite process. Main changes:
 *			- Calibration routines are moved into the event system
 *			- Commander is rewritten as class
 *			- State machines will be model driven
 */

#include "Commander.hpp"

/* commander module headers */
// #include "Arming/PreFlightCheck/PreFlightCheck.hpp"
// #include "Arming/ArmAuthorization/ArmAuthorization.h"
// #include "Arming/HealthFlags/HealthFlags.h"
// #include "commander_helper.h"
// #include "esc_calibration.h"
#include "px4_custom_mode.h"
// #include "state_machine_helper.h"

/* PX4 headers */
// #include <dataman/dataman.h>
#include <drivers/drv_hrt.h>
// #include <drivers/drv_tone_alarm.h>
#include <geo/geo.h>
#include <mathlib/mathlib.h>
// #include <navigator/navigation.h>
// #include <px4_platform_common/events.h>
// #include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
// #include <px4_platform_common/external_reset_lockout.h>
// #include <px4_platform_common/posix.h>
// #include <px4_platform_common/shutdown.h>
// #include <px4_platform_common/tasks.h>
// #include <px4_platform_common/time.h>
// #include <circuit_breaker/circuit_breaker.h>
// #include <systemlib/mavlink_log.h>

#include <math.h>
#include <float.h>
#include <cstring>
#include <matrix/matrix/math.hpp>

// #include <uORB/topics/mavlink_log.h>
// #include <uORB/topics/tune_control.h>

typedef enum VEHICLE_MODE_FLAG {
	VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED = 1, /* 0b00000001 Reserved for future use. | */
	VEHICLE_MODE_FLAG_TEST_ENABLED = 2, /* 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations. | */
	VEHICLE_MODE_FLAG_AUTO_ENABLED = 4, /* 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation. | */
	VEHICLE_MODE_FLAG_GUIDED_ENABLED = 8, /* 0b00001000 guided mode enabled, system flies MISSIONs / mission items. | */
	VEHICLE_MODE_FLAG_STABILIZE_ENABLED = 16, /* 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around. | */
	VEHICLE_MODE_FLAG_HIL_ENABLED = 32, /* 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational. | */
	VEHICLE_MODE_FLAG_MANUAL_INPUT_ENABLED = 64, /* 0b01000000 remote control input is enabled. | */
	VEHICLE_MODE_FLAG_SAFETY_ARMED = 128, /* 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. Additional note: this flag is to be ignore when sent in the command MAV_CMD_DO_SET_MODE and MAV_CMD_COMPONENT_ARM_DISARM shall be used instead. The flag can still be used to report the armed state. | */
	VEHICLE_MODE_FLAG_ENUM_END = 129, /*  | */
} VEHICLE_MODE_FLAG;


// static bool send_vehicle_command(const uint32_t cmd, const float param1 = NAN, const float param2 = NAN,
// 				 const float param3 = NAN,  const float param4 = NAN, const double param5 = static_cast<double>(NAN),
// 				 const double param6 = static_cast<double>(NAN), const float param7 = NAN)
// {
	// vehicle_command_s vcmd{};
	// vcmd.command = cmd;
	// vcmd.param1 = param1;
	// vcmd.param2 = param2;
	// vcmd.param3 = param3;
	// vcmd.param4 = param4;
	// vcmd.param5 = param5;
	// vcmd.param6 = param6;
	// vcmd.param7 = param7;

	// uORB_sim::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	// vcmd.source_system = vehicle_status_sub.get().system_id;
	// vcmd.target_system = vehicle_status_sub.get().system_id;
	// vcmd.source_component = vehicle_status_sub.get().component_id;
	// vcmd.target_component = vehicle_status_sub.get().component_id;

	// uORB_sim::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
	// vcmd.timestamp = hrt_absolute_time();
	// return vcmd_pub.publish(vcmd);
// }

// static bool broadcast_vehicle_command(const uint32_t cmd, const float param1 = NAN, const float param2 = NAN,
// 				      const float param3 = NAN,  const float param4 = NAN, const double param5 = static_cast<double>(NAN),
// 				      const double param6 = static_cast<double>(NAN), const float param7 = NAN)
// {
	// vehicle_command_s vcmd{};
	// vcmd.command = cmd;
	// vcmd.param1 = param1;
	// vcmd.param2 = param2;
	// vcmd.param3 = param3;
	// vcmd.param4 = param4;
	// vcmd.param5 = param5;
	// vcmd.param6 = param6;
	// vcmd.param7 = param7;

	// uORB_sim::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	// vcmd.source_system = vehicle_status_sub.get().system_id;
	// vcmd.target_system = 0;
	// vcmd.source_component = vehicle_status_sub.get().component_id;
	// vcmd.target_component = 0;

	// uORB_sim::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
	// vcmd.timestamp = hrt_absolute_time();
	// return vcmd_pub.publish(vcmd);
// }

/* Delete some useless functions */ 


Commander::Commander(int agent_id) : 
	agent_id_(agent_id), 
	ModuleParams(nullptr)
	// _failure_detector(this)
{
	_vehicle_land_detected.landed = true;

	// XXX for now just set sensors as initialized
	_status_flags.system_sensors_initialized = true;

	// We want to accept RC inputs as default
	_status.nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;
	_status.nav_state_timestamp = hrt_absolute_time();
	_status.arming_state = vehicle_status_s::ARMING_STATE_INIT;

	/* mark all signals lost as long as they haven't been found */
	_status.rc_signal_lost = true;
	_status.data_link_lost = true;

	_status_flags.offboard_control_signal_lost = true;

	_status_flags.power_input_valid = true;
	_status_flags.rc_calibration_valid = true;

	// default for vtol is rotary wing
	// _vtol_status.vtol_in_rw_mode = true;

	_vehicle_gps_position_valid.set_hysteresis_time_from(false, GPS_VALID_TIME);
	_vehicle_gps_position_valid.set_hysteresis_time_from(true, GPS_VALID_TIME);
}

Commander::~Commander()
{
	// perf_free(_loop_perf);
	// perf_free(_preflight_check_perf);
}

bool
Commander::handle_command(const vehicle_command_s &cmd)
{
	/* only handle commands that are meant to be handled by this system and component, or broadcast */
	if (((cmd.target_system != _status.system_id) && (cmd.target_system != 0))
	    || ((cmd.target_component != _status.component_id) && (cmd.target_component != 0))) {
		return false;
	}

	/* result of the command */
	unsigned cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_UNSUPPORTED;

	/* request to set different system mode */
	switch (cmd.command) {

	case vehicle_command_s::VEHICLE_CMD_DO_SET_MODE: {
			uint8_t base_mode = (uint8_t)cmd.param1;
			uint8_t custom_main_mode = (uint8_t)cmd.param2;
			uint8_t custom_sub_mode = (uint8_t)cmd.param3;

			uint8_t desired_main_state = commander_state_s::MAIN_STATE_MAX;
			transition_result_t main_ret = TRANSITION_NOT_CHANGED;

			if (base_mode & VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED) {
				/* use autopilot-specific mode */
				if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_MANUAL) {
					desired_main_state = commander_state_s::MAIN_STATE_MANUAL;

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_ALTCTL) {
					desired_main_state = commander_state_s::MAIN_STATE_ALTCTL;

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_POSCTL) {
					desired_main_state = commander_state_s::MAIN_STATE_POSCTL;

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_AUTO) {
					if (custom_sub_mode > 0) {

						switch (custom_sub_mode) {
						case PX4_CUSTOM_SUB_MODE_AUTO_LOITER:
							desired_main_state = commander_state_s::MAIN_STATE_AUTO_LOITER;
							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_MISSION:
							desired_main_state = commander_state_s::MAIN_STATE_AUTO_MISSION;
							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_RTL:
							desired_main_state = commander_state_s::MAIN_STATE_AUTO_RTL;
							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF:
							desired_main_state = commander_state_s::MAIN_STATE_AUTO_TAKEOFF;
							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_LAND:
							desired_main_state = commander_state_s::MAIN_STATE_AUTO_LAND;
							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET:
							desired_main_state = commander_state_s::MAIN_STATE_AUTO_FOLLOW_TARGET;
							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND:
							desired_main_state = commander_state_s::MAIN_STATE_AUTO_PRECLAND;
							break;

						default:
							main_ret = TRANSITION_DENIED;
							// mavlink_log_critical(&_mavlink_log_pub, "Unsupported auto mode\t");
							// events::send(events::ID("commander_unsupported_auto_mode"), events::Log::Error,
							// 	     "Unsupported auto mode");

							std::cerr << "[PX4 Commander::handle_command] Unsupported auto mode" << std::endl; //added by Peixuan Shu

							break;
						}

					} else {
						desired_main_state = commander_state_s::MAIN_STATE_AUTO_MISSION;
					}

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_ACRO) {
					desired_main_state = commander_state_s::MAIN_STATE_ACRO;

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_STABILIZED) {
					desired_main_state = commander_state_s::MAIN_STATE_STAB;

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_OFFBOARD) {
					desired_main_state = commander_state_s::MAIN_STATE_OFFBOARD;

					// std::cerr << "[PX4 Commander::handle_command] desired_main_state = commander_state_s::MAIN_STATE_OFFBOARD;" << std::endl; //added by Peixuan Shu
				}

			} else {
				/* use base mode */
				if (base_mode & VEHICLE_MODE_FLAG_AUTO_ENABLED) {
					desired_main_state = commander_state_s::MAIN_STATE_AUTO_MISSION;

				} else if (base_mode & VEHICLE_MODE_FLAG_MANUAL_INPUT_ENABLED) {
					if (base_mode & VEHICLE_MODE_FLAG_GUIDED_ENABLED) {
						desired_main_state = commander_state_s::MAIN_STATE_POSCTL;

					} else if (base_mode & VEHICLE_MODE_FLAG_STABILIZE_ENABLED) {
						desired_main_state = commander_state_s::MAIN_STATE_STAB;

					} else {
						desired_main_state = commander_state_s::MAIN_STATE_MANUAL;
					}
				}
			}

			if (desired_main_state != commander_state_s::MAIN_STATE_MAX) {
				main_ret = main_state_transition(_status, desired_main_state, _status_flags, _internal_state);
			}

			if (main_ret != TRANSITION_DENIED) {
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;

				std::cerr << "[PX4 Commander::handle_command] Mode transition denied" << std::endl; //added by Peixuan Shu
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM: {

			// std::cerr << "[PX4 Commander::handle_command] receive VEHICLE_CMD_COMPONENT_ARM_DISARM" << std::endl; //added by Peixuan Shu

			// Adhere to MAVLink specs, but base on knowledge that these fundamentally encode ints
			// for logic state parameters
			const int8_t arming_action = static_cast<int8_t>(lroundf(cmd.param1));

			if (arming_action != vehicle_command_s::ARMING_ACTION_ARM
			    && arming_action != vehicle_command_s::ARMING_ACTION_DISARM) {
				// mavlink_log_critical(&_mavlink_log_pub, "Unsupported ARM_DISARM param: %.3f\t", (double)cmd.param1);
				// events::send<float>(events::ID("commander_unsupported_arm_disarm_param"), events::Log::Error,
				// 		    "Unsupported ARM_DISARM param: {1:.3}", cmd.param1);
				
				std::cerr << "[PX4 Commander::handle_command] Unsupported ARM_DISARM param: {1:" << cmd.param1 << "}" << std::endl; //added by Peixuan Shu

			} else {
				// Arm is forced (checks skipped) when param2 is set to a magic number.
				const bool forced = (static_cast<int>(lroundf(cmd.param2)) == 21196);
				const bool cmd_from_io = (static_cast<int>(roundf(cmd.param3)) == 1234);

				if (!forced) {
					// Flick to in-air restore first if this comes from an onboard system and from IO
					if (cmd.source_system == _status.system_id && cmd.source_component == _status.component_id
					    && cmd_from_io && (arming_action == vehicle_command_s::ARMING_ACTION_ARM)) {
						_status.arming_state = vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE;
					}
				}

				transition_result_t arming_res = TRANSITION_DENIED;
				// arm_disarm_reason_t arm_disarm_reason = cmd.from_external ? arm_disarm_reason_t::command_external :
				// 					arm_disarm_reason_t::command_internal;

				if (arming_action == vehicle_command_s::ARMING_ACTION_ARM) {
					// arming_res = arm(arm_disarm_reason, cmd.from_external || !forced);

					/* accept all arm/disarm request. Modified by Peixuan Shu */
					arming_res == TRANSITION_CHANGED;
					_armed.armed = true;
					_armed.ready_to_arm = true;
					_status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
					_status.armed_time = hrt_absolute_time();

				} else if (arming_action == vehicle_command_s::ARMING_ACTION_DISARM) {
					// arming_res = disarm(arm_disarm_reason, forced);

					/* accept all arm/disarm request. Modified by Peixuan Shu */
					arming_res == TRANSITION_CHANGED;
					_armed.armed = false;
					_armed.ready_to_arm = true;
					_status.arming_state = vehicle_status_s::ARMING_STATE_STANDBY;
					_status.armed_time = 0;
				}

				if (arming_res == TRANSITION_DENIED) {
					cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;

				} else {
					cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

					/* update home position on arming if at least 500 ms from commander start spent to avoid setting home on in-air restart */
					if ((arming_action == vehicle_command_s::ARMING_ACTION_ARM) && (arming_res == TRANSITION_CHANGED)
					    && (hrt_absolute_time() > (_boot_timestamp + INAIR_RESTART_HOLDOFF_INTERVAL))
					    && (_param_com_home_en.get() && !_home_pub.get().manual_home)) {
						set_home_position();
					}
				}
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN:
		/* ignore commands that are handled by other parts of the system */
		break;

	/* Delete some useless commands by Peixuan Shu */

	default:
		/* Warn about unsupported commands, this makes sense because only commands
		 * to this component ID (or all) are passed by mavlink. */
		answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_UNSUPPORTED);

		std::cerr << "[PX4 Commander::handle_command] Unsupported Vehicle command: " << cmd.command << std::endl;
		break; // added by Peixuan Shu
	}

	if (cmd_result != vehicle_command_s::VEHICLE_CMD_RESULT_UNSUPPORTED) {
		/* already warned about unsupported commands in "default" case */
		answer_command(cmd, cmd_result);
	}

	return true;
}


bool
Commander::hasMovedFromCurrentHomeLocation()
{
	float home_dist_xy = -1.f;
	float home_dist_z = -1.f;
	float eph = 0.f;
	float epv = 0.f;

	if (_home_pub.get().valid_lpos && _local_position_sub.get().xy_valid && _local_position_sub.get().z_valid) {
		mavlink_wpm_distance_to_point_local(_home_pub.get().x, _home_pub.get().y, _home_pub.get().z,
						    _local_position_sub.get().x, _local_position_sub.get().y, _local_position_sub.get().z,
						    &home_dist_xy, &home_dist_z);

		eph = _local_position_sub.get().eph;
		epv = _local_position_sub.get().epv;

	} else if (_home_pub.get().valid_hpos && _home_pub.get().valid_alt) {
		if (_status_flags.global_position_valid) {
			const vehicle_global_position_s &gpos = _global_position_sub.get();

			get_distance_to_point_global_wgs84(_home_pub.get().lat, _home_pub.get().lon, _home_pub.get().alt,
							   gpos.lat, gpos.lon, gpos.alt,
							   &home_dist_xy, &home_dist_z);

			eph = gpos.eph;
			epv = gpos.epv;

		} else if (_status_flags.gps_position_valid) {
			vehicle_gps_position_s gps;
			_vehicle_gps_position_sub.copy(&gps);
			const double lat = static_cast<double>(gps.lat) * 1e-7;
			const double lon = static_cast<double>(gps.lon) * 1e-7;
			const float alt = static_cast<float>(gps.alt) * 1e-3f;

			get_distance_to_point_global_wgs84(_home_pub.get().lat, _home_pub.get().lon, _home_pub.get().alt,
							   lat, lon, alt,
							   &home_dist_xy, &home_dist_z);

			eph = gps.eph;
			epv = gps.epv;
		}
	}

	return (home_dist_xy > eph * 2.f) || (home_dist_z > epv * 2.f);
}

/**
* @brief This function initializes the home position an altitude of the vehicle. This happens first time we get a good GPS fix and each
*		 time the vehicle is armed with a good GPS fix.
**/
bool
Commander::set_home_position()
{
	bool updated = false;
	home_position_s home{};

	if (_status_flags.local_position_valid) {
		// Set home position in local coordinates
		const vehicle_local_position_s &lpos = _local_position_sub.get();
		_heading_reset_counter = lpos.heading_reset_counter; // TODO: should not be here

		fillLocalHomePos(home, lpos);
		updated = true;
	}

	if (_status_flags.global_position_valid) {
		// Set home using the global position estimate (fused INS/GNSS)
		const vehicle_global_position_s &gpos = _global_position_sub.get();
		fillGlobalHomePos(home, gpos);
		setHomePosValid();
		updated = true;

	} else if (_status_flags.gps_position_valid) {
		// Set home using GNSS position
		vehicle_gps_position_s gps_pos;
		_vehicle_gps_position_sub.copy(&gps_pos);
		const double lat = static_cast<double>(gps_pos.lat) * 1e-7;
		const double lon = static_cast<double>(gps_pos.lon) * 1e-7;
		const float alt = static_cast<float>(gps_pos.alt) * 1e-3f;
		fillGlobalHomePos(home, lat, lon, alt);
		setHomePosValid();
		updated = true;

	} else if (_local_position_sub.get().z_global) {
		// handle special case where we are setting only altitude using local position reference
		// This might be overwritten by altitude from global or GNSS altitude
		home.alt = _local_position_sub.get().ref_alt;
		home.valid_alt = true;

		updated = true;
	}

	if (updated) {
		home.timestamp = hrt_absolute_time();
		home.manual_home = false;
		updated = _home_pub.update(home);
	}

	return updated;
}

void
Commander::set_in_air_home_position()
{
	home_position_s home{};
	home = _home_pub.get();
	const bool global_home_valid = home.valid_hpos && home.valid_alt;
	const bool local_home_valid = home.valid_lpos;

	if (local_home_valid && !global_home_valid) {
		if (_status_flags.local_position_valid && _status_flags.global_position_valid) {
			// Back-compute lon, lat and alt of home position given the local home position
			// and current positions in local and global (GNSS fused) frames
			const vehicle_local_position_s &lpos = _local_position_sub.get();
			const vehicle_global_position_s &gpos = _global_position_sub.get();

			MapProjection ref_pos{gpos.lat, gpos.lon};

			double home_lat;
			double home_lon;
			ref_pos.reproject(home.x - lpos.x, home.y - lpos.y, home_lat, home_lon);

			const float home_alt = gpos.alt + home.z;
			fillGlobalHomePos(home, home_lat, home_lon, home_alt);

			setHomePosValid();
			home.timestamp = hrt_absolute_time();
			_home_pub.update(home);

		} else if (_status_flags.local_position_valid && _status_flags.gps_position_valid) {
			// Back-compute lon, lat and alt of home position given the local home position
			// and current positions in local and global (GNSS raw) frames
			const vehicle_local_position_s &lpos = _local_position_sub.get();
			vehicle_gps_position_s gps;
			_vehicle_gps_position_sub.copy(&gps);

			const double lat = static_cast<double>(gps.lat) * 1e-7;
			const double lon = static_cast<double>(gps.lon) * 1e-7;
			const float alt = static_cast<float>(gps.alt) * 1e-3f;

			MapProjection ref_pos{lat, lon};

			double home_lat;
			double home_lon;
			ref_pos.reproject(home.x - lpos.x, home.y - lpos.y, home_lat, home_lon);

			const float home_alt = alt + home.z;
			fillGlobalHomePos(home, home_lat, home_lon, home_alt);

			setHomePosValid();
			home.timestamp = hrt_absolute_time();
			_home_pub.update(home);
		}

	} else if (!local_home_valid && global_home_valid) {
		const vehicle_local_position_s &lpos = _local_position_sub.get();

		if (_status_flags.local_position_valid && lpos.xy_global && lpos.z_global) {
			// Back-compute x, y and z of home position given the global home position
			// and the global reference of the local frame
			MapProjection ref_pos{lpos.ref_lat, lpos.ref_lon};

			float home_x;
			float home_y;
			ref_pos.project(home.lat, home.lon, home_x, home_y);

			const float home_z = -(home.alt - lpos.ref_alt);
			fillLocalHomePos(home, home_x, home_y, home_z, NAN);

			home.timestamp = hrt_absolute_time();
			_home_pub.update(home);
		}

	} else if (!local_home_valid && !global_home_valid) {
		// Home position is not known in any frame, set home at current position
		set_home_position();

	} else {
		// nothing to do
	}
}

void
Commander::fillLocalHomePos(home_position_s &home, const vehicle_local_position_s &lpos) const
{
	fillLocalHomePos(home, lpos.x, lpos.y, lpos.z, lpos.heading);
}

void
Commander::fillLocalHomePos(home_position_s &home, float x, float y, float z, float heading) const
{
	home.x = x;
	home.y = y;
	home.z = z;
	home.valid_lpos = true;

	home.yaw = heading;
}

void Commander::fillGlobalHomePos(home_position_s &home, const vehicle_global_position_s &gpos) const
{
	fillGlobalHomePos(home, gpos.lat, gpos.lon, gpos.alt);
}

void Commander::fillGlobalHomePos(home_position_s &home, double lat, double lon, float alt) const
{
	home.lat = lat;
	home.lon = lon;
	home.valid_hpos = true;
	home.alt = alt;
	home.valid_alt = true;
}

void Commander::setHomePosValid()
{
	// play tune first time we initialize HOME
	if (!_status_flags.home_position_valid) {
		// tune_home_set(true); // deleted by Peixuan Shu
	}

	// mark home position as set
	_status_flags.home_position_valid = true;
}

void
Commander::updateHomePositionYaw(float yaw)
{
	if (_param_com_home_en.get()) {
		home_position_s home = _home_pub.get();

		home.yaw = yaw;
		home.timestamp = hrt_absolute_time();

		_home_pub.update(home);
	}
}

void
Commander::run()
{
	// static bool run_inited_ = false;   // added by Peixuan Shu

if (!run_inited_)  // added by Peixuan Shu
{
	run_inited_ = true;   // added by Peixuan Shu

	bool sensor_fail_tune_played = false;

	const param_t param_airmode = param_find("MC_AIRMODE");
	const param_t param_man_arm_gesture = param_find("MAN_ARM_GESTURE");
	const param_t param_rc_map_arm_sw = param_find("RC_MAP_ARM_SW");

	// /* initialize */
	// led_init();
	// buzzer_init();

// #if defined(BOARD_HAS_POWER_CONTROL)
// 	{
// 		// we need to do an initial publication to make sure uORB allocates the buffer, which cannot happen
// 		// in IRQ context.
// 		power_button_state_s button_state{};
// 		button_state.timestamp = hrt_absolute_time();
// 		button_state.event = 0xff;
// 		power_button_state_pub = orb_advertise(ORB_ID(power_button_state), &button_state);

// 		_power_button_state_sub.copy(&button_state);

// 		tune_control_s tune_control{};
// 		button_state.timestamp = hrt_absolute_time();
// 		tune_control_pub = orb_advertise(ORB_ID(tune_control), &tune_control);
// 	}

// 	if (board_register_power_state_notification_cb(power_button_state_notification_cb) != 0) {
// 		PX4_ERR("Failed to register power notification callback");
// 	}

// #endif // BOARD_HAS_POWER_CONTROL

	// get_circuit_breaker_params();  //deleted by Peixuan Shu

	bool param_init_forced = true;

	/* update vehicle status to find out vehicle type (required for preflight checks) */
	_status.system_type = _param_mav_type.get();

	// if (is_rotary_wing(_status) || is_vtol(_status)) {
	// 	_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	// } else if (is_fixed_wing(_status)) {
	// 	_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

	// } else if (is_ground_rover(_status)) {
	// 	_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROVER;

	// } else {
	// 	_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_UNKNOWN;
	// }

	_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING; // modified by Peixuan Shu

	// _status.is_vtol = is_vtol(_status);
	// _status.is_vtol_tailsitter = is_vtol_tailsitter(_status);

	_status.is_vtol = false;  // modified by Peixuan Shu
	_status.is_vtol_tailsitter = false;  // modified by Peixuan Shu

	_boot_timestamp = hrt_absolute_time();

	// initially set to failed
	_last_lpos_fail_time_us = _boot_timestamp;
	_last_gpos_fail_time_us = _boot_timestamp;
	_last_lvel_fail_time_us = _boot_timestamp;

	_status.system_id = _param_mav_sys_id.get();
	// arm_auth_init(&_mavlink_log_pub, &_status.system_id); //deleted by Peixuan Shu
	_status.component_id = _param_mav_comp_id.get();  //added by Peixuan Shu

	// // run preflight immediately to find all relevant parameters, but don't report
	// PreFlightCheck::preflightCheck(&_mavlink_log_pub, _status, _status_flags, _vehicle_control_mode,
	// 			       false, true, hrt_elapsed_time(&_boot_timestamp));

}

	// while (!should_exit()) { // change from while loop into one step by Peixuan Shu

		// perf_begin(_loop_perf); //deleted by Peixuan Shu

		/* update parameters */
		// const bool params_updated = _parameter_update_sub.updated();

		// if (params_updated || param_init_forced) {
		// 	// clear update
		// 	parameter_update_s update;
		// 	_parameter_update_sub.copy(&update);

			// update parameters from storage
			updateParams();

			// /* update parameters */
			// if (!_armed.armed) {
			// 	_status.system_type = _param_mav_type.get();

			// 	const bool is_rotary = is_rotary_wing(_status) || (is_vtol(_status) && _vtol_status.vtol_in_rw_mode);
			// 	const bool is_fixed = is_fixed_wing(_status) || (is_vtol(_status) && !_vtol_status.vtol_in_rw_mode);
			// 	const bool is_ground = is_ground_rover(_status);

			// 	/* disable manual override for all systems that rely on electronic stabilization */
			// 	if (is_rotary) {
			// 		_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

			// 	} else if (is_fixed) {
			// 		_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

			// 	} else if (is_ground) {
			// 		_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROVER;
			// 	}

			// 	/* set vehicle_status.is_vtol flag */
			// 	_status.is_vtol = is_vtol(_status);
			// 	_status.is_vtol_tailsitter = is_vtol_tailsitter(_status);

			// 	/* check and update system / component ID */
			// 	_status.system_id = _param_mav_sys_id.get();
			// 	_status.component_id = _param_mav_comp_id.get();

			// 	get_circuit_breaker_params();

			// 	_status_changed = true;
			// }

			_status_flags.avoidance_system_required = _param_com_obs_avoid.get();

			// _arm_requirements.arm_authorization = _param_arm_auth_required.get();
			// _arm_requirements.esc_check = _param_escs_checks_required.get();
			// _arm_requirements.global_position = !_param_arm_without_gps.get();
			// _arm_requirements.mission = _param_arm_mission_required.get();
			// _arm_requirements.geofence = _param_geofence_action.get() > geofence_result_s::GF_ACTION_NONE;

			_auto_disarm_killed.set_hysteresis_time_from(false, _param_com_kill_disarm.get() * 1_s);
			_offboard_available.set_hysteresis_time_from(true, _param_com_of_loss_t.get() * 1_s);

			// // disable arm gesture if an arm switch is configured
			// if (param_man_arm_gesture != PARAM_INVALID && param_rc_map_arm_sw != PARAM_INVALID) {
			// 	int32_t man_arm_gesture = 0, rc_map_arm_sw = 0;
			// 	param_get(param_man_arm_gesture, &man_arm_gesture);
			// 	param_get(param_rc_map_arm_sw, &rc_map_arm_sw);

			// 	if (rc_map_arm_sw > 0 && man_arm_gesture == 1) {
			// 		man_arm_gesture = 0; // disable arm gesture
			// 		param_set(param_man_arm_gesture, &man_arm_gesture);
			// 		mavlink_log_critical(&_mavlink_log_pub, "Arm stick gesture disabled if arm switch in use\t")
			// 		/* EVENT
			// 		* @description <param>MAN_ARM_GESTURE</param> is now set to disable arm/disarm stick gesture.
			// 		*/
			// 		events::send(events::ID("rc_update_arm_stick_gesture_disabled_with_switch"), {events::Log::Info, events::LogInternal::Disabled},
			// 			     "Arm stick gesture disabled if arm switch in use");
			// 	}
			// }

			// // check for unsafe Airmode settings: yaw airmode requires disabling the stick arm gesture
			// if (param_airmode != PARAM_INVALID && param_man_arm_gesture != PARAM_INVALID) {
			// 	int32_t airmode = 0, man_arm_gesture = 0;
			// 	param_get(param_airmode, &airmode);
			// 	param_get(param_man_arm_gesture, &man_arm_gesture);

			// 	if (airmode == 2 && man_arm_gesture == 1) {
			// 		airmode = 1; // change to roll/pitch airmode
			// 		param_set(param_airmode, &airmode);
			// 		mavlink_log_critical(&_mavlink_log_pub, "Yaw Airmode requires disabling the stick arm gesture\t")
			// 		/* EVENT
			// 		 * @description <param>MC_AIRMODE</param> is now set to roll/pitch airmode.
			// 		 */
			// 		events::send(events::ID("commander_airmode_requires_no_arm_gesture"), {events::Log::Error, events::LogInternal::Disabled},
			// 			     "Yaw Airmode requires disabling the stick arm gesture");
			// 	}
			// }

			// param_init_forced = false;
		// }

		/* Update OA parameter */
		_status_flags.avoidance_system_required = _param_com_obs_avoid.get();

// #if defined(BOARD_HAS_POWER_CONTROL)

// 		/* handle power button state */
// 		if (_power_button_state_sub.updated()) {
			// power_button_state_s button_state;

			// if (_power_button_state_sub.copy(&button_state)) {
			// 	if (button_state.event == power_button_state_s::PWR_BUTTON_STATE_REQUEST_SHUTDOWN) {
			// 		if (shutdown_if_allowed() && (px4_shutdown_request() == 0)) {
			// 			while (1) { px4_usleep(1); }
			// 		}
			// 	}
			// }
// 		}

// #endif // BOARD_HAS_POWER_CONTROL

		offboard_control_update(); /* Check if offboard control signal is lost */

		// if (_system_power_sub.updated()) {
			// system_power_s system_power{};
			// _system_power_sub.copy(&system_power);

			// if (hrt_elapsed_time(&system_power.timestamp) < 1_s) {
			// 	if (system_power.servo_valid &&
			// 	    !system_power.brick_valid &&
			// 	    !system_power.usb_connected) {
			// 		/* flying only on servo rail, this is unsafe */
			// 		_status_flags.power_input_valid = false;

			// 	} else {
			// 		_status_flags.power_input_valid = true;
			// 	}

			// 	_system_power_usb_connected = system_power.usb_connected;
			// }
		// }

		// /* Update land detector */
		// if (_vehicle_land_detected_sub.updated()) {
			// const bool was_landed = _vehicle_land_detected.landed;
			// _vehicle_land_detected_sub.copy(&_vehicle_land_detected);

			// // Only take actions if armed
			// if (_armed.armed) {
			// 	if (!was_landed && _vehicle_land_detected.landed) {
			// 		mavlink_log_info(&_mavlink_log_pub, "Landing detected\t");
			// 		events::send(events::ID("commander_landing_detected"), events::Log::Info, "Landing detected");
			// 		_status.takeoff_time = 0;

			// 	} else if (was_landed && !_vehicle_land_detected.landed) {
			// 		mavlink_log_info(&_mavlink_log_pub, "Takeoff detected\t");
			// 		events::send(events::ID("commander_takeoff_detected"), events::Log::Info, "Takeoff detected");
			// 		_status.takeoff_time = hrt_absolute_time();
			// 		_have_taken_off_since_arming = true;
			// 	}

			// 	// automatically set or update home position
			// 	if (_param_com_home_en.get() && !_home_pub.get().manual_home) {
			// 		// set the home position when taking off, but only if we were previously disarmed
			// 		// and at least 500 ms from commander start spent to avoid setting home on in-air restart
			// 		if (!_vehicle_land_detected.landed && (hrt_elapsed_time(&_boot_timestamp) > INAIR_RESTART_HOLDOFF_INTERVAL)) {
			// 			if (was_landed) {
			// 				set_home_position();

			// 			} else if (_param_com_home_in_air.get()
			// 				   && (!_home_pub.get().valid_lpos || !_home_pub.get().valid_hpos || !_home_pub.get().valid_alt)) {
			// 				set_in_air_home_position();
			// 			}
			// 		}
			// 	}
			// }
		// }

		// _safety_handler.safetyButtonHandler();

		// /* update safety topic */
		// const bool safety_updated = _safety_sub.updated();

		// if (safety_updated) {
			// const bool previous_safety_valid = (_safety.timestamp != 0);
			// const bool previous_safety_off = _safety.safety_off;

			// if (_safety_sub.copy(&_safety)) {
			// 	set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_MOTORCONTROL, _safety.safety_switch_available, _safety.safety_off,
			// 			 _safety.safety_switch_available, _status);

			// 	// disarm if safety is now on and still armed
			// 	if (_armed.armed && _safety.safety_switch_available && !_safety.safety_off
			// 	    && (_status.hil_state == vehicle_status_s::HIL_STATE_OFF)) {
			// 		disarm(arm_disarm_reason_t::safety_button);
			// 	}

			// 	// Notify the user if the status of the safety switch changes
			// 	if (previous_safety_valid && _safety.safety_switch_available && previous_safety_off != _safety.safety_off) {

			// 		if (_safety.safety_off) {
			// 			set_tune(tune_control_s::TUNE_ID_NOTIFY_POSITIVE);

			// 		} else {
			// 			tune_neutral(true);
			// 		}

			// 		_status_changed = true;
			// 	}
			// }
		// }

		// /* update vtol vehicle status*/
		// if (_vtol_vehicle_status_sub.updated()) {
			// /* vtol status changed */
			// _vtol_vehicle_status_sub.copy(&_vtol_status);
			// _status.vtol_fw_permanent_stab = _vtol_status.fw_permanent_stab;

			// /* Make sure that this is only adjusted if vehicle really is of type vtol */
			// if (is_vtol(_status)) {

			// 	// Check if there has been any change while updating the flags
			// 	const auto new_vehicle_type = _vtol_status.vtol_in_rw_mode ?
			// 				      vehicle_status_s::VEHICLE_TYPE_ROTARY_WING :
			// 				      vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

			// 	if (new_vehicle_type != _status.vehicle_type) {
			// 		_status.vehicle_type = _vtol_status.vtol_in_rw_mode ?
			// 				       vehicle_status_s::VEHICLE_TYPE_ROTARY_WING :
			// 				       vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
			// 		_status_changed = true;
			// 	}

			// 	if (_status.in_transition_mode != _vtol_status.vtol_in_trans_mode) {
			// 		_status.in_transition_mode = _vtol_status.vtol_in_trans_mode;
			// 		_status_changed = true;
			// 	}

			// 	if (_status.in_transition_to_fw != _vtol_status.in_transition_to_fw) {
			// 		_status.in_transition_to_fw = _vtol_status.in_transition_to_fw;
			// 		_status_changed = true;
			// 	}

			// 	if (_status_flags.vtol_transition_failure != _vtol_status.vtol_transition_failsafe) {
			// 		_status_flags.vtol_transition_failure = _vtol_status.vtol_transition_failsafe;
			// 		_status_changed = true;
			// 	}

			// 	const bool should_soft_stop = (_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);

			// 	if (_armed.soft_stop != should_soft_stop) {
			// 		_armed.soft_stop = should_soft_stop;
			// 		_status_changed = true;
			// 	}
			// }
		// }

		// if (_esc_status_sub.updated()) {
			// /* ESCs status changed */
			// esc_status_check();

		// } else if (_param_escs_checks_required.get() != 0) {

			// if (!_status_flags.escs_error) {

			// 	if ((_last_esc_status_updated != 0) && (hrt_elapsed_time(&_last_esc_status_updated) > 700_ms)) {
			// 		/* Detect timeout after first telemetry packet received
			// 		 * Some DShot ESCs are unresponsive for ~550ms during their initialization, so we use a timeout higher than that
			// 		 */

			// 		mavlink_log_critical(&_mavlink_log_pub, "ESCs telemetry timeout\t");
			// 		events::send(events::ID("commander_esc_telemetry_timeout"), events::Log::Critical,
			// 			     "ESCs telemetry timeout");
			// 		_status_flags.escs_error = true;

			// 	} else if (_last_esc_status_updated == 0 && hrt_elapsed_time(&_boot_timestamp) > 5000_ms) {
			// 		/* Detect if esc telemetry is not connected after reboot */
			// 		mavlink_log_critical(&_mavlink_log_pub, "ESCs telemetry not connected\t");
			// 		events::send(events::ID("commander_esc_telemetry_not_con"), events::Log::Critical,
			// 			     "ESCs telemetry not connected");
			// 		_status_flags.escs_error = true;
			// 	}
			// }
		// }

		estimator_check();

		// Auto disarm when landed or kill switch engaged
		// if (_armed.armed) {

			// // Check for auto-disarm on landing or pre-flight
			// if (_param_com_disarm_land.get() > 0 || _param_com_disarm_preflight.get() > 0) {

			// 	if (_param_com_disarm_land.get() > 0 && _have_taken_off_since_arming) {
			// 		_auto_disarm_landed.set_hysteresis_time_from(false, _param_com_disarm_land.get() * 1_s);
			// 		_auto_disarm_landed.set_state_and_update(_vehicle_land_detected.landed, hrt_absolute_time());

			// 	} else if (_param_com_disarm_preflight.get() > 0 && !_have_taken_off_since_arming) {
			// 		_auto_disarm_landed.set_hysteresis_time_from(false, _param_com_disarm_preflight.get() * 1_s);
			// 		_auto_disarm_landed.set_state_and_update(true, hrt_absolute_time());
			// 	}

			// 	if (_auto_disarm_landed.get_state()) {
			// 		if (_have_taken_off_since_arming) {
			// 			disarm(arm_disarm_reason_t::auto_disarm_land);

			// 		} else {
			// 			disarm(arm_disarm_reason_t::auto_disarm_preflight);
			// 		}
			// 	}
			// }

			// // Auto disarm after 5 seconds if kill switch is engaged
			// bool auto_disarm = _armed.manual_lockdown;

			// // auto disarm if locked down to avoid user confusion
			// //  skipped in HITL where lockdown is enabled for safety
			// if (_status.hil_state != vehicle_status_s::HIL_STATE_ON) {
			// 	auto_disarm |= _armed.lockdown;
			// }

			// _auto_disarm_killed.set_state_and_update(auto_disarm, hrt_absolute_time());

			// if (_auto_disarm_killed.get_state()) {
			// 	if (_armed.manual_lockdown) {
			// 		disarm(arm_disarm_reason_t::kill_switch, true);

			// 	} else {
			// 		disarm(arm_disarm_reason_t::lockdown, true);
			// 	}
			// }

		// } else {
			// _auto_disarm_landed.set_state_and_update(false, hrt_absolute_time());
			// _auto_disarm_killed.set_state_and_update(false, hrt_absolute_time());
		// }

		// if (_geofence_warning_action_on
		    // && _internal_state.main_state != commander_state_s::MAIN_STATE_AUTO_RTL
		    // && _internal_state.main_state != commander_state_s::MAIN_STATE_AUTO_LOITER
		    // && _internal_state.main_state != commander_state_s::MAIN_STATE_AUTO_LAND) {

			// // reset flag again when we switched out of it
			// _geofence_warning_action_on = false;
		// }

		if (_battery_status_subs.updated()) {
			// battery_status_check();
		}

		/* If in INIT state, try to proceed to STANDBY state */
		if (!_status_flags.calibration_enabled && _status.arming_state == vehicle_status_s::ARMING_STATE_INIT) {

			// _arm_state_machine.arming_state_transition(_status, _vehicle_control_mode, _safety,
			// 		vehicle_status_s::ARMING_STATE_STANDBY, _armed,
			// 		true /* fRunPreArmChecks */, &_mavlink_log_pub, _status_flags,
			// 		_arm_requirements, hrt_elapsed_time(&_boot_timestamp),
			// 		arm_disarm_reason_t::transition_to_standby);
			
			_status.arming_state = vehicle_status_s::ARMING_STATE_STANDBY; // switch to standby without check. Modified by Peixuan Shu
		}

		/* start mission result check */
		// if (_mission_result_sub.updated()) {
			// const mission_result_s &mission_result = _mission_result_sub.get();

			// const auto prev_mission_instance_count = mission_result.instance_count;
			// _mission_result_sub.update();

			// // if mission_result is valid for the current mission
			// const bool mission_result_ok = (mission_result.timestamp > _boot_timestamp)
			// 			       && (mission_result.instance_count > 0);

			// _status_flags.auto_mission_available = mission_result_ok && mission_result.valid;

			// if (mission_result_ok) {
			// 	if (_status.mission_failure != mission_result.failure) {
			// 		_status.mission_failure = mission_result.failure;
			// 		_status_changed = true;

			// 		if (_status.mission_failure) {
			// 			// navigator sends out the exact reason
			// 			mavlink_log_critical(&_mavlink_log_pub, "Mission cannot be completed\t");
			// 			events::send(events::ID("commander_mission_cannot_be_completed"), {events::Log::Critical, events::LogInternal::Info},
			// 				     "Mission cannot be completed");
			// 		}
			// 	}

			// 	/* Only evaluate mission state if home is set */
			// 	if (_status_flags.home_position_valid &&
			// 	    (prev_mission_instance_count != mission_result.instance_count)) {

			// 		if (!_status_flags.auto_mission_available) {
			// 			/* the mission is invalid */
			// 			tune_mission_fail(true);

			// 		} else if (mission_result.warning) {
			// 			/* the mission has a warning */
			// 			tune_mission_warn(true);

			// 		} else {
			// 			/* the mission is valid */
			// 			tune_mission_ok(true);
			// 		}
			// 	}
			// }

			// Transition main state to loiter or auto-mission after takeoff is completed.
			// if (_armed.armed && !_vehicle_land_detected.landed
			    // && (_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF ||
				// _status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF)
			    // && (mission_result.timestamp >= _status.nav_state_timestamp)
			    // && mission_result.finished) {

				// if ((_param_takeoff_finished_action.get() == 1) && _status_flags.auto_mission_available) {
				// 	main_state_transition(_status, commander_state_s::MAIN_STATE_AUTO_MISSION, _status_flags, _internal_state);

				// } else {
				// 	main_state_transition(_status, commander_state_s::MAIN_STATE_AUTO_LOITER, _status_flags, _internal_state);
				// }
			// }
		// }

		/* start geofence result check */
		// _geofence_result_sub.update(&_geofence_result);
		// _status.geofence_violated = _geofence_result.geofence_violated;

		// const bool in_low_battery_failsafe_delay = _battery_failsafe_timestamp != 0;

		// // Geofence actions
		// const bool geofence_action_enabled = _geofence_result.geofence_action != geofence_result_s::GF_ACTION_NONE;

		// if (_armed.armed
		    // && geofence_action_enabled
		    // && !in_low_battery_failsafe_delay) {

			// // check for geofence violation transition
			// if (_geofence_result.geofence_violated && !_geofence_violated_prev) {

				// switch (_geofence_result.geofence_action) {
				// case (geofence_result_s::GF_ACTION_NONE) : {
				// 		// do nothing
				// 		break;
				// 	}

				// case (geofence_result_s::GF_ACTION_WARN) : {
				// 		// do nothing, mavlink critical messages are sent by navigator
				// 		break;
				// 	}

				// case (geofence_result_s::GF_ACTION_LOITER) : {
				// 		if (TRANSITION_CHANGED == main_state_transition(_status, commander_state_s::MAIN_STATE_AUTO_LOITER, _status_flags,
				// 				_internal_state)) {
				// 			_geofence_loiter_on = true;
				// 		}

				// 		break;
				// 	}

				// case (geofence_result_s::GF_ACTION_RTL) : {
				// 		if (TRANSITION_CHANGED == main_state_transition(_status, commander_state_s::MAIN_STATE_AUTO_RTL, _status_flags,
				// 				_internal_state)) {
				// 			_geofence_rtl_on = true;
				// 		}

				// 		break;
				// 	}

				// case (geofence_result_s::GF_ACTION_LAND) : {
				// 		if (TRANSITION_CHANGED == main_state_transition(_status, commander_state_s::MAIN_STATE_AUTO_LAND, _status_flags,
				// 				_internal_state)) {
				// 			_geofence_land_on = true;
				// 		}

				// 		break;
				// 	}

				// case (geofence_result_s::GF_ACTION_TERMINATE) : {
				// 		PX4_WARN("Flight termination because of geofence");

				// 		if (!_flight_termination_triggered && !_lockdown_triggered) {
				// 			_flight_termination_triggered = true;
				// 			mavlink_log_critical(&_mavlink_log_pub, "Geofence violation! Flight terminated\t");
				// 			events::send(events::ID("commander_geofence_termination"), {events::Log::Alert, events::LogInternal::Warning},
				// 				     "Geofence violation! Flight terminated");
				// 			_armed.force_failsafe = true;
				// 			_status_changed = true;
				// 			send_parachute_command();
				// 		}

				// 		break;
				// 	}
				// }
			// }

			// _geofence_violated_prev = _geofence_result.geofence_violated;

			// // reset if no longer in LOITER or if manually switched to LOITER
			// const bool in_loiter_mode = _internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_LOITER;

			// if (!in_loiter_mode) {
			// 	_geofence_loiter_on = false;
			// }


			// reset if no longer in RTL or if manually switched to RTL
			// const bool in_rtl_mode = _internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_RTL;

			// if (!in_rtl_mode) {
				// _geofence_rtl_on = false;
			// }

			// // reset if no longer in LAND or if manually switched to LAND
			// const bool in_land_mode = _internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_LAND;

			// if (!in_land_mode) {
				// _geofence_land_on = false;
			// }

			// _geofence_warning_action_on = _geofence_warning_action_on || (_geofence_loiter_on || _geofence_rtl_on
						    //   || _geofence_land_on);

		// } else {
			// // No geofence checks, reset flags
			// _geofence_loiter_on = false;
			// _geofence_rtl_on = false;
			// _geofence_land_on = false;
			// _geofence_warning_action_on = false;
			// _geofence_violated_prev = false;
		// }

		/* Check for mission flight termination */
		// if (_armed.armed && _mission_result_sub.get().flight_termination &&
		    // !_status_flags.circuit_breaker_flight_termination_disabled) {


			// if (!_flight_termination_triggered && !_lockdown_triggered) {
				// // navigator only requests flight termination on GPS failure
				// mavlink_log_critical(&_mavlink_log_pub, "GPS failure: Flight terminated\t");
				// events::send(events::ID("commander_mission_termination"), {events::Log::Alert, events::LogInternal::Warning},
				// 	     "GPS failure: Flight terminated");
				// _flight_termination_triggered = true;
				// _armed.force_failsafe = true;
				// _status_changed = true;
				// send_parachute_command();
			// }

			// if (hrt_elapsed_time(&_last_termination_message_sent) > 4_s) {
				// _last_termination_message_sent = hrt_absolute_time();
				// mavlink_log_critical(&_mavlink_log_pub, "Flight termination active\t");
				// events::send(events::ID("commander_mission_termination_active"), {events::Log::Alert, events::LogInternal::Warning},
				// 	     "Flight termination active");
			// }
		// }

		// manual_control_setpoint_s manual_control_setpoint;

		// if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
			// if (manual_control_setpoint.valid) {
			// 	if (!_status_flags.rc_signal_found_once) {
			// 		_status_flags.rc_signal_found_once = true;
			// 		set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_RCRECEIVER, true, true,
			// 				 _status_flags.rc_calibration_valid, _status);
			// 		_status_changed = true;

			// 	} else {
			// 		if (_status.rc_signal_lost) {
			// 			if (_last_valid_manual_control_setpoint > 0) {
			// 				float elapsed = hrt_elapsed_time(&_last_valid_manual_control_setpoint) * 1e-6f;
			// 				mavlink_log_info(&_mavlink_log_pub, "Manual control regained after %.1fs\t", (double)elapsed);
			// 				events::send<float>(events::ID("commander_rc_regained"), events::Log::Info,
			// 						    "Manual control regained after {1:.1} s", elapsed);
			// 			}

			// 			set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_RCRECEIVER, true, true,
			// 					 _status_flags.rc_calibration_valid, _status);
			// 			_status_changed = true;
			// 		}
			// 	}

			// 	const bool mode_switch_mapped = (_param_rc_map_fltmode.get() > 0) || (_param_rc_map_mode_sw.get() > 0);
			// 	const bool is_mavlink = manual_control_setpoint.data_source > manual_control_setpoint_s::SOURCE_RC;

			// 	if (!_armed.armed && (is_mavlink || !mode_switch_mapped) && (_internal_state.main_state_changes == 0)) {
			// 		// if there's never been a mode change force position control as initial state
			// 		_internal_state.main_state = commander_state_s::MAIN_STATE_POSCTL;
			// 		_internal_state.main_state_changes++;
			// 	}

			// 	_status.rc_signal_lost = false;
			// 	_is_throttle_above_center = manual_control_setpoint.z > 0.6f;
			// 	_is_throttle_low = manual_control_setpoint.z < 0.1f;
			// 	_last_valid_manual_control_setpoint = manual_control_setpoint.timestamp;

			// } else {
			// 	if (_status_flags.rc_signal_found_once && !_status.rc_signal_lost) {
			// 		mavlink_log_critical(&_mavlink_log_pub, "Manual control lost\t");
			// 		events::send(events::ID("commander_rc_lost"), {events::Log::Critical, events::LogInternal::Info},
			// 			     "Manual control lost");
			// 		_status.rc_signal_lost = true;
			// 		set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_RCRECEIVER, true, true,
			// 				 false, _status);
			// 		_status_changed = true;
			// 	}
			// }


			// const bool override_enabled =
			// 	((_param_com_rc_override.get() & static_cast<int32_t>(RcOverrideBits::AUTO_MODE_BIT))
			// 	 && _vehicle_control_mode.flag_control_auto_enabled)
			// 	|| ((_param_com_rc_override.get() & static_cast<int32_t>(RcOverrideBits::OFFBOARD_MODE_BIT))
			// 	    && _vehicle_control_mode.flag_control_offboard_enabled);

			// // Abort autonomous mode and switch to position mode if sticks are moved significantly
			// // but only if actually in air.
			// if ((_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING)
			//     && !in_low_battery_failsafe_delay && !_geofence_warning_action_on
			//     && _armed.armed
			//     && !_status_flags.rc_calibration_in_progress
			//     && manual_control_setpoint.valid
			//     && manual_control_setpoint.sticks_moving
			//     && override_enabled) {
			// 	const transition_result_t posctl_result =
			// 		main_state_transition(_status, commander_state_s::MAIN_STATE_POSCTL, _status_flags, _internal_state);

			// 	if (posctl_result == TRANSITION_CHANGED) {
			// 		tune_positive(true);
			// 		mavlink_log_info(&_mavlink_log_pub, "Pilot took over position control using sticks\t");
			// 		events::send(events::ID("commander_rc_override_pos"), events::Log::Info,
			// 			     "Pilot took over position control using sticks");
			// 		_status_changed = true;

			// 	} else if (posctl_result == TRANSITION_DENIED) {
			// 		// If transition to POSCTL was denied, then we can try again with ALTCTL.
			// 		const transition_result_t altctl_result =
			// 			main_state_transition(_status, commander_state_s::MAIN_STATE_ALTCTL, _status_flags, _internal_state);

			// 		if (altctl_result == TRANSITION_CHANGED) {
			// 			tune_positive(true);
			// 			mavlink_log_info(&_mavlink_log_pub, "Pilot took over altitude control using sticks\t");
			// 			events::send(events::ID("commander_rc_override_alt"), events::Log::Info,
			// 				     "Pilot took over altitude control using sticks");
			// 			_status_changed = true;
			// 		}
			// 	}
			// }
		// }

		// // data link checks which update the status
		// data_link_check();

		// avoidance_check();

		// // engine failure detection
		// // TODO: move out of commander
		// if (_actuator_controls_sub.updated()) {
		// 	/* Check engine failure
		// 	 * only for fixed wing for now
		// 	 */
			// if (!_status_flags.circuit_breaker_engaged_enginefailure_check &&
			//     _status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING && !_status.is_vtol && _armed.armed) {

			// 	actuator_controls_s actuator_controls{};
			// 	_actuator_controls_sub.copy(&actuator_controls);

			// 	const float throttle = actuator_controls.control[actuator_controls_s::INDEX_THROTTLE];
			// 	const float current2throttle = _battery_current / throttle;

			// 	if (((throttle > _param_ef_throttle_thres.get()) && (current2throttle < _param_ef_current2throttle_thres.get()))
			// 	    || _status.engine_failure) {

			// 		const float elapsed = hrt_elapsed_time(&_timestamp_engine_healthy) / 1e6f;

			// 		/* potential failure, measure time */
			// 		if ((_timestamp_engine_healthy > 0) && (elapsed > _param_ef_time_thres.get())
			// 		    && !_status.engine_failure) {

			// 			_status.engine_failure = true;
			// 			_status_changed = true;

			// 			PX4_ERR("Engine Failure");
			// 			set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_MOTORCONTROL, true, true, false, _status);
			// 		}
			// 	}

			// } else {
			// 	/* no failure reset flag */
			// 	_timestamp_engine_healthy = hrt_absolute_time();

			// 	if (_status.engine_failure) {
			// 		_status.engine_failure = false;
			// 		_status_changed = true;
			// 	}
			// }
		// }

		/* check if we are disarmed and there is a better mode to wait in */
		// if (!_armed.armed) {
			/* if there is no radio control but GPS lock the user might want to fly using
			 * just a tablet. Since the RC will force its mode switch setting on connecting
			 * we can as well just wait in a hold mode which enables tablet control.
			 */
			// if (_status.rc_signal_lost && (_internal_state.main_state == commander_state_s::MAIN_STATE_MANUAL)
			//     && _status_flags.global_position_valid) {

				// main_state_transition(_status, commander_state_s::MAIN_STATE_AUTO_LOITER, _status_flags, _internal_state);
			// }
		// }

		/* handle commands last, as the system needs to be updated to handle them */
		if (_vehicle_command_sub.updated()) {
			/* got command */
			// const unsigned last_generation = _vehicle_command_sub.get_last_generation();
			vehicle_command_s cmd;

			if (_vehicle_command_sub.copy(&cmd)) {
				// if (_vehicle_command_sub.get_last_generation() != last_generation + 1) {
				// 	PX4_ERR("vehicle_command lost, generation %u -> %u", last_generation, _vehicle_command_sub.get_last_generation());
				// }

  				/* Handle command. Call main_state_transition() to update _internal_state.main_state */
				if (handle_command(cmd)) {
					_status_changed = true;
				}
			}
		}

		// if (_action_request_sub.updated()) {
			// const unsigned last_generation = _action_request_sub.get_last_generation();
			// action_request_s action_request;

			// if (_action_request_sub.copy(&action_request)) {
			// 	if (_action_request_sub.get_last_generation() != last_generation + 1) {
			// 		PX4_ERR("action_request lost, generation %u -> %u", last_generation, _action_request_sub.get_last_generation());
			// 	}

			// 	executeActionRequest(action_request);
			// }
		// }

		/* Check for failure detector status */
		// if (_failure_detector.update(_status, _vehicle_control_mode)) {
			// _status.failure_detector_status = _failure_detector.getStatus().value;
			// auto fd_status_flags = _failure_detector.getStatusFlags();
			// _status_changed = true;

			// if (_armed.armed) {
			// 	if (fd_status_flags.arm_escs) {
			// 		// 500ms is the PWM spoolup time. Within this timeframe controllers are not affecting actuator_outputs
			// 		if (hrt_elapsed_time(&_status.armed_time) < 500_ms) {
			// 			disarm(arm_disarm_reason_t::failure_detector);
			// 			mavlink_log_critical(&_mavlink_log_pub, "ESCs did not respond to arm request\t");
			// 			events::send(events::ID("commander_fd_escs_not_arming"), events::Log::Critical, "ESCs did not respond to arm request");
			// 		}
			// 	}

			// 	if (fd_status_flags.roll || fd_status_flags.pitch || fd_status_flags.alt || fd_status_flags.ext) {
			// 		const bool is_right_after_takeoff = hrt_elapsed_time(&_status.takeoff_time) < (1_s * _param_com_lkdown_tko.get());

			// 		if (is_right_after_takeoff && !_lockdown_triggered) {
			// 			// This handles the case where something fails during the early takeoff phase
			// 			_armed.lockdown = true;
			// 			_lockdown_triggered = true;
			// 			mavlink_log_emergency(&_mavlink_log_pub, "Critical failure detected: lockdown\t");
			// 			/* EVENT
			// 			 * @description
			// 			 * When a critical failure is detected right after takeoff, the system turns off the motors.
			// 			 * Failures include an exceeding tilt angle, altitude failure or an external failure trigger.
			// 			 *
			// 			 * <profile name="dev">
			// 			 * This can be configured with the parameter <param>COM_LKDOWN_TKO</param>.
			// 			 * </profile>
			// 			 */
			// 			events::send(events::ID("commander_fd_lockdown"), {events::Log::Emergency, events::LogInternal::Warning},
			// 				     "Critical failure detected: lockdown");

			// 		} else if (!_status_flags.circuit_breaker_flight_termination_disabled &&
			// 			   !_flight_termination_triggered && !_lockdown_triggered) {

			// 			_armed.force_failsafe = true;
			// 			_flight_termination_triggered = true;
			// 			mavlink_log_emergency(&_mavlink_log_pub, "Critical failure detected: terminate flight\t");
			// 			/* EVENT
			// 			 * @description
			// 			 * Critical failures include an exceeding tilt angle, altitude failure or an external failure trigger.
			// 			 *
			// 			 * <profile name="dev">
			// 			 * Flight termination can be disabled with the parameter <param>CBRK_FLIGHTTERM</param>.
			// 			 * </profile>
			// 			 */
			// 			events::send(events::ID("commander_fd_terminate"), {events::Log::Emergency, events::LogInternal::Warning},
			// 				     "Critical failure detected: terminate flight");
			// 			send_parachute_command();
			// 		}
			// 	}

			// 	if (fd_status_flags.imbalanced_prop
			// 	    && !_imbalanced_propeller_check_triggered) {
			// 		_status_changed = true;
			// 		_imbalanced_propeller_check_triggered = true;
			// 		imbalanced_prop_failsafe(&_mavlink_log_pub, _status, _status_flags, &_internal_state,
			// 					 (imbalanced_propeller_action_t)_param_com_imb_prop_act.get());
			// 	}
			// }
		// }

		// Publish wind speed warning if enabled via parameter
		// if (_param_com_wind_warn.get() > FLT_EPSILON && !_vehicle_land_detected.landed) {
			// checkWindAndWarn();
		// }

		_status_flags.flight_terminated = _armed.force_failsafe || _armed.manual_lockdown;

		/* Get current timestamp */
		const hrt_abstime now = hrt_absolute_time();

		// automatically set or update home position
		if (_param_com_home_en.get() && !_home_pub.get().manual_home) {
			if (!_armed.armed && _vehicle_land_detected.landed) {
				const bool can_set_home_lpos_first_time = (!_home_pub.get().valid_lpos && _status_flags.local_position_valid);
				const bool can_set_home_gpos_first_time = ((!_home_pub.get().valid_hpos || !_home_pub.get().valid_alt)
						&& (_status_flags.global_position_valid || _status_flags.gps_position_valid));
				const bool can_set_home_alt_first_time = (!_home_pub.get().valid_alt && _local_position_sub.get().z_global);

				if (can_set_home_lpos_first_time
				    || can_set_home_gpos_first_time
				    || can_set_home_alt_first_time
				    || hasMovedFromCurrentHomeLocation()) {
					set_home_position();
				}
			}
		}

		// check for arming state change
		if (_was_armed != _armed.armed) {
			_status_changed = true;

			if (_armed.armed) {
				if (!_vehicle_land_detected.landed) { // check if takeoff already detected upon arming
					_have_taken_off_since_arming = true;
				}

			} else { // increase the flight uuid upon disarming
				const int32_t flight_uuid = _param_flight_uuid.get() + 1;
				_param_flight_uuid.set(flight_uuid);
				_param_flight_uuid.commit_no_notification();

				_last_disarmed_timestamp = hrt_absolute_time();
			}
		}

		if (!_armed.armed) {
			/* Reset the flag if disarmed. */
			_have_taken_off_since_arming = false;
			_imbalanced_propeller_check_triggered = false;
		}

		/* now set navigation state according to failsafe and main state */
		// change _status.nav_state (containing failsafe decision logic)
		bool nav_state_changed = set_nav_state(_status,
						       _armed,
						       _internal_state,
						       /* &_mavlink_log_pub, */
						       static_cast<link_loss_actions_t>(_param_nav_dll_act.get()),
						       /* _mission_result_sub.get().finished,*/ true,
						       /* _mission_result_sub.get().stay_in_failsafe,*/ false,
						       _status_flags,
						       _vehicle_land_detected.landed,
						       static_cast<link_loss_actions_t>(_param_nav_rcl_act.get()),
						       static_cast<offboard_loss_actions_t>(_param_com_obl_act.get()),
						       static_cast<quadchute_actions_t>(_param_com_qc_act.get()),
						       static_cast<offboard_loss_rc_actions_t>(_param_com_obl_rc_act.get()),
						       static_cast<position_nav_loss_actions_t>(_param_com_posctl_navl.get()),
						       _param_com_rcl_act_t.get(),
						       _param_com_rcl_except.get());

		if (nav_state_changed) {
			_status.nav_state_timestamp = hrt_absolute_time();
		}

		if (_status.failsafe != _failsafe_old) {
			_status_changed = true;

			if (_status.failsafe) {
				// mavlink_log_info(&_mavlink_log_pub, "Failsafe mode activated\t");
				// events::send(events::ID("commander_failsafe_activated"), events::Log::Info, "Failsafe mode activated");

				std::cerr << "[PX4 Commander.cpp] Failsafe mode activated" << std::endl; // added by Peixuan Shu

			} else {
				// mavlink_log_info(&_mavlink_log_pub, "Failsafe mode deactivated\t");
				// events::send(events::ID("commander_failsafe_deactivated"), events::Log::Info, "Failsafe mode deactivated");

				std::cerr << "[PX4 Commander.cpp] Failsafe mode deactivated" << std::endl; // added by Peixuan Shu
			}

			_failsafe_old = _status.failsafe;
		}

		/* publish states (armed, control_mode, vehicle_status, commander_state, vehicle_status_flags, failure_detector_status) at 2 Hz or immediately when changed */
		if (hrt_elapsed_time(&_status.timestamp) >= 500_ms || _status_changed || nav_state_changed) {

			/* Publish vehicle_control_mode flag uorb messages for pos/att controller based on the flight mode */
			update_control_mode(); 

			_status.timestamp = hrt_absolute_time();
			_status_pub.publish(_status); // publish vehicle_status uorb messages

			// switch ((PrearmedMode)_param_com_prearm_mode.get()) {
			// case PrearmedMode::DISABLED:
			// 	/* skip prearmed state  */
			// 	_armed.prearmed = false;
			// 	break;

			// case PrearmedMode::ALWAYS:
			// 	/* safety is not present, go into prearmed
			// 	* (all output drivers should be started / unlocked last in the boot process
			// 	* when the rest of the system is fully initialized)
			// 	*/
			// 	_armed.prearmed = (hrt_elapsed_time(&_boot_timestamp) > 5_s);
			// 	break;

			// case PrearmedMode::SAFETY_BUTTON:
			// 	if (_safety.safety_switch_available) {
			// 		/* safety switch is present, go into prearmed if safety is off */
			// 		_armed.prearmed = _safety.safety_off;

			// 	} else {
			// 		/* safety switch is not present, do not go into prearmed */
			// 		_armed.prearmed = false;
			// 	}

			// 	break;

			// default:
			// 	_armed.prearmed = false;
			// 	break;
			// }

			_armed.timestamp = hrt_absolute_time();
			_armed_pub.publish(_armed); // publish armed uorb messages

			/* publish internal state for logging purposes */
			_internal_state.timestamp = hrt_absolute_time();
			_commander_state_pub.publish(_internal_state);

			// Evaluate current prearm status
			// if (!_armed.armed && !_status_flags.calibration_enabled) {
				// perf_begin(_preflight_check_perf);
				// bool preflight_check_res = PreFlightCheck::preflightCheck(nullptr, _status, _status_flags, _vehicle_control_mode,
				// 			   false, true, hrt_elapsed_time(&_boot_timestamp));
				// perf_end(_preflight_check_perf);

				// // skip arm authorization check until actual arming attempt
				// PreFlightCheck::arm_requirements_t arm_req = _arm_requirements;
				// arm_req.arm_authorization = false;
				// bool prearm_check_res = PreFlightCheck::preArmCheck(nullptr, _status_flags, _vehicle_control_mode, _safety, arm_req,
				// 			_status, false);

				// set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_PREARM_CHECK, true, true, (preflight_check_res
				// 		 && prearm_check_res), _status);
			// }

			/* publish vehicle_status_flags */
			_status_flags.timestamp = hrt_absolute_time();
			_vehicle_status_flags_pub.publish(_status_flags);

			// /* publish failure_detector data */
			// failure_detector_status_s fd_status{};
			// fd_status.timestamp = hrt_absolute_time();
			// fd_status.fd_roll = _failure_detector.getStatusFlags().roll;
			// fd_status.fd_pitch = _failure_detector.getStatusFlags().pitch;
			// fd_status.fd_alt = _failure_detector.getStatusFlags().alt;
			// fd_status.fd_ext = _failure_detector.getStatusFlags().ext;
			// fd_status.fd_arm_escs = _failure_detector.getStatusFlags().arm_escs;
			// fd_status.fd_high_wind = _failure_detector.getStatusFlags().high_wind;
			// fd_status.fd_battery = _failure_detector.getStatusFlags().battery;
			// fd_status.fd_imbalanced_prop = _failure_detector.getStatusFlags().imbalanced_prop;
			// fd_status.imbalanced_prop_metric = _failure_detector.getImbalancedPropMetric();
			// _failure_detector_status_pub.publish(fd_status);
		}

		// /* play arming and battery warning tunes */
		// if (!_arm_tune_played && _armed.armed &&
		//     (_safety.safety_switch_available || (_safety.safety_switch_available && _safety.safety_off))) {

		// 	/* play tune when armed */
		// 	set_tune(tune_control_s::TUNE_ID_ARMING_WARNING);
		// 	_arm_tune_played = true;

		// } else if (!_status_flags.usb_connected &&
		// 	   (_status.hil_state != vehicle_status_s::HIL_STATE_ON) &&
		// 	   (_battery_warning == battery_status_s::BATTERY_WARNING_CRITICAL)) {
		// 	/* play tune on battery critical */
		// 	set_tune(tune_control_s::TUNE_ID_BATTERY_WARNING_FAST);

		// } else if ((_status.hil_state != vehicle_status_s::HIL_STATE_ON) &&
		// 	   (_battery_warning == battery_status_s::BATTERY_WARNING_LOW)) {
		// 	/* play tune on battery warning */
		// 	set_tune(tune_control_s::TUNE_ID_BATTERY_WARNING_SLOW);

		// } else if (_status.failsafe && _armed.armed) {
		// 	tune_failsafe(true);

		// } else {
		// 	set_tune(tune_control_s::TUNE_ID_STOP);
		// }

		// /* reset arm_tune_played when disarmed */
		// if (!_armed.armed || (_safety.safety_switch_available && !_safety.safety_off)) {

		// 	// Notify the user that it is safe to approach the vehicle
		// 	if (_arm_tune_played) {
		// 		tune_neutral(true);
		// 	}

		// 	_arm_tune_played = false;
		// }

		// /* play sensor failure tunes if we already waited for hotplug sensors to come up and failed */
		// _status_flags.system_hotplug_timeout = (hrt_elapsed_time(&_boot_timestamp) > HOTPLUG_SENS_TIMEOUT);

		// if (!sensor_fail_tune_played && (!_status_flags.system_sensors_initialized
		// 				 && _status_flags.system_hotplug_timeout)) {

		// 	set_tune_override(tune_control_s::TUNE_ID_GPS_WARNING);
		// 	sensor_fail_tune_played = true;
		// 	_status_changed = true;
		// }

		// check if the worker has finished
		// if (_worker_thread.hasResult()) {
			// int ret = _worker_thread.getResultAndReset();
			// _armed.in_esc_calibration_mode = false;

			// if (_status_flags.calibration_enabled) { // did we do a calibration?
			// 	_status_flags.calibration_enabled = false;

			// 	if (ret == 0) {
			// 		tune_positive(true);

			// 	} else {
			// 		tune_negative(true);
			// 	}
			// }
		// }

		// control_status_leds(_status_changed, _battery_warning);

		_status_changed = false;

		/* store last position lock state */
		_last_local_altitude_valid = _status_flags.local_altitude_valid;
		_last_local_position_valid = _status_flags.local_position_valid;
		_last_global_position_valid = _status_flags.global_position_valid;

		_was_armed = _armed.armed;

		// arm_auth_update(now, params_updated || param_init_forced);

		// px4_indicate_external_reset_lockout(LockoutComponent::Commander, _armed.armed);

		// perf_end(_loop_perf);

		// // sleep if there are no vehicle_commands or action_requests to process
		// if (!_vehicle_command_sub.updated() && !_action_request_sub.updated()) {
		// 	px4_usleep(COMMANDER_MONITORING_INTERVAL);
		// }
	// }   // change from while loop into one step by Peixuan Shu

	// rgbled_set_color_and_mode(led_control_s::COLOR_WHITE, led_control_s::MODE_OFF);

	// /* close fds */
	// led_deinit();
	// buzzer_deinit();
}


/* check if pos/vel from estimator is valid */
bool Commander::check_posvel_validity(const bool data_valid, const float data_accuracy, const float required_accuracy,
				      const hrt_abstime &data_timestamp_us, hrt_abstime &last_fail_time_us,
				      const bool was_valid)
{
	bool valid = was_valid;
	const bool data_stale = ((hrt_elapsed_time(&data_timestamp_us) > _param_com_pos_fs_delay.get() * 1_s)
				 || (data_timestamp_us == 0));
	const float req_accuracy = (was_valid ? required_accuracy * 2.5f : required_accuracy);
	const bool level_check_pass = data_valid && !data_stale && (data_accuracy < req_accuracy);

	// Check accuracy with hysteresis in both test level and time
	if (level_check_pass) {
		if (!was_valid) {
			// check if probation period has elapsed
			if (hrt_elapsed_time(&last_fail_time_us) > 1_s) {
				valid = true;
			}
		}

	} else {
		// level check failed
		if (was_valid) {
			// FAILURE! no longer valid
			valid = false;
		}

		last_fail_time_us = hrt_absolute_time();
	}

	if (was_valid != valid) {
		_status_changed = true;
	}

	return valid;
}

/* Specify vehicle control flags for pos/att controller based on the flight mode */
void
Commander::update_control_mode()
{
	_vehicle_control_mode = {};

	/* set vehicle_control_mode according to set_navigation_state */
	_vehicle_control_mode.flag_armed = _armed.armed;

	switch (_status.nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		_vehicle_control_mode.flag_control_manual_enabled = true;
		_vehicle_control_mode.flag_control_rates_enabled = stabilization_required();
		_vehicle_control_mode.flag_control_attitude_enabled = stabilization_required();
		break;

	case vehicle_status_s::NAVIGATION_STATE_STAB:
		_vehicle_control_mode.flag_control_manual_enabled = true;
		_vehicle_control_mode.flag_control_rates_enabled = true;
		_vehicle_control_mode.flag_control_attitude_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		_vehicle_control_mode.flag_control_manual_enabled = true;
		_vehicle_control_mode.flag_control_rates_enabled = true;
		_vehicle_control_mode.flag_control_attitude_enabled = true;
		_vehicle_control_mode.flag_control_altitude_enabled = true;
		_vehicle_control_mode.flag_control_climb_rate_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		_vehicle_control_mode.flag_control_manual_enabled = true;
		_vehicle_control_mode.flag_control_rates_enabled = true;
		_vehicle_control_mode.flag_control_attitude_enabled = true;
		_vehicle_control_mode.flag_control_altitude_enabled = true;
		_vehicle_control_mode.flag_control_climb_rate_enabled = true;
		_vehicle_control_mode.flag_control_position_enabled = true;
		_vehicle_control_mode.flag_control_velocity_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF:
		_vehicle_control_mode.flag_control_auto_enabled = true;
		_vehicle_control_mode.flag_control_rates_enabled = true;
		_vehicle_control_mode.flag_control_attitude_enabled = true;
		_vehicle_control_mode.flag_control_altitude_enabled = true;
		_vehicle_control_mode.flag_control_climb_rate_enabled = true;
		_vehicle_control_mode.flag_control_position_enabled = true;
		_vehicle_control_mode.flag_control_velocity_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ACRO:
		_vehicle_control_mode.flag_control_manual_enabled = true;
		_vehicle_control_mode.flag_control_rates_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_DESCEND:
		_vehicle_control_mode.flag_control_auto_enabled = true;
		_vehicle_control_mode.flag_control_rates_enabled = true;
		_vehicle_control_mode.flag_control_attitude_enabled = true;
		_vehicle_control_mode.flag_control_climb_rate_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
		/* disable all controllers on termination */
		_vehicle_control_mode.flag_control_termination_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
		_vehicle_control_mode.flag_control_offboard_enabled = true;

		// offboard_control_mode is published in mavlink_receiver.cpp when receiving setpoint_raw
		if (_offboard_control_mode_sub.get().position) {
			_vehicle_control_mode.flag_control_position_enabled = true;
			_vehicle_control_mode.flag_control_velocity_enabled = true;
			_vehicle_control_mode.flag_control_altitude_enabled = true;
			_vehicle_control_mode.flag_control_climb_rate_enabled = true;
			_vehicle_control_mode.flag_control_acceleration_enabled = true;
			_vehicle_control_mode.flag_control_rates_enabled = true;
			_vehicle_control_mode.flag_control_attitude_enabled = true;

		} else if (_offboard_control_mode_sub.get().velocity) {
			_vehicle_control_mode.flag_control_velocity_enabled = true;
			_vehicle_control_mode.flag_control_altitude_enabled = true;
			_vehicle_control_mode.flag_control_climb_rate_enabled = true;
			_vehicle_control_mode.flag_control_acceleration_enabled = true;
			_vehicle_control_mode.flag_control_rates_enabled = true;
			_vehicle_control_mode.flag_control_attitude_enabled = true;

		} else if (_offboard_control_mode_sub.get().acceleration) {
			_vehicle_control_mode.flag_control_acceleration_enabled = true;
			_vehicle_control_mode.flag_control_rates_enabled = true;
			_vehicle_control_mode.flag_control_attitude_enabled = true;

		} else if (_offboard_control_mode_sub.get().attitude) {
			_vehicle_control_mode.flag_control_rates_enabled = true;
			_vehicle_control_mode.flag_control_attitude_enabled = true;

		} else if (_offboard_control_mode_sub.get().body_rate) {
			_vehicle_control_mode.flag_control_rates_enabled = true;
		}

		break;

	case vehicle_status_s::NAVIGATION_STATE_ORBIT:
		_vehicle_control_mode.flag_control_manual_enabled = false;
		_vehicle_control_mode.flag_control_auto_enabled = false;
		_vehicle_control_mode.flag_control_rates_enabled = true;
		_vehicle_control_mode.flag_control_attitude_enabled = true;
		_vehicle_control_mode.flag_control_altitude_enabled = true;
		_vehicle_control_mode.flag_control_climb_rate_enabled = true;
		_vehicle_control_mode.flag_control_position_enabled = true;
		_vehicle_control_mode.flag_control_velocity_enabled = true;
		break;

	default:
		break;
	}

	_vehicle_control_mode.flag_multicopter_position_control_enabled =
		(_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING)
		&& (_vehicle_control_mode.flag_control_altitude_enabled
		    || _vehicle_control_mode.flag_control_climb_rate_enabled
		    || _vehicle_control_mode.flag_control_position_enabled
		    || _vehicle_control_mode.flag_control_velocity_enabled
		    || _vehicle_control_mode.flag_control_acceleration_enabled);

	_vehicle_control_mode.timestamp = hrt_absolute_time();
	_control_mode_pub.publish(_vehicle_control_mode);
}

bool
Commander::stabilization_required()
{
	//delete vtol decision by Peixuan Shu
	return (_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING ||		// is a rotary wing, or
		_status.vtol_fw_permanent_stab  /* || 	// is a VTOL in fixed wing mode and stabilisation is on, or
		(_vtol_status.vtol_in_trans_mode && 	// is currently a VTOL transitioning AND
		 _status.vehicle_type ==
		 vehicle_status_s::VEHICLE_TYPE_FIXED_WING) */);	// is a fixed wing, ie: transitioning back to rotary wing mode
}

/* Send back command ack */
void Commander::answer_command(const vehicle_command_s &cmd, uint8_t result)
{
	switch (result) {
	case vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED:
		break;

	case vehicle_command_s::VEHICLE_CMD_RESULT_DENIED:
		// tune_negative(true);  /* Blink red LED and play negative tune (if use_buzzer == true). */
		break;

	case vehicle_command_s::VEHICLE_CMD_RESULT_FAILED:
		// tune_negative(true);  /* Blink red LED and play negative tune (if use_buzzer == true). */
		break;

	case vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
		// tune_negative(true);  /* Blink red LED and play negative tune (if use_buzzer == true). */
		break;

	case vehicle_command_s::VEHICLE_CMD_RESULT_UNSUPPORTED:
		// tune_negative(true);  /* Blink red LED and play negative tune (if use_buzzer == true). */
		break;

	default:
		break;
	}

	/* publish ACK */
	vehicle_command_ack_s command_ack{};
	command_ack.command = cmd.command;
	command_ack.result = result;
	command_ack.target_system = cmd.source_system;
	command_ack.target_component = cmd.source_component;
	command_ack.timestamp = hrt_absolute_time();
	_command_ack_pub.publish(command_ack);
}

/* Check if estimator is valid */
void Commander::estimator_check()
{
	// Check if quality checking of position accuracy and consistency is to be performed
	const bool run_quality_checks = !_status_flags.circuit_breaker_engaged_posfailure_check;

	_local_position_sub.update();
	_global_position_sub.update();

	const vehicle_local_position_s &lpos = _local_position_sub.get();

	if (lpos.heading_reset_counter != _heading_reset_counter) {
		if (_status_flags.home_position_valid) {
			updateHomePositionYaw(_home_pub.get().yaw + lpos.delta_heading);
		}

		_heading_reset_counter = lpos.heading_reset_counter;
	}

	// const bool mag_fault_prev = _estimator_status_flags_sub.get().cs_mag_fault;
	// const bool gnss_heading_fault_prev = _estimator_status_flags_sub.get().cs_gps_yaw_fault;

	// // use primary estimator_status
	// if (_estimator_selector_status_sub.updated()) {
		// estimator_selector_status_s estimator_selector_status;

		// if (_estimator_selector_status_sub.copy(&estimator_selector_status)) {
		// 	if (estimator_selector_status.primary_instance != _estimator_status_sub.get_instance()) {
		// 		_estimator_status_sub.ChangeInstance(estimator_selector_status.primary_instance);
		// 		_estimator_status_flags_sub.ChangeInstance(estimator_selector_status.primary_instance);
		// 	}
		// }
	// }

	// if (_estimator_status_flags_sub.update()) {
		// const estimator_status_flags_s &estimator_status_flags = _estimator_status_flags_sub.get();

		// _status_flags.dead_reckoning = estimator_status_flags.cs_wind_dead_reckoning
		// 			       || estimator_status_flags.cs_inertial_dead_reckoning;

		// if (!(estimator_status_flags.cs_inertial_dead_reckoning || estimator_status_flags.cs_wind_dead_reckoning)) {
		// 	// position requirements (update if not dead reckoning)
		// 	bool gps             = estimator_status_flags.cs_gps;
		// 	bool optical_flow    = estimator_status_flags.cs_opt_flow;
		// 	bool vision_position = estimator_status_flags.cs_ev_pos;

		// 	_status_flags.position_reliant_on_gps             =  gps && !optical_flow && !vision_position;
		// 	_status_flags.position_reliant_on_optical_flow    = !gps &&  optical_flow && !vision_position;
		// 	_status_flags.position_reliant_on_vision_position = !gps && !optical_flow &&  vision_position;
		// }

		// // Check for a magnetomer fault and notify the user
		// if (!mag_fault_prev && estimator_status_flags.cs_mag_fault) {
		// 	mavlink_log_critical(&_mavlink_log_pub, "Compass needs calibration - Land now!\t");
		// 	events::send(events::ID("commander_stopping_mag_use"), events::Log::Critical,
		// 		     "Stopping compass use! Land now and calibrate the compass");
		// 	set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_MAG, true, true, false, _status);
		// }

		// if (!gnss_heading_fault_prev && estimator_status_flags.cs_gps_yaw_fault) {
		// 	mavlink_log_critical(&_mavlink_log_pub, "GNSS heading not reliable - Land now!\t");
		// 	events::send(events::ID("commander_stopping_gnss_heading_use"), events::Log::Critical,
		// 		     "GNSS heading not reliable. Land now!");
		// 	set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_GPS, true, true, false, _status);
		// }
	// }


	// /* Check estimator status for signs of bad yaw induced post takeoff navigation failure
	// * for a short time interval after takeoff.
	// * Most of the time, the drone can recover from a bad initial yaw using GPS-inertial
	// * heading estimation (yaw emergency estimator) or GPS heading (fixed wings only), but
	// * if this does not fix the issue we need to stop using a position controlled
	// * mode to prevent flyaway crashes.
	// */
	// bool pre_flt_fail_innov_heading = false;
	// bool pre_flt_fail_innov_vel_horiz = false;

	// if (_estimator_status_sub.updated()) {

		// estimator_status_s estimator_status;

		// if (_estimator_status_sub.copy(&estimator_status)) {

		// 	pre_flt_fail_innov_heading = estimator_status.pre_flt_fail_innov_heading;
		// 	pre_flt_fail_innov_vel_horiz = estimator_status.pre_flt_fail_innov_vel_horiz;

		// 	if (run_quality_checks && _status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {

		// 		if (_status.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
		// 			_nav_test_failed = false;
		// 			_nav_test_passed = false;

		// 		} else {
		// 			if (!_nav_test_passed) {
		// 				// Both test ratios need to pass/fail together to change the nav test status
		// 				const bool innovation_pass = (estimator_status.vel_test_ratio < 1.f) && (estimator_status.pos_test_ratio < 1.f)
		// 							     && (estimator_status.vel_test_ratio > FLT_EPSILON) && (estimator_status.pos_test_ratio > FLT_EPSILON);

		// 				const bool innovation_fail = (estimator_status.vel_test_ratio >= 1.f) && (estimator_status.pos_test_ratio >= 1.f);

		// 				if (innovation_pass) {
		// 					_time_last_innov_pass = hrt_absolute_time();

		// 					// if nav status is unconfirmed, confirm yaw angle as passed after 30 seconds or achieving 5 m/s of speed
		// 					const bool sufficient_time = (_status.takeoff_time != 0) && (hrt_elapsed_time(&_status.takeoff_time) > 30_s);
		// 					const bool sufficient_speed = matrix::Vector2f(lpos.vx, lpos.vy).longerThan(5.f);

		// 					// Even if the test already failed, allow it to pass if it did not fail during the last 10 seconds
		// 					if (hrt_elapsed_time(&_time_last_innov_fail) > 10_s
		// 					    && (sufficient_time || sufficient_speed)) {
		// 						_nav_test_passed = true;
		// 						_nav_test_failed = false;
		// 					}

		// 				} else if (innovation_fail) {
		// 					_time_last_innov_fail = hrt_absolute_time();

		// 					if (!_nav_test_failed && hrt_elapsed_time(&_time_last_innov_pass) > 2_s) {
		// 						// if the innovation test has failed continuously, declare the nav as failed
		// 						_nav_test_failed = true;
		// 						mavlink_log_emergency(&_mavlink_log_pub, "Navigation failure! Land and recalibrate sensors\t");
		// 						events::send(events::ID("commander_navigation_failure"), events::Log::Emergency,
		// 							     "Navigation failure! Land and recalibrate the sensors");
		// 					}
		// 				}
		// 			}
		// 		}
		// 	}
		// }
	// }

	// run position and velocity accuracy checks
	// Check if quality checking of position accuracy and consistency is to be performed
	if (run_quality_checks) {
		float lpos_eph_threshold_adj = _param_com_pos_fs_eph.get();

		// relax local position eph threshold in operator controlled position mode
		if (_internal_state.main_state == commander_state_s::MAIN_STATE_POSCTL &&
		    ((_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL)
		     || (_status.nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL))) {

			// Set the allowable position uncertainty based on combination of flight and estimator state
			// When we are in a operator demanded position control mode and are solely reliant on optical flow,
			// do not check position error because it will gradually increase throughout flight and the operator will compensate for the drift
			if (_status_flags.position_reliant_on_optical_flow) {
				lpos_eph_threshold_adj = INFINITY;
			}
		}

		bool xy_valid = lpos.xy_valid && !_nav_test_failed;
		bool v_xy_valid = lpos.v_xy_valid && !_nav_test_failed;

		// if (!_armed.armed) {
			// if (pre_flt_fail_innov_heading || pre_flt_fail_innov_vel_horiz) {
			// 	xy_valid = false;
			// }

			// if (pre_flt_fail_innov_vel_horiz) {
			// 	v_xy_valid = false;
			// }
		// }

		const vehicle_global_position_s &gpos = _global_position_sub.get();

		_status_flags.global_position_valid =
			check_posvel_validity(xy_valid, gpos.eph, _param_com_pos_fs_eph.get(), gpos.timestamp,
					      _last_gpos_fail_time_us, _status_flags.global_position_valid);

		_status_flags.local_position_valid =
			check_posvel_validity(xy_valid, lpos.eph, lpos_eph_threshold_adj, lpos.timestamp,
					      _last_lpos_fail_time_us, _status_flags.local_position_valid);
		
		// std::cerr << "_status_flags.local_position_valid: " << _status_flags.local_position_valid << std::endl; // added by Peixuan Shu

		_status_flags.local_velocity_valid =
			check_posvel_validity(v_xy_valid, lpos.evh, _param_com_vel_fs_evh.get(), lpos.timestamp,
					      _last_lvel_fail_time_us, _status_flags.local_velocity_valid);
	}


	// altitude
	_status_flags.local_altitude_valid = lpos.z_valid
					     && (hrt_elapsed_time(&lpos.timestamp) < (_param_com_pos_fs_delay.get() * 1_s));


	// attitude
	vehicle_attitude_s attitude{};
	_vehicle_attitude_sub.copy(&attitude);
	const matrix::Quatf q{attitude.q};
	const bool no_element_larger_than_one = (fabsf(q(0)) <= 1.f)
						&& (fabsf(q(1)) <= 1.f)
						&& (fabsf(q(2)) <= 1.f)
						&& (fabsf(q(3)) <= 1.f);
	const bool norm_in_tolerance = (fabsf(1.f - q.norm()) <= 1e-6f);

	const bool attitude_valid = (hrt_elapsed_time(&attitude.timestamp) < 1_s)
				    && norm_in_tolerance && no_element_larger_than_one;

	if (_status_flags.attitude_valid && !attitude_valid) {
		// PX4_ERR("attitude estimate no longer valid");
		std::cerr << "[PX4 Commander.cpp] attitude estimate no longer valid" << std::endl; // modified by Peixuan Shu
	}

	_status_flags.attitude_valid = attitude_valid;


	// angular velocity
	vehicle_angular_velocity_s angular_velocity{};
	_vehicle_angular_velocity_sub.copy(&angular_velocity);
	const bool condition_angular_velocity_time_valid = (angular_velocity.timestamp != 0)
			&& (hrt_elapsed_time(&angular_velocity.timestamp) < 1_s);
	const bool condition_angular_velocity_finite = PX4_ISFINITE(angular_velocity.xyz[0])
			&& PX4_ISFINITE(angular_velocity.xyz[1]) && PX4_ISFINITE(angular_velocity.xyz[2]);
	const bool angular_velocity_valid = condition_angular_velocity_time_valid
					    && condition_angular_velocity_finite;

	if (_status_flags.angular_velocity_valid && !angular_velocity_valid) {
		const char err_str[] {"angular velocity no longer valid"};

		if (!condition_angular_velocity_time_valid) {
			// PX4_ERR("%s (timeout)", err_str);
			std::cerr << "[PX4 Commander.cpp] angular velocity no longer valid (timeout)" << std::endl; // modified by Peixuan Shu

		} else if (!condition_angular_velocity_finite) {
			// PX4_ERR("%s (non-finite values)", err_str);
			std::cerr << "[PX4 Commander.cpp] angular velocity no longer valid (non-finite values)" << std::endl; // modified by Peixuan Shu
		}
	}

	_status_flags.angular_velocity_valid = angular_velocity_valid;


	// gps
	const bool condition_gps_position_was_valid = _status_flags.gps_position_valid;

	if (_vehicle_gps_position_sub.updated()) {
		vehicle_gps_position_s vehicle_gps_position;

		if (_vehicle_gps_position_sub.copy(&vehicle_gps_position)) {

			bool time = (vehicle_gps_position.timestamp != 0) && (hrt_elapsed_time(&vehicle_gps_position.timestamp) < 1_s);

			bool fix = vehicle_gps_position.fix_type >= 2;
			bool eph = vehicle_gps_position.eph < _param_com_pos_fs_eph.get();
			bool epv = vehicle_gps_position.epv < _param_com_pos_fs_epv.get();
			bool evh = vehicle_gps_position.s_variance_m_s < _param_com_vel_fs_evh.get();

			_vehicle_gps_position_valid.set_state_and_update(time && fix && eph && epv && evh, hrt_absolute_time());
			_status_flags.gps_position_valid = _vehicle_gps_position_valid.get_state();

			_vehicle_gps_position_timestamp_last = vehicle_gps_position.timestamp;
		}

	} else {
		const hrt_abstime now_us = hrt_absolute_time();

		if (now_us > _vehicle_gps_position_timestamp_last + GPS_VALID_TIME) {
			_vehicle_gps_position_valid.set_state_and_update(false, now_us);
			_status_flags.gps_position_valid = false;
		}
	}

	if (condition_gps_position_was_valid && !_status_flags.gps_position_valid) {
		// PX4_DEBUG("GPS no longer valid");
		std::cerr << "[PX4 Commander.cpp] GPS no longer valid" << std::endl; // modified by Peixuan Shu
	}
}

/* Check if offboard control signal is lost */
void
Commander::offboard_control_update()
{
	bool offboard_available = false;

	if (_offboard_control_mode_sub.updated()) {
		const offboard_control_mode_s old = _offboard_control_mode_sub.get();

		if (_offboard_control_mode_sub.update()) {
			const offboard_control_mode_s &ocm = _offboard_control_mode_sub.get();

			if (old.position != ocm.position ||
			    old.velocity != ocm.velocity ||
			    old.acceleration != ocm.acceleration ||
			    old.attitude != ocm.attitude ||
			    old.body_rate != ocm.body_rate ||
			    old.actuator != ocm.actuator) {

				_status_changed = true;
			}

			if (ocm.position || ocm.velocity || ocm.acceleration || ocm.attitude || ocm.body_rate || ocm.actuator) {
				offboard_available = true;
			}
		}
	}

	if (_offboard_control_mode_sub.get().position && !_status_flags.local_position_valid) {
		offboard_available = false;

	} else if (_offboard_control_mode_sub.get().velocity && !_status_flags.local_velocity_valid) {
		offboard_available = false;

	} else if (_offboard_control_mode_sub.get().acceleration && !_status_flags.local_velocity_valid) {
		// OFFBOARD acceleration handled by position controller
		offboard_available = false;
	}

	_offboard_available.set_state_and_update(offboard_available, hrt_absolute_time());

	const bool offboard_lost = !_offboard_available.get_state();

	if (_status_flags.offboard_control_signal_lost != offboard_lost) {
		_status_flags.offboard_control_signal_lost = offboard_lost;
		_status_changed = true;
	}
}
