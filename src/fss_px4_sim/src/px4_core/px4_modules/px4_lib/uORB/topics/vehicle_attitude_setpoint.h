/**
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief simulated uorb topic struct definition
 * 
 * 
 * @version 1.0
 * @date 2023-11-28
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */
 
#pragma once

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

struct vehicle_attitude_setpoint_s {
	uint64_t timestamp;
	float roll_body;
	float pitch_body;
	float yaw_body;
	float yaw_sp_move_rate;
	float q_d[4];
	float thrust_body[3];
	bool roll_reset_integral;
	bool pitch_reset_integral;
	bool yaw_reset_integral;
	bool fw_control_yaw;
	uint8_t apply_flaps;
	uint8_t _padding0[7]; // required for logger

	static constexpr uint8_t FLAPS_OFF = 0;
	static constexpr uint8_t FLAPS_LAND = 1;
	static constexpr uint8_t FLAPS_TAKEOFF = 2;

};

