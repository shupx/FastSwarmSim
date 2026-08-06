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

struct vehicle_local_position_setpoint_s {

	uint64_t timestamp;
	float x;
	float y;
	float z;
	float yaw;
	float yawspeed;
	float vx;
	float vy;
	float vz;
	float acceleration[3];
	float jerk[3];
	float thrust[3];
	uint8_t _padding0[4]; // required for logger

};
