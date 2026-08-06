/**
 * @file px4_uorb_lists.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Store the PX4 global uorb messages to simulate the real PX4 uORB messages. 
 * 
 * Note: This program relies on px4_lib/uORB/
 * 
 * @version 1.0
 * @date 2023-12-14
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */

#include "uORB_sim.hpp"

namespace uORB_sim { 

// @TODO: Initialize if no publisher exists?

// actuator_armed_s actuator_armed{};
// autotune_attitude_control_status_s autotune_attitude_control_status{};
// battery_status_s battery_status{};
// commander_state_s commander_state{};
// cpuload_s cpuload{};
// home_position_s home_position{};
// manual_control_setpoint_s manual_control_setpoint{};
// offboard_control_mode_s offboard_control_mode{};
// takeoff_status_s takeoff_status{};
// vehicle_air_data_s vehicle_air_data{};
// vehicle_angular_velocity_s vehicle_angular_velocity{};
// vehicle_attitude_s vehicle_attitude{};
// vehicle_command_s vehicle_command{};
// vehicle_command_ack_s vehicle_command_ack{};
// vehicle_attitude_setpoint_s vehicle_attitude_setpoint{};
// vehicle_constraints_s vehicle_constraints{};
// vehicle_control_mode_s vehicle_control_mode{};
// vehicle_global_position_s vehicle_global_position{};
// vehicle_gps_position_s vehicle_gps_position{};
// vehicle_land_detected_s vehicle_land_detected{};
// vehicle_local_position_s vehicle_local_position{};
// vehicle_local_position_setpoint_s vehicle_local_position_setpoint{};
// vehicle_local_position_setpoint_s trajectory_setpoint{};
// vehicle_odometry_s vehicle_odometry{};
// vehicle_rates_setpoint_s vehicle_rates_setpoint{};
// vehicle_status_s vehicle_status{};
// vehicle_status_flags_s vehicle_status_flags{};

/* Store the PX4 uORB messages for arbitrary number of UAVs  (define) */
// (These messages are global/extern!)
// initialize with at least one element for UAV 1
std::vector<actuator_armed_s> actuator_armed {actuator_armed_s{.timestamp = 0}};
std::vector<autotune_attitude_control_status_s> autotune_attitude_control_status {autotune_attitude_control_status_s{.timestamp = 0}};
std::vector<battery_status_s> battery_status {battery_status_s{.timestamp = 0}};
std::vector<commander_state_s> commander_state {commander_state_s{.timestamp = 0}};
std::vector<cpuload_s> cpuload {cpuload_s{.timestamp = 0}};
std::vector<home_position_s> home_position {home_position_s{.timestamp = 0}};
std::vector<manual_control_setpoint_s> manual_control_setpoint {manual_control_setpoint_s{.timestamp = 0}};
std::vector<offboard_control_mode_s> offboard_control_mode {offboard_control_mode_s{.timestamp = 0}};
std::vector<takeoff_status_s> takeoff_status {takeoff_status_s{.timestamp = 0}};
std::vector<vehicle_air_data_s> vehicle_air_data {vehicle_air_data_s{.timestamp = 0}};
std::vector<vehicle_angular_velocity_s> vehicle_angular_velocity {vehicle_angular_velocity_s{.timestamp = 0}};
std::vector<vehicle_attitude_s> vehicle_attitude {vehicle_attitude_s{.timestamp = 0}};
std::vector<vehicle_attitude_setpoint_s> vehicle_attitude_setpoint {vehicle_attitude_setpoint_s{.timestamp = 0}};
std::vector<vehicle_command_s> vehicle_command {vehicle_command_s{.timestamp = 0}};
std::vector<vehicle_command_ack_s> vehicle_command_ack {vehicle_command_ack_s{.timestamp = 0}};
std::vector<vehicle_constraints_s> vehicle_constraints {vehicle_constraints_s{.timestamp = 0}};
std::vector<vehicle_control_mode_s> vehicle_control_mode {vehicle_control_mode_s{.timestamp = 0}};
std::vector<vehicle_global_position_s> vehicle_global_position {vehicle_global_position_s{.timestamp = 0}};
std::vector<vehicle_gps_position_s> vehicle_gps_position {vehicle_gps_position_s{.timestamp = 0}};
std::vector<vehicle_land_detected_s> vehicle_land_detected {vehicle_land_detected_s{.timestamp = 0}};
std::vector<vehicle_local_position_s> vehicle_local_position {vehicle_local_position_s{.timestamp = 0}};
std::vector<vehicle_local_position_setpoint_s> vehicle_local_position_setpoint {vehicle_local_position_setpoint_s{.timestamp = 0}}; // real target setpoints
std::vector<vehicle_local_position_setpoint_s> trajectory_setpoint {vehicle_local_position_setpoint_s{.timestamp = 0}}; // mavlink setpoints
std::vector<vehicle_odometry_s> vehicle_odometry {vehicle_odometry_s{.timestamp = 0}};
std::vector<vehicle_rates_setpoint_s> vehicle_rates_setpoint {vehicle_rates_setpoint_s{.timestamp = 0}};
std::vector<vehicle_status_s> vehicle_status {vehicle_status_s{.timestamp = 0}};
std::vector<vehicle_status_flags_s> vehicle_status_flags {vehicle_status_flags_s{.timestamp = 0}};

/* allocate global storage for messages of agent i */
void allocate_uorb_message_storage(int expected_agent_num)
{
    while(actuator_armed.size() < expected_agent_num)
    {
        actuator_armed.emplace_back(actuator_armed_s{.timestamp = 0});
    }
    while(battery_status.size() < expected_agent_num)
    {
        battery_status.emplace_back(battery_status_s{.timestamp = 0});
    }
    while(commander_state.size() < expected_agent_num)
    {
        commander_state.emplace_back(commander_state_s{.timestamp = 0});
    }
    while(cpuload.size() < expected_agent_num)
    {
        cpuload.emplace_back(cpuload_s{.timestamp = 0});
    }
    while(home_position.size() < expected_agent_num)
    {
        home_position.emplace_back(home_position_s{.timestamp = 0});
    }
    while(manual_control_setpoint.size() < expected_agent_num)
    {
        manual_control_setpoint.emplace_back(manual_control_setpoint_s{.timestamp = 0});
    }
    while(offboard_control_mode.size() < expected_agent_num)
    {
        offboard_control_mode.emplace_back(offboard_control_mode_s{.timestamp = 0});
    }
    while(takeoff_status.size() < expected_agent_num)
    {
        takeoff_status.emplace_back(takeoff_status_s{.timestamp = 0});
    }
    while(vehicle_air_data.size() < expected_agent_num)
    {
        vehicle_air_data.emplace_back(vehicle_air_data_s{.timestamp = 0});
    }
    while(vehicle_angular_velocity.size() < expected_agent_num)
    {
        vehicle_angular_velocity.emplace_back(vehicle_angular_velocity_s{.timestamp = 0});
    }
    while(vehicle_attitude.size() < expected_agent_num)
    {
        vehicle_attitude.emplace_back(vehicle_attitude_s{.timestamp = 0});
    }
    while(vehicle_attitude_setpoint.size() < expected_agent_num)
    {
        vehicle_attitude_setpoint.emplace_back(vehicle_attitude_setpoint_s{.timestamp = 0});
    }
    while(vehicle_command.size() < expected_agent_num)
    {
        vehicle_command.emplace_back(vehicle_command_s{.timestamp = 0});
    }
    while(vehicle_command_ack.size() < expected_agent_num)
    {
        vehicle_command_ack.emplace_back(vehicle_command_ack_s{.timestamp = 0});
    }
    while(vehicle_constraints.size() < expected_agent_num)
    {
        vehicle_constraints.emplace_back(vehicle_constraints_s{.timestamp = 0});
    }
    while(autotune_attitude_control_status.size() < expected_agent_num)
    {
        autotune_attitude_control_status.emplace_back(autotune_attitude_control_status_s{.timestamp = 0});
    }
    while(vehicle_control_mode.size() < expected_agent_num)
    {
        vehicle_control_mode.emplace_back(vehicle_control_mode_s{.timestamp = 0});
    }
    while(vehicle_global_position.size() < expected_agent_num)
    {
        vehicle_global_position.emplace_back(vehicle_global_position_s{.timestamp = 0});
    }
    while(vehicle_gps_position.size() < expected_agent_num)
    {
        vehicle_gps_position.emplace_back(vehicle_gps_position_s{.timestamp = 0});
    }
    while(vehicle_land_detected.size() < expected_agent_num)
    {
        vehicle_land_detected.emplace_back(vehicle_land_detected_s{.timestamp = 0});
    }
    while(vehicle_local_position.size() < expected_agent_num)
    {
        vehicle_local_position.emplace_back(vehicle_local_position_s{.timestamp = 0});
    }
    while(vehicle_local_position_setpoint.size() < expected_agent_num)
    {
        vehicle_local_position_setpoint.emplace_back(vehicle_local_position_setpoint_s{.timestamp = 0});
    }
    while(trajectory_setpoint.size() < expected_agent_num)
    {
        trajectory_setpoint.emplace_back(vehicle_local_position_setpoint_s{.timestamp = 0});
    }
    while(vehicle_odometry.size() < expected_agent_num)
    {
        vehicle_odometry.emplace_back(vehicle_odometry_s{.timestamp = 0});
    }
    while(vehicle_rates_setpoint.size() < expected_agent_num)
    {
        vehicle_rates_setpoint.emplace_back(vehicle_rates_setpoint_s{.timestamp = 0});
    }
    while(vehicle_status.size() < expected_agent_num)
    {
        vehicle_status.emplace_back(vehicle_status_s{.timestamp = 0});
    }
    while(vehicle_status_flags.size() < expected_agent_num)
    {
        vehicle_status_flags.emplace_back(vehicle_status_flags_s{.timestamp = 0});
    }
}

}