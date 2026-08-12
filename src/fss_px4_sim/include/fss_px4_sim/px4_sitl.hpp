/**
 * @file px4_sitl.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief PX4 main function. Communicate with mavros and dynamics.
 * Mavros <---> PX4_SITL <---> Quadrotor Dynamics
 * 
 * Note: This program relies on mavlink, px4_modules, quadrotor_dynamics
 * 
 * @version 1.0
 * @date 2023-12-11
 * Modified by Peixuan Shu (2026-08-09): own an instance-local PX4 runtime
 * context for multiple flight stacks in one process.
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */


#ifndef __PX4_SITL_H__
#define __PX4_SITL_H__

#include <rclcpp/rclcpp.hpp>
#include <iostream> // for std::cout, std::endl
#include <memory>  // for std::shared_ptr

#include <mavlink/v2.0/common/mavlink.h> // from ros-noetic-mavlink or include/ folder

#include <parameters/px4_parameters.hpp> // store all extern(global) px4 parameters
#include <matrix/matrix/math.hpp> // for px4 geometry utils
#include <geo/geo.h> // for px4 geo utils (gps to ENU)
#include <hysteresis/hysteresis.h> // for px4 hysteresis utils
#include <mavros/frame_tf.h> // for mavros::ftf frame conversion
#include <uORB/uORB_sim.hpp> // simulate uORB publication and subscription. Store the extern(global) simulated uORB messages

#include "fss_px4_sim/px4_instance_context.hpp"
#include "px4_modules/mavlink/mavlink_main.hpp"
#include "px4_modules/commander/Commander.hpp"
#include "px4_modules/mc_pos_control/MulticopterPositionControl.hpp"
#include "px4_modules/mc_att_control/mc_att_control.hpp"
#include "px4_modules/px4_lib/drivers/drv_hrt.h"  // time utils, store the extern(global) simulated time

#include "fss_px4_sim/quadrotor_dynamics.hpp"


namespace MavrosQuadSimulator
{

/**
 * \brief PX4 flight stack simulation. Communicate with mavros and dynamics.
 * Mavros <---> PX4_SITL <---> Quadrotor Dynamics
 */
class PX4SITL
{
public:
    // Modified by Peixuan Shu: PX4SITL owns the complete runtime context.
    PX4SITL(rclcpp::Node &node, const std::shared_ptr<Dynamics> &dynamics);

    /* Load px4 parameters from ROS parameter space to override the default values from <parameters/px4_parameters.hpp>*/    
    void load_px4_params_from_ros_params();

    /**
     * \brief Run px4 modules for one step. Run pos and att controller to calculate control output
     * @param time_us The microseconds (us) now.
     */
    void Run(const uint64_t &time_us);

    bool uses_direct_ros() const
    {
        return mavlink_ && mavlink_->transport() == MAVLINK::Transport::DirectRos;
    }
    uint8_t mavlink_system_id() const { return mavlink_ ? mavlink_->system_id() : 1; }
    uint8_t mavlink_component_id() const { return mavlink_ ? mavlink_->component_id() : 1; }
    void receive_mavlink_message(const mavlink_message_t &message)
    {
        if (mavlink_) mavlink_->receive_message(message);
    }
    void set_mavlink_send_callback(MAVLINK::SendCallback callback)
    {
        if (mavlink_) mavlink_->set_send_callback(std::move(callback));
    }

    /* Get px4 params from px4::parameters */
    template <px4::params p>
    void get_px4_param(float& output);

    /* Get px4 params from px4::parameters */
    template <px4::params p>
    void get_px4_param(int32_t& output);

    /* Get px4 params from px4::parameters */
    template <px4::params p>
    void get_px4_param(bool& output);

private:
    // Added by Peixuan Shu: declare context first so it outlives all modules.
    Px4InstanceContext context_;
    rclcpp::Node &node_;

    enum class position_mode : uint32_t {
        MOCAP,
        GPS
    };
    position_mode local_pos_source_;

    float init_x_East_metre_;
    float init_y_North_metre_;
    float init_z_Up_metre_;

    double world_origin_lat_; // Reference point latitude (degrees). The world (0,0,0) point
    double world_origin_lon_; // Reference point longitude (degrees). The world (0,0,0) point
    float world_origin_asml_alt_; // Reference altitude AMSL (metres). The world (0,0,0) point

    double init_lat_; // latitude at initial point
    double init_lon_; // longitude at initial point

    bool pos_inited_; // if the LoadInitPos() is called

    /* init landed detector hysteresis */
    systemlib::Hysteresis _landed_hysteresis{true};
    systemlib::Hysteresis _maybe_landed_hysteresis{true};
    systemlib::Hysteresis _ground_contact_hysteresis{true};
    systemlib::Hysteresis _freefall_hysteresis{false};
    
    uint64_t init_ref_timestamp_ = 0; // 0 means not initialized
    uint64_t last_ref_timestamp_ = 0; // 0 means not initialized

    double last_ref_lat_ = 0; // 0 means not initialized
    double last_ref_lon_ = 0; // 0 means not initialized
    float last_ref_alt_ = 0; // 0 means not initialized

	// publications with topic
    // Modified by Peixuan Shu: bind PX4SITL's own topics directly to context_.
	uORB_sim::Publication<vehicle_attitude_s> _attitude_pub{context_.uorb_domain, ORB_ID(vehicle_attitude)};
	uORB_sim::Publication<vehicle_local_position_s> _local_position_pub{context_.uorb_domain, ORB_ID(vehicle_local_position)};
    uORB_sim::Publication<vehicle_angular_velocity_s> _vehicle_angular_velocity_pub{context_.uorb_domain, ORB_ID(vehicle_angular_velocity)};
    uORB_sim::Publication<battery_status_s> _battery_status_pub{context_.uorb_domain, ORB_ID(battery_status)};
	uORB_sim::Publication<vehicle_global_position_s> _global_position_pub{context_.uorb_domain, ORB_ID(vehicle_global_position)};
	uORB_sim::Publication<vehicle_odometry_s> _odometry_pub{context_.uorb_domain, ORB_ID(vehicle_odometry)};
    uORB_sim::Publication<vehicle_land_detected_s> _vehicle_land_detected_pub{context_.uorb_domain, ORB_ID(vehicle_land_detected)};

    uORB_sim::Subscription<vehicle_command_s> _vehicle_command_sub{context_.uorb_domain, ORB_ID(vehicle_command)};
    uORB_sim::Subscription<vehicle_rates_setpoint_s> _vehicle_rates_setpoint_sub{context_.uorb_domain, ORB_ID(vehicle_rates_setpoint)};
    uORB_sim::Subscription<vehicle_local_position_s> _vehicle_local_position_sub{context_.uorb_domain, ORB_ID(vehicle_local_position)};
    uORB_sim::Subscription<vehicle_control_mode_s> _vehicle_control_mode_sub{context_.uorb_domain, ORB_ID(vehicle_control_mode)};
    

    std::shared_ptr<MAVLINK> mavlink_;
    std::shared_ptr<Commander> commander_;
    std::shared_ptr<MulticopterPositionControl> mc_pos_control_; 
    std::shared_ptr<MulticopterAttitudeControl> mc_att_control_; 

    std::shared_ptr<Dynamics> uav_dynamics_;

    /* load initial position and local_pos_source */
    void update_init_pos_from_dynamics();

    /**
     * \brief Update px4 estimator uorb states (pos/vel/acc/att, etc.) from UAV dynamical model 
     * @param time_us The microseconds (us) now.
     */
    void UpdateDroneStates(const uint64_t &time_us);

    /* Land detector module. Update vehicle_land_detected uORB message*/
    void DectectLand(const uint64_t &time_us);

    /* Stream MAVLink messages at a given frequency. */
    void StreamMavlink(const uint64_t &time_us);

    /* Send control input calculated by the controller to the quadrotor dynamics */
    void SendControlInput();

};



}



#endif
