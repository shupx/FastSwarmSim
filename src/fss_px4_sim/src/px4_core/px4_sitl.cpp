/**
 * @file px4_sitl.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief PX4 main function. Communicate with mavros and dynamics.
 * Mavros <---> PX4_SITL <---> Quadrotor Dynamics
 * 
 * Note: This program relies on mavlink, px4_modules
 * 
 * @version 1.0
 * @date 2023-12-11
 * Modified by Peixuan Shu (2026-08-09): bind PX4 modules to instance-local
 * uORB, parameter, and simulation-clock contexts.
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */


#include "fss_px4_sim/px4_sitl.hpp"

namespace MavrosQuadSimulator
{

PX4SITL::PX4SITL(rclcpp::Node &node,
    const std::shared_ptr<Dynamics> &dynamics)
    : node_(node), uav_dynamics_(dynamics), pos_inited_(false)
{
    // Added by Peixuan Shu: modules bind to the context owned by this PX4SITL.
    Px4InstanceContext::Scope context_scope(context_);
    /* Load px4 parameters from ROS parameter space to override the default values from <parameters/px4_parameters.hpp>*/
    load_px4_params_from_ros_params(); // Before loading px4 modules!

    /* update_init_pos_from_dynamics */
    update_init_pos_from_dynamics();

    /* Load px4 modules */
	int mavlink_udp_local_port = 0;
	int mavlink_udp_remote_port = 24540;
	int mavlink_system_id = 1; // added or modified by Peixuan Shu
	int mavlink_component_id = 1; // added or modified by Peixuan Shu
	node_.get_parameter_or("mavlink_udp_local_port", mavlink_udp_local_port, mavlink_udp_local_port);
	node_.get_parameter_or("mavlink_udp_remote_port", mavlink_udp_remote_port, mavlink_udp_remote_port);
	node_.get_parameter_or("MAV_SYS_ID", mavlink_system_id, mavlink_system_id); // added or modified by Peixuan Shu
	node_.get_parameter_or("MAV_COMP_ID", mavlink_component_id, mavlink_component_id); // added or modified by Peixuan Shu
	if (mavlink_system_id < 1 || mavlink_system_id > 255 || mavlink_component_id < 1 || mavlink_component_id > 255) {
		throw std::invalid_argument("MAV_SYS_ID and MAV_COMP_ID must be in [1, 255]");
	}

	mavlink_ = std::make_shared<MAVLINK>(context_, mavlink_udp_local_port, mavlink_udp_remote_port,
		static_cast<uint8_t>(mavlink_system_id), static_cast<uint8_t>(mavlink_component_id));
    mavlink_->start();
    commander_ = std::make_shared<Commander>();
    mc_pos_control_ = std::make_shared<MulticopterPositionControl>(false);
    mc_att_control_ = std::make_shared<MulticopterAttitudeControl>(false);

    /* Init px4 modules */
    mc_pos_control_->init();
    mc_att_control_->init();
}

void PX4SITL::load_px4_params_from_ros_params()
{
    /* Modified by Peixuan Shu: load ROS overrides into this instance's ParameterStore. */
    for (int i=0; i<sizeof(px4::parameters)/sizeof(px4::parameters[0]); ++i)
    {
        switch (px4::parameters_type[i])
        {
            case PARAM_TYPE_INT32:
            {
                int configured = px4::parameters[i].val.i;
                const int default_value = configured;
                node_.get_parameter_or(px4::parameters[i].name, configured, default_value);
                context_.parameter_store.set(static_cast<param_t>(i), &configured);
                if (configured != default_value)
                {
                    std::cout << "[PX4SITL] Reset " << px4::parameters[i].name << " from " << default_value << " to " << configured << std::endl;
                }
                break;
            }
            case PARAM_TYPE_FLOAT:
            {
                float configured = px4::parameters[i].val.f;
                const float default_value = configured;
                node_.get_parameter_or(px4::parameters[i].name, configured, default_value);
                context_.parameter_store.set(static_cast<param_t>(i), &configured);
                if (configured != default_value)
                {
                    std::cout << "[PX4SITL] Reset " << px4::parameters[i].name << " from " << default_value << " to " << configured << std::endl;
                }
                break; 
            }   
            default:
            {
                throw std::invalid_argument("[PX4SITL::load_px4_param_from_ros_params] param type is unknown.");
                break;  
            }      
        }
    }
}

void PX4SITL::update_init_pos_from_dynamics()
{
    /* load initial position and local_pos_source */
    int source;
    node_.get_parameter_or("local_pos_source", source, 0);
    local_pos_source_ = (enum position_mode)source;

    node_.get_parameter_or("world_origin_latitude_deg", world_origin_lat_, 39.978861);
    node_.get_parameter_or("world_origin_longitude_deg", world_origin_lon_, 116.339803);
    node_.get_parameter_or("world_origin_AMSL_alt_metre", world_origin_asml_alt_, 53.0F);

    Dynamics::State init_state = uav_dynamics_->getState();
    init_x_East_metre_ = init_state.pos[0];
    init_y_North_metre_ = init_state.pos[1];
    init_z_Up_metre_ = init_state.pos[2];

    /* calculate the lat/lon of the initial position */
    MapProjection global_local_proj_ref{world_origin_lat_, world_origin_lon_, 0}; // Init MapProjection from PX4 geo.h
    global_local_proj_ref.reproject(init_y_North_metre_, init_x_East_metre_, init_lat_, init_lon_);

    pos_inited_ = true;
}

void PX4SITL::Run(const uint64_t &time_us)
{
    if (!pos_inited_)
    {
        RCLCPP_ERROR(node_.get_logger(), "PX4SITL position is not initialized");
    }
    else
    {
        /* Modified by Peixuan Shu: advance this PX4 instance's simulation clock. */
        context_.clock.set(time_us);
        Px4InstanceContext::Scope context_scope(context_);

        /* Update px4 estimator uorb states (pos/vel/acc/att, etc.) from UAV dynamical model */
        UpdateDroneStates(time_us);

        /* Update vehicle_land_detected uORB message*/
        DectectLand(time_us);

        /* Run commander module to handle vehicle_command and switch/publish vehicle mode/status uorb messages */
        commander_->run();

        /* Run pos and att controller to calculate control output */
        mc_pos_control_->Run(); // calling period should between [0.002f, 0.04f] 25Hz-500Hz
        mc_att_control_->Run(); // calling should between [0.0002f, 0.02f] 50Hz-5000Hz
        //@TODO re-takeoff after land and armed

        /* Stream mavlink messages */
        StreamMavlink(time_us);

        /* Send control input calculated by the controller to the quadrotor dynamics */
        SendControlInput();
    }


}

void PX4SITL::StreamMavlink(const uint64_t &time_us)
{
    mavlink_->Stream(time_us);
}

void PX4SITL::SendControlInput()
{
    /* get rate/throttle calculated by the px4 controllers */
    vehicle_rates_setpoint_s vehicle_rates_setpoint_msg {};
    _vehicle_rates_setpoint_sub.update(&vehicle_rates_setpoint_msg);

    Eigen::Vector3d body_rate_frd {vehicle_rates_setpoint_msg.roll, vehicle_rates_setpoint_msg.pitch, vehicle_rates_setpoint_msg.yaw}; // angular rates in FRD body frame
    float throttle =  matrix::Vector3f(vehicle_rates_setpoint_msg.thrust_body).norm(); // throttle [0,1]

    /* throttle - thrust model */
    /* <1> thrust_force = K1 * Voltage^K2 * ( K3*throttle^2 + (1-K3)*throttle ) */
    /* <2> thrust_force = K1 * ( K3*throttle^2 + (1-K3)*throttle ) */  // here we use <2>
    float hover_throttle, K1, K2, K3;
    get_px4_param<px4::params::MPC_THR_HOVER>(hover_throttle); // set by px4_parameters.cpp or ROS params
    get_px4_param<px4::params::THR_MDL_FAC>(K3); // set by px4_parameters.cpp or ROS params 
    /** Refer to https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter.html#thrust-curve
     *  Typical values of THR_MDL_FAC are between 0.3 and 0.5.
     */
    double mass = uav_dynamics_->getMass();
    double g = uav_dynamics_->getGravityAcc();
    K1 = mass*g / ( K3*hover_throttle*hover_throttle + (1-K3)*hover_throttle );

    Dynamics::Input input;
    input.omega = mavros::ftf::transform_frame_aircraft_baselink(body_rate_frd); //Transform data expressed in Aircraft(FRD) frame to Baselink(FLU) frame.
    input.thrust = K1 * ( K3*throttle*throttle + (1-K3)*throttle );

    // std::cout << "throttle: " << throttle << std::endl;
    // std::cout << "input.thrust(N) " << input.thrust << std::endl;
    
    uav_dynamics_->setInput(input);
}

void PX4SITL::DectectLand(const uint64_t &time_us)
{
    /* Land detector module. Update vehicle_land_detected uORB message*/
    // Refer to https://github.com/PX4/PX4-Autopilot/blob/v1.13.3/src/modules/land_detector/LandDetector.cpp#L141

    // /* init landed detector hysteresis */
    // static systemlib::Hysteresis _landed_hysteresis{true};
    // static systemlib::Hysteresis _maybe_landed_hysteresis{true};
    // static systemlib::Hysteresis _ground_contact_hysteresis{true};
    // static systemlib::Hysteresis _freefall_hysteresis{false};

    float trigger_landed_time;  // confirm landed after <trigger_landed_time> seconds
    get_px4_param<px4::params::LNDMC_TRIG_TIME>(trigger_landed_time);
    _landed_hysteresis.set_hysteresis_time_from(false, trigger_landed_time * 1_s);
    _maybe_landed_hysteresis.set_hysteresis_time_from(false, trigger_landed_time * 1_s);
    _ground_contact_hysteresis.set_hysteresis_time_from(false, trigger_landed_time * 1_s);
    _freefall_hysteresis.set_hysteresis_time_from(false, 300_ms);

    /* subscribe to local height and arm state */
    vehicle_local_position_s vehicle_local_position_msg{};
    vehicle_control_mode_s vehicle_control_mode_msg{};
    if(_vehicle_local_position_sub.update(&vehicle_local_position_msg)
        &&_vehicle_control_mode_sub.update(&vehicle_control_mode_msg)) // has new state
    {
        float height  = -vehicle_local_position_msg.z; // NED frame
        bool armed = vehicle_control_mode_msg.flag_armed;
        matrix::Vector3f _acceleration {vehicle_local_position_msg.ax, 
            vehicle_local_position_msg.ay, vehicle_local_position_msg.az - CONSTANTS_ONE_G}; // Specific force (non-gravitational force acc)

        _landed_hysteresis.set_state_and_update(height<=0.02 || !armed, time_us);
        _maybe_landed_hysteresis.set_state_and_update(height<=0.02 || !armed, time_us);
        _ground_contact_hysteresis.set_state_and_update(height<=0.02, time_us);
        _freefall_hysteresis.set_state_and_update(_acceleration.norm() < 2.f, time_us); // norm of specific force. Should be close to 9.8 m/s^2 when landed.

        vehicle_land_detected_s vehicle_land_detected_msg{};
        vehicle_land_detected_msg.timestamp = time_us;
        vehicle_land_detected_msg.landed = _landed_hysteresis.get_state();
        vehicle_land_detected_msg.maybe_landed = _ground_contact_hysteresis.get_state();
        vehicle_land_detected_msg.ground_contact = _ground_contact_hysteresis.get_state();
        vehicle_land_detected_msg.freefall = _freefall_hysteresis.get_state();
        _vehicle_land_detected_pub.publish(vehicle_land_detected_msg);
    }
}


void PX4SITL::UpdateDroneStates(const uint64_t &time_us)
{
    /* Read true values from uav dynamic models (world ENU, body FLU) */
    Dynamics::State state = uav_dynamics_->getState(); // in world ENU frame
    Eigen::Vector3d acc = uav_dynamics_->getAcc(); // in world ENU frame
    Eigen::Quaterniond q = uav_dynamics_->getQuat(); // body FLU -> world ENU
    Eigen::Vector3d omega = uav_dynamics_->getAngVel(); // in body FLU frame

    /* Transform from Dynamic frame (world ENU, body FLU/baselink) to PX4 frame (world NED, body FRD/aircraft) using mavros::ftf */
    Eigen::Vector3d pos_ned = mavros::ftf::transform_frame_enu_ned(state.pos); //Transform data expressed in ENU to NED frame.
    Eigen::Vector3d vel_ned = mavros::ftf::transform_frame_enu_ned(state.vel); //Transform data expressed in ENU to NED frame.
    Eigen::Vector3d acc_ned = mavros::ftf::transform_frame_enu_ned(acc); //Transform data expressed in ENU to NED frame.
    Eigen::Vector3d omega_frd = mavros::ftf::transform_frame_baselink_aircraft(omega); //Transform data expressed in Baselink(FLU) frame to Aircraft(FRD in mavros melodic/noetic) frame.
    Eigen::Quaterniond q_ned_frd = mavros::ftf::transform_orientation_enu_ned(
                mavros::ftf::transform_orientation_baselink_aircraft(q));


    // Refer to https://github.com/PX4/PX4-Autopilot/blob/v1.13.3/msg/vehicle_attitude.msg
    vehicle_attitude_s vehicle_attitude_msg{};
    vehicle_attitude_msg.timestamp = time_us;
    vehicle_attitude_msg.timestamp_sample = time_us;
    vehicle_attitude_msg.q[0] = q_ned_frd.w();
    vehicle_attitude_msg.q[1] = q_ned_frd.x();
    vehicle_attitude_msg.q[2] = q_ned_frd.y();
    vehicle_attitude_msg.q[3] = q_ned_frd.z();
    _attitude_pub.publish(vehicle_attitude_msg);


    // Refer to https://github.com/PX4/PX4-Autopilot/blob/v1.13.3/msg/vehicle_global_position.msg
    vehicle_global_position_s vehicle_global_position_msg{};
    vehicle_global_position_msg.timestamp = time_us;
    vehicle_global_position_msg.timestamp_sample = time_us;
    /* Calculate lat/lon according to x(North), y(East) and origin lat/lon */
    double lat, lon;
    MapProjection global_local_proj_ref{world_origin_lat_, world_origin_lon_, time_us}; // MapProjection from PX4 geo.h
    global_local_proj_ref.reproject(pos_ned[0], pos_ned[1], lat, lon);
    vehicle_global_position_msg.lat = lat;
    vehicle_global_position_msg.lon = lon;
    vehicle_global_position_msg.alt = world_origin_asml_alt_ - pos_ned[2]; // Altitude AMSL, (meters) .Note that positive local.z is Down, but positive AMSL alt is UP.
    vehicle_global_position_msg.alt_ellipsoid = vehicle_global_position_msg.alt; // Altitude above ellipsoid, (meters) 
    //@TODO conversion from Geoid(MSL) to Ellipsoid(WGS84) altitude based on GeographicLib::Geoid of <GeographicLib/Geoid.hpp> as mavros does. But in fact alt_ellipsoid is not used by mavros at all.
    /***********************************************************************/
    // #include <GeographicLib/Geoid.hpp>
    // std::shared_ptr<GeographicLib::Geoid> egm96_5; // This class loads egm96_5 dataset to RAM, it is about 24 MiB.
    // egm96_5 = std::make_shared<GeographicLib::Geoid>("egm96-5", "", true, true); // Using smallest dataset with 5' grid, // From default location, // Use cubic interpolation, Thread safe
    // vehicle_global_position_msg.alt_ellipsoid = vehicle_global_position_msg.alt + GeographicLib::Geoid::GEOIDTOELLIPSOID * (*egm96_5)(lat, lon); // AMSL TO WGS84 altitude
    /************************************************************************/
    _global_position_pub.publish(vehicle_global_position_msg);


    // Refer to https://github.com/PX4/PX4-Autopilot/blob/v1.13.3/msg/vehicle_local_position.msg
    vehicle_local_position_s vehicle_local_position_msg{};
    vehicle_local_position_msg.timestamp = time_us;
    vehicle_local_position_msg.timestamp_sample = time_us;
    vehicle_local_position_msg.xy_global = true; // true if position (x, y) has a valid global reference (ref_lat, ref_lon)
    vehicle_local_position_msg.z_global = true; // true if z has a valid global reference (ref_alt)
    vehicle_local_position_msg.xy_valid = true; // true if x and y are valid
    vehicle_local_position_msg.z_valid = true; // true if z are valid
    vehicle_local_position_msg.v_xy_valid = true; // true if vx and vy are valid
    vehicle_local_position_msg.v_z_valid = true; // true if vz is valid
    switch (local_pos_source_)
    {
        case position_mode::MOCAP: { // local position origin at world origin
            // static float init_ref_timestamp = time_us;
            if (init_ref_timestamp_ == 0) { init_ref_timestamp_ = time_us;} // initialize
            vehicle_local_position_msg.ref_timestamp = init_ref_timestamp_; // Time when reference position was set since system start, (microseconds)
            vehicle_local_position_msg.ref_lat = world_origin_lat_; // (degrees) lat at local position(0,0)
            vehicle_local_position_msg.ref_lon = world_origin_lon_; // (degrees) lon at local position(0,0)
            vehicle_local_position_msg.ref_alt = world_origin_asml_alt_; // (metres) AMSL(Geoid) altitude at local position(0,0,0)
            vehicle_local_position_msg.x = pos_ned[0];
            vehicle_local_position_msg.y = pos_ned[1];
            vehicle_local_position_msg.z = pos_ned[2];
            break;
        }
        case position_mode::GPS: { // local position origin at init position or set by mavros/global_position/set_gp_origin
            // static float last_ref_timestamp = time_us;
            // static double last_ref_lat = init_lat_;
            // static double last_ref_lon = init_lon_;
            // static float last_ref_alt = world_origin_asml_alt_;
            if (last_ref_timestamp_ == 0) { last_ref_timestamp_ = time_us;} // initialize
            if (last_ref_lat_ == 0) { last_ref_lat_ = init_lat_;} // initialize
            if (last_ref_lon_ == 0) { last_ref_lon_ = init_lon_;} // initialize
            if (last_ref_alt_ == 0) { last_ref_alt_ = world_origin_asml_alt_;} // initialize
            if (_vehicle_command_sub.updated())
            {
                vehicle_command_s vehicle_command;
                if (_vehicle_command_sub.update(&vehicle_command)) {
                    if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN) {
                        if (-pos_ned[2] < 0.01) {
                            last_ref_timestamp_ = time_us;
                            last_ref_lat_ = vehicle_command.param5;
                            last_ref_lon_ = vehicle_command.param6;
                            last_ref_alt_ = vehicle_command.param7;
                            std::cout << std::setprecision(12) << "[PX4SITL] New NED origin (LLA): " << last_ref_lat_ << ", " << last_ref_lon_ << ", " << last_ref_alt_ << std::endl;
                        }
                        else{
                            std::cout << "[PX4SITL] Error! Set new NED origin in air is not allowed!" << std::endl;
                        }
                    }
                }    
            }
            vehicle_local_position_msg.ref_timestamp = last_ref_timestamp_; // Time when reference position was set since system start, (microseconds)
            vehicle_local_position_msg.ref_lat = last_ref_lat_; // (degrees) lat at local position(0,0)
            vehicle_local_position_msg.ref_lon = last_ref_lon_; // (degrees) lon at local position(0,0)
            vehicle_local_position_msg.ref_alt = last_ref_alt_; // (metres) AMSL(Geoid) altitude at local position(0,0,0)
            /* Calculate the local x(North), y(East) according to ref lat/lon */
            float north, east;
            MapProjection global_local_proj_ref{vehicle_local_position_msg.ref_lat, vehicle_local_position_msg.ref_lon, time_us}; // MapProjection from PX4 geo.h
            global_local_proj_ref.project(vehicle_global_position_msg.lat, vehicle_global_position_msg.lon, north, east);
            vehicle_local_position_msg.x = north;
            vehicle_local_position_msg.y = east;
            vehicle_local_position_msg.z = pos_ned[2];
            break;
        }
        default: {
            throw std::invalid_argument("[PX4SITL::UpdateDroneStates] Invalid local_pos_source_");
            break;
        }
    }
    vehicle_local_position_msg.vx = vel_ned[0];
    vehicle_local_position_msg.vy = vel_ned[1];
    vehicle_local_position_msg.vz = vel_ned[2];   
    vehicle_local_position_msg.z_deriv = vel_ned[2];   
    vehicle_local_position_msg.ax = acc_ned[0];
    vehicle_local_position_msg.ay = acc_ned[1];
    vehicle_local_position_msg.az = acc_ned[2]; 
    // vehicle_local_position_msg.heading = q.toRotationMatrix().eulerAngles(2,1,0);
    /* Instance is set from a quaternion representing transformation
	 * from frame 2 to frame 1.
	 * This instance will hold the angles defining the 3-2-1 intrinsic
	 * Tait-Bryan rotation sequence from frame 1 to frame 2.
     */
    double q_vec[] = {q_ned_frd.w(),q_ned_frd.x(),q_ned_frd.y(),q_ned_frd.z()};
    vehicle_local_position_msg.heading = matrix::Eulerd{matrix::Quatd{q_vec}}.psi(); 
    vehicle_local_position_msg.heading_good_for_control = true;
    _local_position_pub.publish(vehicle_local_position_msg);


    // Refer to https://github.com/PX4/PX4-Autopilot/blob/v1.13.3/msg/vehicle_angular_velocity.msg
    vehicle_angular_velocity_s vehicle_angular_velocity_msg{};
    vehicle_angular_velocity_msg.timestamp = time_us;
    vehicle_angular_velocity_msg.timestamp_sample = time_us;
    vehicle_angular_velocity_msg.xyz[0] = omega_frd[0];
    vehicle_angular_velocity_msg.xyz[1] = omega_frd[1];
    vehicle_angular_velocity_msg.xyz[2] = omega_frd[2];
    _vehicle_angular_velocity_pub.publish(vehicle_angular_velocity_msg);


    // Refer to https://github.com/PX4/PX4-Autopilot/blob/v1.13.3/msg/battery_status.msg
    battery_status_s battery_status_msg{};
    battery_status_msg.timestamp = time_us;
    battery_status_msg.connected = true;
    battery_status_msg.voltage_filtered_v = 7.6; //V
    battery_status_msg.current_filtered_a = 7.0; // A
    battery_status_msg.remaining = 0.99; // %
    _battery_status_pub.publish(battery_status_msg);

    //@TODO estimator states

    //@TODO vehicle_odometry_s (mocap/vision input position)
    // // Refer to https://github.com/PX4/PX4-Autopilot/blob/v1.13.3/msg/vehicle_odometry.msg
    // vehicle_odometry_s vehicle_odometry_msg{};
    // vehicle_odometry_msg.timestamp = time_us;
    // vehicle_odometry_msg.timestamp_sample = time_us;
    // vehicle_odometry_msg.local_frame = vehicle_odometry_s::LOCAL_FRAME_NED;
    // vehicle_odometry_msg.x = pos_ned[0];
    // vehicle_odometry_msg.y = pos_ned[1];
    // vehicle_odometry_msg.z = pos_ned[2];
    // vehicle_odometry_msg.q[0] = q_ned_frd.w();
    // vehicle_odometry_msg.q[1] = q_ned_frd.x();
    // vehicle_odometry_msg.q[2] = q_ned_frd.y();
    // vehicle_odometry_msg.q[3] = q_ned_frd.z();
    // vehicle_odometry_msg.q_offset[0] = 1.0;
    // vehicle_odometry_msg.q_offset[1] = 0.0;
    // vehicle_odometry_msg.q_offset[2] = 0.0;
    // vehicle_odometry_msg.q_offset[3] = 0.0;
    // vehicle_odometry_msg.velocity_frame = vehicle_odometry_s::LOCAL_FRAME_NED;
    // vehicle_odometry_msg.vx = vel_ned[0];
    // vehicle_odometry_msg.vy = vel_ned[1];
    // vehicle_odometry_msg.vz = vel_ned[2];   
    // vehicle_odometry_msg.rollspeed = omega_frd[0];
    // vehicle_odometry_msg.pitchspeed = omega_frd[1];
    // vehicle_odometry_msg.yawspeed = omega_frd[2];
    // _odometry_pub.publish(vehicle_odometry_msg);
}


template <px4::params p>
void PX4SITL::get_px4_param(float& output)
{
	    // static type-check
	    static_assert(px4::parameters_type[(int)p] == PARAM_TYPE_FLOAT, "parameter type must be float");

        context_.parameter_store.get(static_cast<param_t>(p), &output);
}

template <px4::params p>
void PX4SITL::get_px4_param(int32_t& output)
{
	    // static type-check
	    static_assert(px4::parameters_type[(int)p] == PARAM_TYPE_INT32, "parameter type must be int32_t");

        context_.parameter_store.get(static_cast<param_t>(p), &output);
}

template <px4::params p>
void PX4SITL::get_px4_param(bool& output)
{
	    // static type-check
	    static_assert(px4::parameters_type[(int)p] == PARAM_TYPE_INT32, "parameter type must be int32_t");

        int32_t value = 0;
        context_.parameter_store.get(static_cast<param_t>(p), &value);
        output = value != 0;
}


}
