/**
 * @file px4_rotor_sim_node.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief tailored PX4 core components (pos+att controller, udp mavlink, FSM commander) + quadrotor_dynamics. main loop
 * 
 * Note: This program relies on px4_sitl, quadrotor_dynamics and fss_time
 * 
 * @version 1.0
 * @date 2026-08-06
 * Modified by Peixuan Shu (2026-08-09): PX4SITL owns its instance-local
 * simulation context instead of process-global agent storage.
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2026, Peixuan Shu
 * All rights reserved.
 * 
 */

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include "fss_time/fss_time.hpp"
#include "fss_px4_sim/mavros_lite/core.hpp"
#include "fss_px4_sim/px4_sitl.hpp"
#include "fss_px4_sim/quadrotor_dynamics.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
using MavrosQuadSimulator::Dynamics;
using MavrosQuadSimulator::PX4SITL;
}

// Modified by Peixuan Shu: one node owns the PX4 runtime and physics for
// one vehicle; MAVROS communicates directly with PX4SITL over UDP.
class MavrosPx4QuadrotorSim final : public rclcpp::Node
{
public:
  MavrosPx4QuadrotorSim()
  : Node("px4_rotor_sim_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    const auto init_x = parameter("init_x_East_metre", 0.0);
    const auto init_y = parameter("init_y_North_metre", 0.0);
    const auto init_z = parameter("init_z_Up_metre", 0.0);
    const auto init_roll = parameter("init_roll_deg", 0.0);
    const auto init_pitch = parameter("init_pitch_deg", 0.0);
    const auto init_yaw = parameter("init_yaw_deg", 0.0);

    dynamics_ = std::make_shared<Dynamics>();
    dynamics_->setSimStep(0.01);
    dynamics_->setPos(init_x, init_y, init_z);
    dynamics_->setRPY(init_roll * M_PI / 180.0, init_pitch * M_PI / 180.0,
      init_yaw * M_PI / 180.0);
    
    px4_sitl_ = std::make_shared<PX4SITL>(*this, dynamics_);

    /* create mavros_lite if use direct ros interface (px4_sitl pub and sub mavros-like topics directly instead of using udp) */
    if (px4_sitl_->uses_direct_ros()) {
      fss_px4_sim::mavros_lite::MavrosLite::Config config;
      config.target_system = px4_sitl_->mavlink_system_id();
      config.target_component = px4_sitl_->mavlink_component_id();
      config.topic_prefix = "mavros";
      mavros_lite_ = std::make_shared<fss_px4_sim::mavros_lite::MavrosLite>(*this, config);

      const std::weak_ptr<PX4SITL> px4_sitl_weak = px4_sitl_;
      const std::weak_ptr<fss_px4_sim::mavros_lite::MavrosLite> mavros_lite_weak = mavros_lite_;
      mavros_lite_->set_send_callback([px4_sitl_weak](const mavlink_message_t &message) {
        if (const auto px4_sitl = px4_sitl_weak.lock()) {
          px4_sitl->receive_mavlink_message(message);
        }
      });
      px4_sitl_->set_mavlink_send_callback([mavros_lite_weak](const mavlink_message_t &message) {
        if (const auto mavros_lite = mavros_lite_weak.lock()) {
          mavros_lite->receive_message(message);
        }
      });
    }
  }

  void run()
  {
    if (parameter("use_fss_sim_time", false)) {
      auto & fss_time_participant = fss_time::thread_time_participant::for_current_thread(*this, "px4_rotor_sim_node");
      fss_time_participant.set_follows_real_time(false); // If false, the fss_time coordinator will wait for the current loop iteration to finish before proceeding, and will not enforce real-time pacing, even if an iteration takes longer than the real-time period. This is necessary to ensure that each step of the simulated dynamics and PX4 SITL is finished before the next step, otherwise the simulation will be unstable.
    }
    fss_time::Rate rate(*this, 100.0);
    double last_time = now().seconds();
    while (rclcpp::ok()) {
      const auto stamp = now();
      const double current_time = stamp.seconds();
      if (current_time < last_time) {
        RCLCPP_ERROR(get_logger(), "fss_time moved backwards from %.9f to %.9f", last_time, current_time);
        last_time = current_time;
      }
      px4_sitl_->Run(static_cast<uint64_t>(stamp.nanoseconds() / 1000));
      dynamics_->step(last_time, current_time);
      last_time = current_time;
      rate.sleep();
    }
  }

private:
  template<typename T>
  T parameter(const std::string & name, const T & default_value)
  {
    if (!has_parameter(name)) declare_parameter<T>(name, default_value);
    return get_parameter(name).get_value<T>();
  }

  std::shared_ptr<Dynamics> dynamics_;
  std::shared_ptr<PX4SITL> px4_sitl_;
  std::shared_ptr<fss_px4_sim::mavros_lite::MavrosLite> mavros_lite_;

public:
  std::shared_ptr<fss_px4_sim::mavros_lite::MavrosLite> mavros_lite() const
  {
    return mavros_lite_;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MavrosPx4QuadrotorSim>();

  std::thread spin_thread([node]() {
    if (node->mavros_lite()) {
      // Some MAVROS Lite plugin services synchronously wait for MAVLink replies.
      // Keep a second executor thread available for the reply callbacks.
      const auto executor_threads = std::clamp(std::thread::hardware_concurrency(), 2u, 3u);
      rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), executor_threads);
      executor.add_node(node);
      executor.spin();
    } else {
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(node);
      executor.spin();
    }
  });

  node->run(); // main loop controlled by fss_time::Rate to allow synchronization with other nodes in the simulation.

  rclcpp::shutdown();

  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  return 0;
}
