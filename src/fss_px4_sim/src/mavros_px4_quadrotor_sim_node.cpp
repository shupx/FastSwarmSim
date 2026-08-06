/**
 * @file mavros_px4_quadrotor_sim_node.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Mavros(sim) + PX4 controller + quadrotor_dynamics. main loop
 * 
 * Note: This program relies on mavros_sim, px4_sitl, quadrotor_dynamics and fss_time
 * 
 * @version 1.0
 * @date 2026-08-06
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2026, Peixuan Shu
 * All rights reserved.
 * 
 */

#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "fss_time/fss_time.hpp"
#include "fss_px4_sim/mavros_sim/plugins.hpp"
#include "fss_px4_sim/px4_sitl.hpp"
#include "fss_px4_sim/quadrotor_dynamics.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
using MavrosQuadSimulator::Dynamics;
using MavrosQuadSimulator::PX4SITL;
constexpr int kAgentId = 0;
}

// One executable represents exactly one vehicle.  This node owns the PX4
// runtime and physics only; all ROS/MAVROS endpoints live in MavrosSim.
class MavrosPx4QuadrotorSim final : public rclcpp::Node
{
public:
  MavrosPx4QuadrotorSim()
  : Node("mavros_px4_quadrotor_sim_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    const auto init_x = parameter("init_x_East_metre", 0.0);
    const auto init_y = parameter("init_y_North_metre", 0.0);
    const auto init_z = parameter("init_z_Up_metre", 0.0);
    const auto init_roll = parameter("init_roll_deg", 0.0);
    const auto init_pitch = parameter("init_pitch_deg", 0.0);
    const auto init_yaw = parameter("init_yaw_deg", 0.0);

    // The copied PX4 v1.13.3 runtime uses agent-indexed storage.  Full-node
    // launch creates one process per UAV, making index zero process-local.
    px4::allocate_mavlink_message_storage(1);
    uORB_sim::allocate_uorb_message_storage(1);
    px4::allocate_px4_params_storage(1);

    dynamics_ = std::make_shared<Dynamics>();
    dynamics_->setSimStep(0.01);
    dynamics_->setPos(init_x, init_y, init_z);
    dynamics_->setRPY(init_roll * M_PI / 180.0, init_pitch * M_PI / 180.0,
      init_yaw * M_PI / 180.0);
    px4_sitl_ = std::make_shared<PX4SITL>(kAgentId, *this, dynamics_);
    mavros_sim_ = std::make_unique<fss_px4_sim::mavros_sim::MavrosSim>(
      *this, [this](px4::mavlink_receive_handle handle, const mavlink_message_t & message) {
        std::scoped_lock lock(px4_mutex_);
        auto & entry = px4::mavlink_receive_lists.at(kAgentId).at(static_cast<size_t>(handle));
        entry.msg = message;
        entry.updated = true;
      }); // This lambda is called by mavros_sim when new mavlink messages are received from ROS topics. It stores the mavlink message into the PX4 receive list, which will be processed by the PX4 SITL.
  }

  void run()
  {
    if (declare_parameter("use_fss_sim_time", false).get_value<bool>() == true) {
      auto fss_time_participant = fss_time::thread_time_participant::for_current_thread(*this, "mavros_px4_quadrotor_sim_node", false);
      fss_time_participant.set_follows_real_time(false); // Blocks the fss_time coordinator until it finishes one loop iteration, even it is slower than real time. This is necessary to ensure that each step of the simulated dynamics and PX4 SITL is finished before the next step, otherwise the simulation will be unstable.
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
      {
        std::scoped_lock lock(px4_mutex_);
        px4_sitl_->Run(static_cast<uint64_t>(stamp.nanoseconds() / 1000));
        dynamics_->step(last_time, current_time);
        publish_streams(stamp);
      }
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

  void publish_streams(const rclcpp::Time & stamp)
  {
    for (auto & entry : px4::mavlink_stream_lists.at(kAgentId)) {
      if (!entry.updated) continue;
      mavros_sim_->handle_message(entry.msg, stamp);
      entry.updated = false;
    }
  }

  std::mutex px4_mutex_;
  std::shared_ptr<Dynamics> dynamics_;
  std::shared_ptr<PX4SITL> px4_sitl_;
  std::unique_ptr<fss_px4_sim::mavros_sim::MavrosSim> mavros_sim_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MavrosPx4QuadrotorSim>();

  std::thread spin_thread([node]() {
    rclcpp::spin(node);
  });

  node->run(); // main loop controlled by fss_time::Rate to allow synchronization with other nodes in the simulation.

  rclcpp::shutdown();

  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  return 0;
}
