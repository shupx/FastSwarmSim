#ifndef FSS_PX4_SIM__MAVROS_SIM__PLUGIN_HPP_
#define FSS_PX4_SIM__MAVROS_SIM__PLUGIN_HPP_

#include <functional>
#include <memory>

#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include "px4_modules/mavlink/mavlink_msg_list.hpp"

namespace fss_px4_sim::mavros_sim
{

using MavlinkIngress = std::function<void(px4::mavlink_receive_handle, const mavlink_message_t &)>;

struct SharedState
{
  Eigen::Quaterniond attitude_enu{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d body_omega_flu{Eigen::Vector3d::Zero()};
};

class Plugin
{
public:
  Plugin(rclcpp::Node & node, MavlinkIngress ingress, std::shared_ptr<SharedState> state)
  : node_(node), ingress_(std::move(ingress)), state_(std::move(state)) {}
  virtual ~Plugin() = default;
  virtual bool handles(uint32_t message_id) const = 0;
  virtual void handle_message(const mavlink_message_t & message, const rclcpp::Time & stamp) = 0;

protected:
  template<typename T>
  T parameter(const std::string & name, const T & default_value)
  {
    if (!node_.has_parameter(name)) {
      node_.declare_parameter<T>(name, default_value);
    }
    return node_.get_parameter(name).get_value<T>();
  }

  rclcpp::Node & node_;
  MavlinkIngress ingress_;
  std::shared_ptr<SharedState> state_;
};

std::unique_ptr<Plugin> make_setpoint_raw_plugin(rclcpp::Node &, MavlinkIngress, std::shared_ptr<SharedState>);
std::unique_ptr<Plugin> make_local_position_plugin(rclcpp::Node &, MavlinkIngress, std::shared_ptr<SharedState>);
std::unique_ptr<Plugin> make_imu_plugin(rclcpp::Node &, MavlinkIngress, std::shared_ptr<SharedState>);
std::unique_ptr<Plugin> make_system_status_plugin(rclcpp::Node &, MavlinkIngress, std::shared_ptr<SharedState>);
std::unique_ptr<Plugin> make_command_plugin(rclcpp::Node &, MavlinkIngress, std::shared_ptr<SharedState>);
std::unique_ptr<Plugin> make_global_position_plugin(rclcpp::Node &, MavlinkIngress, std::shared_ptr<SharedState>);

}  // namespace fss_px4_sim::mavros_sim

#endif  // FSS_PX4_SIM__MAVROS_SIM__PLUGIN_HPP_
