#include "fss_px4_sim/mavros_sim/plugins.hpp"

namespace fss_px4_sim::mavros_sim
{

MavrosSim::MavrosSim(rclcpp::Node & node, MavlinkIngress ingress)
: state_(std::make_shared<SharedState>())
{
  // Keep plugin ownership and MAVLink dispatch separate, as MAVROS ROS 2 does.
  // This transport is simulator-local: callbacks only enqueue frames for PX4.
  plugins_.emplace_back(make_setpoint_raw_plugin(node, ingress, state_));
  plugins_.emplace_back(make_local_position_plugin(node, ingress, state_));
  plugins_.emplace_back(make_imu_plugin(node, ingress, state_));
  plugins_.emplace_back(make_system_status_plugin(node, ingress, state_));
  plugins_.emplace_back(make_command_plugin(node, ingress, state_));
  plugins_.emplace_back(make_global_position_plugin(node, ingress, state_));
}

MavrosSim::~MavrosSim() = default;

/* Publish mavlink messages into ROS topics */
void MavrosSim::handle_message(const mavlink_message_t & message, const rclcpp::Time & stamp)
{
  for (const auto & plugin : plugins_) {
    if (plugin->handles(message.msgid)) {
      plugin->handle_message(message, stamp);
      return;
    }
  }
}

}  // namespace fss_px4_sim::mavros_sim
