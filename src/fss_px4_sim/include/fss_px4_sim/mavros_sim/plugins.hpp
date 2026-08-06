#ifndef FSS_PX4_SIM__MAVROS_SIM__PLUGINS_HPP_
#define FSS_PX4_SIM__MAVROS_SIM__PLUGINS_HPP_

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "px4_modules/mavlink/mavlink_msg_list.hpp"

namespace fss_px4_sim::mavros_sim
{

// The shim owns every ROS endpoint.  It is deliberately separated from the
// PX4 loop: callbacks only encode MAVLink into this sink and never mutate
// uORB, parameters, or PX4 controller state directly.
using MavlinkIngress = std::function<void(
    px4::mavlink_receive_handle, const mavlink_message_t &)>;

class MavrosSim
{
public:
  MavrosSim(rclcpp::Node & node, MavlinkIngress ingress);
  ~MavrosSim();

  MavrosSim(const MavrosSim &) = delete;
  MavrosSim & operator=(const MavrosSim &) = delete;

  // Called by the owning 100 Hz PX4 loop for every MAVLink telemetry frame.
  void handle_message(const mavlink_message_t & message, const rclcpp::Time & stamp);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace fss_px4_sim::mavros_sim

#endif  // FSS_PX4_SIM__MAVROS_SIM__PLUGINS_HPP_
