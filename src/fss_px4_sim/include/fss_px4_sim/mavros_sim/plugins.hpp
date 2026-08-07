#ifndef FSS_PX4_SIM__MAVROS_SIM__PLUGINS_HPP_
#define FSS_PX4_SIM__MAVROS_SIM__PLUGINS_HPP_

#include <memory>
#include <vector>

#include "fss_px4_sim/mavros_sim/plugin.hpp"

namespace fss_px4_sim::mavros_sim
{

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
  std::shared_ptr<SharedState> state_;
  std::vector<std::unique_ptr<Plugin>> plugins_;
};

}  // namespace fss_px4_sim::mavros_sim

#endif  // FSS_PX4_SIM__MAVROS_SIM__PLUGINS_HPP_
