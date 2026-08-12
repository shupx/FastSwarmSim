#include "fss_px4_sim/mavros_lite/core.hpp"

namespace fss_px4_sim::mavros_lite
{

MavrosLite::MavrosLite(rclcpp::Node & node)
: node_(node)
{
  Config config;
  config.target_system = static_cast<uint8_t>(declare_parameter<int>("target_system", config.target_system));
  config.target_component = static_cast<uint8_t>(declare_parameter<int>("target_component", config.target_component));
  config.system_id = static_cast<uint8_t>(declare_parameter<int>("system_id", config.system_id));
  config.component_id = static_cast<uint8_t>(declare_parameter<int>("component_id", config.component_id));

  target_system_ = config.target_system;
  target_component_ = config.target_component;
  system_id_ = config.system_id;
  component_id_ = config.component_id;

  add_module(make_imu(*this));
  add_module(make_local_position(*this));
  add_module(make_global_position(*this));
  add_module(make_setpoint_raw(*this));
  add_module(make_command(*this));
  add_module(make_sys_status(*this));
}

MavrosLite::MavrosLite(rclcpp::Node & node, const Config & config)
: node_(node), topic_prefix_(config.topic_prefix),
  target_system_(config.target_system), target_component_(config.target_component),
  system_id_(config.system_id), component_id_(config.component_id)
{
  add_module(make_imu(*this));
  add_module(make_local_position(*this));
  add_module(make_global_position(*this));
  add_module(make_setpoint_raw(*this));
  add_module(make_command(*this));
  add_module(make_sys_status(*this));
}

std::string MavrosLite::resolve_topic(const std::string & name) const
{
  return topic_prefix_.empty() ? name : topic_prefix_ + "/" + name;
}

void MavrosLite::add_module(std::unique_ptr<Module> module)
{
  modules_.push_back(std::move(module));
}

void MavrosLite::set_send_callback(SendCallback callback)
{
  std::lock_guard<std::mutex> lock(send_mutex_);
  send_callback_ = std::move(callback);
}

void MavrosLite::send_message(mavlink_message_t & message)
{
  std::lock_guard<std::mutex> lock(send_mutex_);
  if (send_callback_) {
    send_callback_(message);
    return;
  }
  RCLCPP_WARN_THROTTLE(
    node_.get_logger(), *node_.get_clock(), 5000, "No MAVLink send callback configured");
}

void MavrosLite::receive_message(const mavlink_message_t & message)
{
  if (message.sysid != target_system_) {
    return;
  }
  for (auto & module : modules_) {
    module->handle_message(message);
  }
}

}  // namespace fss_px4_sim::mavros_lite
