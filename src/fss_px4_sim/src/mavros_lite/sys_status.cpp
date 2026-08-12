#include "fss_px4_sim/mavros_lite/core.hpp"

#include <mavros/px4_custom_mode.hpp>
#include <cstring>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/status_text.hpp>
#include <mavros_msgs/msg/sys_status.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include <chrono>

namespace fss_px4_sim::mavros_lite
{
class SysStatus final : public Module
{
public:
  explicit SysStatus(Core & core)
  : Module(core)
  {
    const auto qos = rclcpp::SensorDataQoS();
    state_pub_ = core.create_publisher<mavros_msgs::msg::State>("state", qos);
    sys_pub_ = core.create_publisher<mavros_msgs::msg::SysStatus>("sys_status", qos);
    battery_pub_ = core.create_publisher<sensor_msgs::msg::BatteryState>("battery", qos);
    text_pub_ = core.create_publisher<mavros_msgs::msg::StatusText>("statustext/recv", qos);
    mode_srv_ = core.create_service<mavros_msgs::srv::SetMode>("set_mode",
      [this](const std::shared_ptr<rmw_request_id_t>,
        mavros_msgs::srv::SetMode::Request::SharedPtr request,
        mavros_msgs::srv::SetMode::Response::SharedPtr response) {send_mode(*request, *response);});
    heartbeat_timer_ = core.create_wall_timer(
      std::chrono::seconds(1), [this] {send_heartbeat();});
    timeout_timer_ = core.create_wall_timer(
      std::chrono::seconds(1), [this] {check_connection();});
  }
  void handle_message(const mavlink_message_t & message) override
  {
    if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
      mavlink_heartbeat_t value{}; mavlink_msg_heartbeat_decode(&message, &value);
      mavros_msgs::msg::State output; output.header.stamp = core_.stamp(); output.connected = true;
      output.armed = value.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;
      output.guided = value.base_mode & MAV_MODE_FLAG_GUIDED_ENABLED;
      output.manual_input = value.base_mode & MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
      output.system_status = value.system_status;
      output.mode = mode_name(value.custom_mode, value.base_mode);
      {std::lock_guard<std::mutex> lock(core_.state().mutex); core_.state().connected = true; core_.state().armed = output.armed; core_.state().hil = value.base_mode & MAV_MODE_FLAG_HIL_ENABLED;}
      {std::lock_guard<std::mutex> lock(heartbeat_mutex_); last_heartbeat_ = std::chrono::steady_clock::now();}
      state_pub_->publish(output);
    } else if (message.msgid == MAVLINK_MSG_ID_SYS_STATUS) {
      mavlink_sys_status_t value{}; mavlink_msg_sys_status_decode(&message, &value);
      mavros_msgs::msg::SysStatus output; output.header.stamp = core_.stamp();
      output.sensors_present = value.onboard_control_sensors_present; output.sensors_enabled = value.onboard_control_sensors_enabled;
      output.sensors_health = value.onboard_control_sensors_health; output.load = value.load;
      output.voltage_battery = value.voltage_battery; output.current_battery = value.current_battery; output.battery_remaining = value.battery_remaining;
      output.drop_rate_comm = value.drop_rate_comm; output.errors_comm = value.errors_comm; output.errors_count1 = value.errors_count1;
      output.errors_count2 = value.errors_count2; output.errors_count3 = value.errors_count3; output.errors_count4 = value.errors_count4; sys_pub_->publish(output);
      sensor_msgs::msg::BatteryState battery; battery.header.stamp = output.header.stamp; battery.voltage = value.voltage_battery / 1000.0;
      battery.current = -value.current_battery / 100.0; battery.percentage = value.battery_remaining / 100.0; battery.present = true; battery_pub_->publish(battery);
    } else if (message.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
      mavlink_statustext_t value{}; mavlink_msg_statustext_decode(&message, &value);
      mavros_msgs::msg::StatusText output; output.header.stamp = core_.stamp(); output.severity = value.severity;
      output.text.assign(reinterpret_cast<char *>(value.text), strnlen(reinterpret_cast<char *>(value.text), sizeof(value.text)));
      text_pub_->publish(output);
    }
  }
private:
  std::string mode_name(uint32_t custom, uint8_t base) const
  {
    if (!(base & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)) return "MANUAL";
    px4::custom_mode mode(custom);
    switch (mode.mode.main_mode) {
      case px4::custom_mode::MAIN_MODE_OFFBOARD: return "OFFBOARD";
      case px4::custom_mode::MAIN_MODE_POSCTL: return "POSCTL";
      case px4::custom_mode::MAIN_MODE_ALTCTL: return "ALTCTL";
      case px4::custom_mode::MAIN_MODE_AUTO: return "AUTO";
      case px4::custom_mode::MAIN_MODE_STABILIZED: return "STABILIZED";
      default: return "MANUAL";
    }
  }
  void send_mode(const mavros_msgs::srv::SetMode::Request & request, mavros_msgs::srv::SetMode::Response & response)
  {
    mavlink_message_t message{}; uint32_t custom = 0;
    if (request.custom_mode == "OFFBOARD") custom = px4::define_mode(px4::custom_mode::MAIN_MODE_OFFBOARD);
    else if (request.custom_mode == "POSCTL") custom = px4::define_mode(px4::custom_mode::MAIN_MODE_POSCTL);
    else if (request.custom_mode == "ALTCTL") custom = px4::define_mode(px4::custom_mode::MAIN_MODE_ALTCTL);
    else if (request.custom_mode == "AUTO") custom = px4::define_mode(px4::custom_mode::MAIN_MODE_AUTO);
    else if (!request.custom_mode.empty()) {response.mode_sent = false; return;}
    mavlink_msg_set_mode_pack(core_.system_id(), core_.component_id(), &message, core_.target_system(), request.base_mode | (request.custom_mode.empty() ? 0 : MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), custom);
    core_.send_message(message); response.mode_sent = true;
  }
  void send_heartbeat()
  {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(
      core_.system_id(), core_.component_id(), &message, MAV_TYPE_ONBOARD_CONTROLLER,
      MAV_AUTOPILOT_INVALID, MAV_MODE_MANUAL_ARMED, 0, MAV_STATE_ACTIVE);
    core_.send_message(message);
  }
  void check_connection()
  {
    std::lock_guard<std::mutex> heartbeat_lock(heartbeat_mutex_);
    if (last_heartbeat_.time_since_epoch().count() == 0 ||
      std::chrono::steady_clock::now() - last_heartbeat_ < std::chrono::seconds(10)) return;
    mavros_msgs::msg::State output; output.header.stamp = core_.stamp(); output.connected = false;
    output.system_status = MAV_STATE_UNINIT; state_pub_->publish(output);
    {std::lock_guard<std::mutex> lock(core_.state().mutex); core_.state().connected = false;}
    last_heartbeat_ = {};
  }
  rclcpp::Publisher<mavros_msgs::msg::State>::SharedPtr state_pub_;
  rclcpp::Publisher<mavros_msgs::msg::SysStatus>::SharedPtr sys_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<mavros_msgs::msg::StatusText>::SharedPtr text_pub_;
  rclcpp::Service<mavros_msgs::srv::SetMode>::SharedPtr mode_srv_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_, timeout_timer_;
  std::mutex heartbeat_mutex_;
  std::chrono::steady_clock::time_point last_heartbeat_{};
};
std::unique_ptr<Module> make_sys_status(Core & core) {return std::make_unique<SysStatus>(core);}
}  // namespace fss_px4_sim::mavros_lite
