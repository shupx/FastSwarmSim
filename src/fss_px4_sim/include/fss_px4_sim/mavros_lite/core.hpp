#pragma once

#include <mavlink/v2.0/common/mavlink.h>

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace fss_px4_sim::mavros_lite
{

struct SharedState
{
  std::mutex mutex;
  geometry_msgs::msg::Quaternion orientation;
  geometry_msgs::msg::Vector3 angular_velocity;
  sensor_msgs::msg::NavSatFix gps_fix;
  bool have_attitude{false};
  bool have_gps_fix{false};
  bool connected{false};
  bool armed{false};
  bool hil{false};
};

class MavrosLite;

class Module
{
public:
  explicit Module(MavrosLite & core) : core_(core) {}
  virtual ~Module() = default;
  virtual void handle_message(const mavlink_message_t & message) = 0;

protected:
  MavrosLite & core_;
};

class MavrosLite
{
public:
  using SendCallback = std::function<void(const mavlink_message_t &)>;

  struct Config
  {
    uint8_t target_system{1};
    uint8_t target_component{1};
    uint8_t system_id{1};
    uint8_t component_id{MAV_COMP_ID_ONBOARD_COMPUTER};
    std::string topic_prefix;
  };

  explicit MavrosLite(rclcpp::Node & node);
  MavrosLite(rclcpp::Node & node, const Config & config);
  ~MavrosLite() = default;

  void add_module(std::unique_ptr<Module> module);
  void receive_message(const mavlink_message_t & message);
  void set_send_callback(SendCallback callback);
  void send_message(mavlink_message_t & message);
  rclcpp::Time stamp() const {return node_.now();}
  rclcpp::Node & node() {return node_;}
  const rclcpp::Node & node() const {return node_;}
  rclcpp::Logger logger() const {return node_.get_logger();}
  rclcpp::Clock::SharedPtr clock() const {return node_.get_clock();}

  template<typename ParameterT>
  ParameterT declare_parameter(const std::string & name, const ParameterT & default_value)
  {
    return node_.declare_parameter<ParameterT>(name, default_value);
  }

  template<typename MessageT, typename ... Args>
  auto create_publisher(const std::string & topic, Args && ... args)
  {
    return node_.create_publisher<MessageT>(resolve_topic(topic), std::forward<Args>(args)...);
  }

  template<typename MessageT, typename ... Args>
  auto create_subscription(const std::string & topic, Args && ... args)
  {
    return node_.create_subscription<MessageT>(resolve_topic(topic), std::forward<Args>(args)...);
  }

  template<typename ServiceT, typename ... Args>
  auto create_service(const std::string & service, Args && ... args)
  {
    return node_.create_service<ServiceT>(resolve_topic(service), std::forward<Args>(args)...);
  }

  template<typename ... Args>
  auto create_wall_timer(Args && ... args)
  {
    return node_.create_wall_timer(std::forward<Args>(args)...);
  }
  uint8_t target_system() const {return target_system_;}
  uint8_t target_component() const {return target_component_;}
  uint8_t system_id() const {return system_id_;}
  uint8_t component_id() const {return component_id_;}
  SharedState & state() {return state_;}

private:
  std::string resolve_topic(const std::string & name) const;

  rclcpp::Node & node_;
  std::string topic_prefix_;
  uint8_t target_system_{};
  uint8_t target_component_{};
  uint8_t system_id_{};
  uint8_t component_id_{};
  std::mutex send_mutex_;
  SendCallback send_callback_;
  std::vector<std::unique_ptr<Module>> modules_;
  SharedState state_;
};

std::unique_ptr<Module> make_setpoint_raw(MavrosLite & core);
std::unique_ptr<Module> make_local_position(MavrosLite & core);
std::unique_ptr<Module> make_imu(MavrosLite & core);
std::unique_ptr<Module> make_sys_status(MavrosLite & core);
std::unique_ptr<Module> make_command(MavrosLite & core);
std::unique_ptr<Module> make_global_position(MavrosLite & core);

}  // namespace fss_px4_sim::mavros_lite
