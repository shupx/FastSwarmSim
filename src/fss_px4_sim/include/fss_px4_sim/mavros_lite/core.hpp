#pragma once

#include <arpa/inet.h>
#include <mavlink/v2.0/common/mavlink.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
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

class Core;

class Module
{
public:
  explicit Module(Core & core) : core_(core) {}
  virtual ~Module() = default;
  virtual void handle_message(const mavlink_message_t & message) = 0;

protected:
  Core & core_;
};

class Core : public rclcpp::Node
{
public:
  Core();
  ~Core() override;

  void add_module(std::unique_ptr<Module> module);
  void send_message(mavlink_message_t & message);
  rclcpp::Time stamp() const {return now();}
  uint8_t target_system() const {return target_system_;}
  uint8_t target_component() const {return target_component_;}
  uint8_t system_id() const {return system_id_;}
  uint8_t component_id() const {return component_id_;}
  SharedState & state() {return state_;}

private:
  void start_transport();
  void receive_loop();

  int socket_fd_{-1};
  uint16_t bind_port_{};
  std::string remote_host_;
  uint16_t remote_port_{};
  uint8_t target_system_{};
  uint8_t target_component_{};
  uint8_t system_id_{};
  uint8_t component_id_{};
  sockaddr_in remote_address_{};
  bool remote_learned_{false};
  std::atomic<bool> running_{false};
  std::thread receive_thread_;
  std::mutex send_mutex_;
  std::vector<std::unique_ptr<Module>> modules_;
  SharedState state_;
};

std::unique_ptr<Module> make_setpoint_raw(Core & core);
std::unique_ptr<Module> make_local_position(Core & core);
std::unique_ptr<Module> make_imu(Core & core);
std::unique_ptr<Module> make_sys_status(Core & core);
std::unique_ptr<Module> make_command(Core & core);
std::unique_ptr<Module> make_global_position(Core & core);

}  // namespace fss_px4_sim::mavros_lite
