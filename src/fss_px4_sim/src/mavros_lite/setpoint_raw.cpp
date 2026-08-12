#include "fss_px4_sim/mavros_lite/core.hpp"

#include <algorithm>
#include <cmath>

#include <mavros/frame_tf.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/global_position_target.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace fss_px4_sim::mavros_lite
{
namespace ftf = mavros::ftf;

class SetpointRaw final : public Module
{
public:
  explicit SetpointRaw(MavrosLite & core)
  : Module(core)
  {
    thrust_scaling_ = core.declare_parameter<double>("setpoint_raw.thrust_scaling", 1.0);
    auto qos = rclcpp::SensorDataQoS();
    local_pub_ = core.create_publisher<mavros_msgs::msg::PositionTarget>(
      "setpoint_raw/target_local", qos);
    global_pub_ = core.create_publisher<mavros_msgs::msg::GlobalPositionTarget>(
      "setpoint_raw/target_global", qos);
    attitude_pub_ = core.create_publisher<mavros_msgs::msg::AttitudeTarget>(
      "setpoint_raw/target_attitude", qos);
    local_sub_ = core.create_subscription<mavros_msgs::msg::PositionTarget>(
      "setpoint_raw/local", qos,
      [this](mavros_msgs::msg::PositionTarget::SharedPtr msg) {send_local(*msg);});
    global_sub_ = core.create_subscription<mavros_msgs::msg::GlobalPositionTarget>(
      "setpoint_raw/global", qos,
      [this](mavros_msgs::msg::GlobalPositionTarget::SharedPtr msg) {send_global(*msg);});
    attitude_sub_ = core.create_subscription<mavros_msgs::msg::AttitudeTarget>(
      "setpoint_raw/attitude", qos,
      [this](mavros_msgs::msg::AttitudeTarget::SharedPtr msg) {send_attitude(*msg);});
  }

  void handle_message(const mavlink_message_t & message) override
  {
    if (message.msgid == MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED) {
      mavlink_position_target_local_ned_t value{};
      mavlink_msg_position_target_local_ned_decode(&message, &value);
      auto position = ftf::transform_frame_ned_enu(Eigen::Vector3d(value.x, value.y, value.z));
      auto velocity = ftf::transform_frame_ned_enu(Eigen::Vector3d(value.vx, value.vy, value.vz));
      auto acceleration = ftf::transform_frame_ned_enu(
        Eigen::Vector3d(value.afx, value.afy, value.afz));
      mavros_msgs::msg::PositionTarget output;
      output.header.stamp = core_.stamp();
      output.coordinate_frame = value.coordinate_frame;
      output.type_mask = value.type_mask;
      output.position = tf2::toMsg(position);
      tf2::toMsg(velocity, output.velocity);
      tf2::toMsg(acceleration, output.acceleration_or_force);
      output.yaw = transform_yaw(value.yaw);
      output.yaw_rate = -value.yaw_rate;
      local_pub_->publish(output);
    } else if (message.msgid == MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT) {
      mavlink_position_target_global_int_t value{};
      mavlink_msg_position_target_global_int_decode(&message, &value);
      mavros_msgs::msg::GlobalPositionTarget output;
      output.header.stamp = core_.stamp();
      output.coordinate_frame = value.coordinate_frame;
      output.type_mask = value.type_mask;
      output.latitude = value.lat_int / 1e7;
      output.longitude = value.lon_int / 1e7;
      output.altitude = value.alt;
      tf2::toMsg(ftf::transform_frame_ned_enu(Eigen::Vector3d(value.vx, value.vy, value.vz)),
        output.velocity);
      tf2::toMsg(ftf::transform_frame_ned_enu(Eigen::Vector3d(value.afx, value.afy, value.afz)),
        output.acceleration_or_force);
      output.yaw = transform_yaw(value.yaw);
      output.yaw_rate = -value.yaw_rate;
      global_pub_->publish(output);
    } else if (message.msgid == MAVLINK_MSG_ID_ATTITUDE_TARGET) {
      mavlink_attitude_target_t value{};
      mavlink_msg_attitude_target_decode(&message, &value);
      std::array<float, 4> q{value.q[0], value.q[1], value.q[2], value.q[3]};
      const auto orientation = ftf::transform_orientation_ned_enu(
        ftf::transform_orientation_baselink_aircraft(ftf::mavlink_to_quaternion(q)));
      mavros_msgs::msg::AttitudeTarget output;
      output.header.stamp = core_.stamp();
      output.type_mask = value.type_mask;
      output.orientation = tf2::toMsg(orientation);
      tf2::toMsg(ftf::transform_frame_baselink_aircraft(Eigen::Vector3d(
          value.body_roll_rate, value.body_pitch_rate, value.body_yaw_rate)), output.body_rate);
      output.thrust = value.thrust;
      attitude_pub_->publish(output);
    }
  }

private:
  static float transform_yaw(float yaw)
  {
    return ftf::quaternion_get_yaw(ftf::transform_orientation_aircraft_baselink(
      ftf::transform_orientation_ned_enu(ftf::quaternion_from_rpy(0.0, 0.0, yaw))));
  }

  uint32_t boot_ms(const builtin_interfaces::msg::Time & stamp) const
  {
    if (stamp.sec == 0 && stamp.nanosec == 0) {
      return static_cast<uint32_t>(core_.stamp().nanoseconds() / 1000000ULL);
    }
    return static_cast<uint32_t>(rclcpp::Time(stamp).nanoseconds() / 1000000ULL);
  }

  void send_local(const mavros_msgs::msg::PositionTarget & input)
  {
    Eigen::Vector3d position = ftf::to_eigen(input.position);
    Eigen::Vector3d velocity = ftf::to_eigen(input.velocity);
    Eigen::Vector3d acceleration = ftf::to_eigen(input.acceleration_or_force);
    float yaw{};
    if (input.coordinate_frame == input.FRAME_BODY_NED ||
      input.coordinate_frame == input.FRAME_BODY_OFFSET_NED)
    {
      position = ftf::transform_frame_baselink_aircraft(position);
      velocity = ftf::transform_frame_baselink_aircraft(velocity);
      acceleration = ftf::transform_frame_baselink_aircraft(acceleration);
      yaw = ftf::quaternion_get_yaw(ftf::transform_orientation_absolute_frame_aircraft_baselink(
        ftf::quaternion_from_rpy(0.0, 0.0, input.yaw)));
    } else {
      position = ftf::transform_frame_enu_ned(position);
      velocity = ftf::transform_frame_enu_ned(velocity);
      acceleration = ftf::transform_frame_enu_ned(acceleration);
      yaw = transform_yaw(input.yaw);
    }
    mavlink_message_t message{};
    mavlink_msg_set_position_target_local_ned_pack(
      core_.system_id(), core_.component_id(), &message, boot_ms(input.header.stamp),
      core_.target_system(), core_.target_component(), input.coordinate_frame, input.type_mask,
      position.x(), position.y(), position.z(), velocity.x(), velocity.y(), velocity.z(),
      acceleration.x(), acceleration.y(), acceleration.z(), yaw, -input.yaw_rate);
    core_.send_message(message);
  }

  void send_global(const mavros_msgs::msg::GlobalPositionTarget & input)
  {
    const auto velocity = ftf::transform_frame_enu_ned(ftf::to_eigen(input.velocity));
    const auto acceleration = ftf::transform_frame_enu_ned(
      ftf::to_eigen(input.acceleration_or_force));
    mavlink_message_t message{};
    mavlink_msg_set_position_target_global_int_pack(
      core_.system_id(), core_.component_id(), &message, boot_ms(input.header.stamp),
      core_.target_system(), core_.target_component(), input.coordinate_frame, input.type_mask,
      static_cast<int32_t>(input.latitude * 1e7), static_cast<int32_t>(input.longitude * 1e7),
      input.altitude, velocity.x(), velocity.y(), velocity.z(), acceleration.x(), acceleration.y(),
      acceleration.z(), transform_yaw(input.yaw), -input.yaw_rate);
    core_.send_message(message);
  }

  void send_attitude(const mavros_msgs::msg::AttitudeTarget & input)
  {
    const auto orientation = ftf::transform_orientation_enu_ned(
      ftf::transform_orientation_baselink_aircraft(ftf::to_eigen(input.orientation)));
    const auto rate = ftf::transform_frame_baselink_aircraft(ftf::to_eigen(input.body_rate));
    const float q[4] = {static_cast<float>(orientation.w()), static_cast<float>(orientation.x()),
      static_cast<float>(orientation.y()), static_cast<float>(orientation.z())};
    const float thrust_body[3] = {0.0F, 0.0F, 0.0F};
    const float thrust = std::clamp(static_cast<float>(input.thrust * thrust_scaling_), 0.0F, 1.0F);
    mavlink_message_t message{};
    mavlink_msg_set_attitude_target_pack(
      core_.system_id(), core_.component_id(), &message, boot_ms(input.header.stamp),
      core_.target_system(), core_.target_component(), input.type_mask, q, rate.x(), rate.y(),
      rate.z(), thrust, thrust_body);
    core_.send_message(message);
  }

  double thrust_scaling_{};
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr local_pub_;
  rclcpp::Publisher<mavros_msgs::msg::GlobalPositionTarget>::SharedPtr global_pub_;
  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr attitude_pub_;
  rclcpp::Subscription<mavros_msgs::msg::PositionTarget>::SharedPtr local_sub_;
  rclcpp::Subscription<mavros_msgs::msg::GlobalPositionTarget>::SharedPtr global_sub_;
  rclcpp::Subscription<mavros_msgs::msg::AttitudeTarget>::SharedPtr attitude_sub_;
};

std::unique_ptr<Module> make_setpoint_raw(MavrosLite & core)
{
  return std::make_unique<SetpointRaw>(core);
}
}  // namespace fss_px4_sim::mavros_lite
