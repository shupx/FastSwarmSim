#include "fss_px4_sim/mavros_sim/plugin.hpp"

#include <algorithm>
#include <cmath>

#include "mavros/frame_tf.h"
#include "mavros_msgs/msg/attitude_target.hpp"
#include "mavros_msgs/msg/global_position_target.hpp"
#include "mavros_msgs/msg/position_target.hpp"

namespace fss_px4_sim::mavros_sim
{
namespace
{
constexpr uint8_t kSystemId = 1;
constexpr uint8_t kComponentId = 1;

uint32_t stamp_ms(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<uint32_t>(static_cast<uint64_t>(stamp.sec) * 1000ULL + stamp.nanosec / 1000000ULL);
}

Eigen::Quaterniond to_eigen(const geometry_msgs::msg::Quaternion & q) {return {q.w, q.x, q.y, q.z};}
geometry_msgs::msg::Quaternion to_msg(const Eigen::Quaterniond & q)
{geometry_msgs::msg::Quaternion out; out.w=q.w(); out.x=q.x(); out.y=q.y(); out.z=q.z(); return out;}
geometry_msgs::msg::Point to_point(const Eigen::Vector3d & v)
{geometry_msgs::msg::Point out; out.x=v.x(); out.y=v.y(); out.z=v.z(); return out;}
geometry_msgs::msg::Vector3 to_vector(const Eigen::Vector3d & v)
{geometry_msgs::msg::Vector3 out; out.x=v.x(); out.y=v.y(); out.z=v.z(); return out;}
Eigen::Vector3d to_eigen(const geometry_msgs::msg::Point & p) {return {p.x, p.y, p.z};}
Eigen::Vector3d to_eigen(const geometry_msgs::msg::Vector3 & v) {return {v.x, v.y, v.z};}
float yaw_ned_to_enu(float yaw) {return static_cast<float>(M_PI_2 - yaw);}
float yaw_enu_to_ned(float yaw) {return static_cast<float>(M_PI_2 - yaw);}
}  // namespace

class SetpointRawPlugin final : public Plugin
{
public:
  using Plugin::Plugin;
  SetpointRawPlugin(rclcpp::Node & node, MavlinkIngress ingress, std::shared_ptr<SharedState> state)
  : Plugin(node, std::move(ingress), std::move(state))
  {
    const auto qos = rclcpp::SensorDataQoS();
    local_sub_ = node_.create_subscription<mavros_msgs::msg::PositionTarget>("mavros/setpoint_raw/local", qos,
      [this](mavros_msgs::msg::PositionTarget::SharedPtr message) {local_cb(*message);});
    global_sub_ = node_.create_subscription<mavros_msgs::msg::GlobalPositionTarget>("mavros/setpoint_raw/global", qos,
      [this](mavros_msgs::msg::GlobalPositionTarget::SharedPtr message) {global_cb(*message);});
    attitude_sub_ = node_.create_subscription<mavros_msgs::msg::AttitudeTarget>("mavros/setpoint_raw/attitude", qos,
      [this](mavros_msgs::msg::AttitudeTarget::SharedPtr message) {attitude_cb(*message);});
    target_local_pub_ = node_.create_publisher<mavros_msgs::msg::PositionTarget>("mavros/setpoint_raw/target_local", qos);
    target_global_pub_ = node_.create_publisher<mavros_msgs::msg::GlobalPositionTarget>("mavros/setpoint_raw/target_global", qos);
    target_attitude_pub_ = node_.create_publisher<mavros_msgs::msg::AttitudeTarget>("mavros/setpoint_raw/target_attitude", qos);
    thrust_scaling_ = parameter("setpoint_raw.thrust_scaling", 1.0);
  }

  bool handles(uint32_t id) const override
  {return id == MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED || id == MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT || id == MAVLINK_MSG_ID_ATTITUDE_TARGET;}

  void handle_message(const mavlink_message_t & message, const rclcpp::Time & stamp) override
  {
    if (message.msgid == MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED) {
      mavlink_position_target_local_ned_t data{}; mavlink_msg_position_target_local_ned_decode(&message, &data);
      mavros_msgs::msg::PositionTarget out; out.header.stamp=stamp; out.coordinate_frame=data.coordinate_frame; out.type_mask=data.type_mask;
      out.position=to_point(mavros::ftf::transform_frame_ned_enu({data.x,data.y,data.z}));
      out.velocity=to_vector(mavros::ftf::transform_frame_ned_enu({data.vx,data.vy,data.vz}));
      out.acceleration_or_force=to_vector(mavros::ftf::transform_frame_ned_enu({data.afx,data.afy,data.afz}));
      out.yaw=yaw_ned_to_enu(data.yaw); out.yaw_rate=-data.yaw_rate; target_local_pub_->publish(out);
    } else if (message.msgid == MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT) {
      mavlink_position_target_global_int_t data{}; mavlink_msg_position_target_global_int_decode(&message, &data);
      mavros_msgs::msg::GlobalPositionTarget out; out.header.stamp=stamp; out.coordinate_frame=data.coordinate_frame; out.type_mask=data.type_mask;
      out.latitude=data.lat_int / 1e7; out.longitude=data.lon_int / 1e7; out.altitude=data.alt;
      out.velocity=to_vector(mavros::ftf::transform_frame_ned_enu({data.vx,data.vy,data.vz}));
      out.acceleration_or_force=to_vector(mavros::ftf::transform_frame_ned_enu({data.afx,data.afy,data.afz}));
      out.yaw=yaw_ned_to_enu(data.yaw); out.yaw_rate=-data.yaw_rate; target_global_pub_->publish(out);
    } else {
      mavlink_attitude_target_t data{}; mavlink_msg_attitude_target_decode(&message, &data);
      mavros_msgs::msg::AttitudeTarget out; out.header.stamp=stamp; out.type_mask=data.type_mask;
      const Eigen::Quaterniond q(data.q[0],data.q[1],data.q[2],data.q[3]);
      out.orientation=to_msg(mavros::ftf::transform_orientation_baselink_aircraft(mavros::ftf::transform_orientation_enu_ned(q)));
      out.body_rate=to_vector(mavros::ftf::transform_frame_aircraft_baselink({data.body_roll_rate,data.body_pitch_rate,data.body_yaw_rate}));
      out.thrust=data.thrust; target_attitude_pub_->publish(out);
    }
  }

private:
  void send(px4::mavlink_receive_handle handle, const mavlink_message_t & message) {ingress_(handle, message);}
  void local_cb(const mavros_msgs::msg::PositionTarget & req)
  {
    auto p=to_eigen(req.position), v=to_eigen(req.velocity), a=to_eigen(req.acceleration_or_force);
    float yaw{};
    if (req.coordinate_frame == mavros_msgs::msg::PositionTarget::FRAME_BODY_NED || req.coordinate_frame == mavros_msgs::msg::PositionTarget::FRAME_BODY_OFFSET_NED) {
      p=mavros::ftf::transform_frame_baselink_aircraft(p); v=mavros::ftf::transform_frame_baselink_aircraft(v); a=mavros::ftf::transform_frame_baselink_aircraft(a); yaw=-req.yaw;
    } else {p=mavros::ftf::transform_frame_enu_ned(p); v=mavros::ftf::transform_frame_enu_ned(v); a=mavros::ftf::transform_frame_enu_ned(a); yaw=yaw_enu_to_ned(req.yaw);}
    mavlink_message_t message{};
    mavlink_msg_set_position_target_local_ned_pack(kSystemId,kComponentId,&message,stamp_ms(req.header.stamp),kSystemId,kComponentId,req.coordinate_frame,req.type_mask,p.x(),p.y(),p.z(),v.x(),v.y(),v.z(),a.x(),a.y(),a.z(),yaw,-req.yaw_rate);
    send(px4::mavlink_receive_handle::SET_POSITION_TARGET_LOCAL_NED,message);
  }
  void global_cb(const mavros_msgs::msg::GlobalPositionTarget & req)
  {
    const auto v=mavros::ftf::transform_frame_enu_ned(to_eigen(req.velocity)); const auto a=mavros::ftf::transform_frame_enu_ned(to_eigen(req.acceleration_or_force));
    mavlink_message_t message{};
    mavlink_msg_set_position_target_global_int_pack(kSystemId,kComponentId,&message,stamp_ms(req.header.stamp),kSystemId,kComponentId,req.coordinate_frame,req.type_mask,static_cast<int32_t>(std::llround(req.latitude*1e7)),static_cast<int32_t>(std::llround(req.longitude*1e7)),req.altitude,v.x(),v.y(),v.z(),a.x(),a.y(),a.z(),yaw_enu_to_ned(req.yaw),-req.yaw_rate);
    send(px4::mavlink_receive_handle::SET_POSITION_TARGET_GLOBAL_INT,message);
  }
  void attitude_cb(const mavros_msgs::msg::AttitudeTarget & req)
  {
    const float thrust=static_cast<float>(std::clamp(req.thrust * thrust_scaling_, 0.0, 1.0));
    const auto q=mavros::ftf::transform_orientation_enu_ned(mavros::ftf::transform_orientation_baselink_aircraft(to_eigen(req.orientation)));
    const auto r=mavros::ftf::transform_frame_baselink_aircraft(to_eigen(req.body_rate)); float qa[4]{static_cast<float>(q.w()),static_cast<float>(q.x()),static_cast<float>(q.y()),static_cast<float>(q.z())}; float thrust_body[3]{};
    mavlink_message_t message{}; mavlink_msg_set_attitude_target_pack(kSystemId,kComponentId,&message,stamp_ms(req.header.stamp),kSystemId,kComponentId,req.type_mask,qa,r.x(),r.y(),r.z(),thrust,thrust_body);
    send(px4::mavlink_receive_handle::SET_ATTITUDE_TARGET,message);
  }
  double thrust_scaling_{};
  rclcpp::Subscription<mavros_msgs::msg::PositionTarget>::SharedPtr local_sub_;
  rclcpp::Subscription<mavros_msgs::msg::GlobalPositionTarget>::SharedPtr global_sub_;
  rclcpp::Subscription<mavros_msgs::msg::AttitudeTarget>::SharedPtr attitude_sub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr target_local_pub_;
  rclcpp::Publisher<mavros_msgs::msg::GlobalPositionTarget>::SharedPtr target_global_pub_;
  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr target_attitude_pub_;
};

std::unique_ptr<Plugin> make_setpoint_raw_plugin(rclcpp::Node & node, MavlinkIngress ingress, std::shared_ptr<SharedState> state)
{return std::make_unique<SetpointRawPlugin>(node, std::move(ingress), std::move(state));}

}  // namespace fss_px4_sim::mavros_sim
