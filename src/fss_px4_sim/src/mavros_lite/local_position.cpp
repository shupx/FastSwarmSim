#include "fss_px4_sim/mavros_lite/core.hpp"

#include <mavros/frame_tf.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace fss_px4_sim::mavros_lite
{
namespace ftf = mavros::ftf;

class LocalPosition final : public Module
{
public:
  explicit LocalPosition(MavrosLite & core)
  : Module(core)
  {
    frame_id_ = core.declare_parameter<std::string>("local_position.frame_id", "map");
    child_frame_id_ = core.declare_parameter<std::string>(
      "local_position.child_frame_id", "base_link");
    const auto qos = rclcpp::SensorDataQoS();
    pose_pub_ = core.create_publisher<geometry_msgs::msg::PoseStamped>("local_position/pose", qos);
    velocity_local_pub_ = core.create_publisher<geometry_msgs::msg::TwistStamped>(
      "local_position/velocity_local", qos);
    velocity_body_pub_ = core.create_publisher<geometry_msgs::msg::TwistStamped>(
      "local_position/velocity_body", qos);
    odom_pub_ = core.create_publisher<nav_msgs::msg::Odometry>("local_position/odom", qos);
  }

  void handle_message(const mavlink_message_t & message) override
  {
    if (message.msgid != MAVLINK_MSG_ID_LOCAL_POSITION_NED) {
      return;
    }
    mavlink_local_position_ned_t value{};
    mavlink_msg_local_position_ned_decode(&message, &value);
    const auto position = ftf::transform_frame_ned_enu(Eigen::Vector3d(value.x, value.y, value.z));
    const auto velocity = ftf::transform_frame_ned_enu(Eigen::Vector3d(value.vx, value.vy, value.vz));
    geometry_msgs::msg::Quaternion orientation;
    geometry_msgs::msg::Vector3 angular;
    orientation.w = 1.0;
    {
      std::lock_guard<std::mutex> lock(core_.state().mutex);
      if (core_.state().have_attitude) {
        orientation = core_.state().orientation;
        angular = core_.state().angular_velocity;
      }
    }
    const auto orientation_eigen = ftf::to_eigen(orientation);
    const auto body_velocity = ftf::transform_frame_enu_baselink(velocity, orientation_eigen.inverse());
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = core_.stamp();
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;
    odom.pose.pose.position = tf2::toMsg(position);
    odom.pose.pose.orientation = orientation;
    tf2::toMsg(body_velocity, odom.twist.twist.linear);
    odom.twist.twist.angular = angular;
    odom_pub_->publish(odom);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = odom.header;
    pose.pose = odom.pose.pose;
    pose_pub_->publish(pose);
    geometry_msgs::msg::TwistStamped body;
    body.header.stamp = odom.header.stamp;
    body.header.frame_id = child_frame_id_;
    body.twist = odom.twist.twist;
    velocity_body_pub_->publish(body);
    geometry_msgs::msg::TwistStamped local;
    local.header.stamp = odom.header.stamp;
    local.header.frame_id = child_frame_id_;
    tf2::toMsg(velocity, local.twist.linear);
    tf2::toMsg(ftf::transform_frame_baselink_enu(ftf::to_eigen(angular), orientation_eigen),
      local.twist.angular);
    velocity_local_pub_->publish(local);
  }

private:
  std::string frame_id_;
  std::string child_frame_id_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_local_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_body_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

std::unique_ptr<Module> make_local_position(MavrosLite & core)
{
  return std::make_unique<LocalPosition>(core);
}
}  // namespace fss_px4_sim::mavros_lite
