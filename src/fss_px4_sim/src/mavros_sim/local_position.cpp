#include "fss_px4_sim/mavros_sim/plugin.hpp"

#include "mavros/frame_tf.h"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace fss_px4_sim::mavros_sim
{
namespace
{
geometry_msgs::msg::Point point(const Eigen::Vector3d & v) {geometry_msgs::msg::Point r; r.x=v.x(); r.y=v.y(); r.z=v.z(); return r;}
geometry_msgs::msg::Vector3 vector(const Eigen::Vector3d & v) {geometry_msgs::msg::Vector3 r; r.x=v.x(); r.y=v.y(); r.z=v.z(); return r;}
geometry_msgs::msg::Quaternion quaternion(const Eigen::Quaterniond & q) {geometry_msgs::msg::Quaternion r; r.w=q.w(); r.x=q.x(); r.y=q.y(); r.z=q.z(); return r;}
}

class LocalPositionPlugin final : public Plugin
{
public:
  using Plugin::Plugin;
  LocalPositionPlugin(rclcpp::Node & node, MavlinkIngress ingress, std::shared_ptr<SharedState> state)
  : Plugin(node, std::move(ingress), std::move(state))
  {
    const auto qos=rclcpp::SensorDataQoS();
    pose_pub_=node_.create_publisher<geometry_msgs::msg::PoseStamped>("mavros/local_position/pose",qos);
    pose_cov_pub_=node_.create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("mavros/local_position/pose_cov",qos);
    velocity_local_pub_=node_.create_publisher<geometry_msgs::msg::TwistStamped>("mavros/local_position/velocity_local",qos);
    velocity_body_pub_=node_.create_publisher<geometry_msgs::msg::TwistStamped>("mavros/local_position/velocity_body",qos);
    velocity_cov_pub_=node_.create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("mavros/local_position/velocity_body_cov",qos);
    accel_pub_=node_.create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>("mavros/local_position/accel",qos);
    odom_pub_=node_.create_publisher<nav_msgs::msg::Odometry>("mavros/local_position/odom",qos);
    frame_id_=parameter("local_position.frame_id",std::string("map"));
    child_frame_id_=parameter("local_position.tf.child_frame_id",std::string("base_link"));
  }
  bool handles(uint32_t id) const override {return id==MAVLINK_MSG_ID_LOCAL_POSITION_NED || id==MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV;}
  void handle_message(const mavlink_message_t & message,const rclcpp::Time & stamp) override
  {
    if (message.msgid==MAVLINK_MSG_ID_LOCAL_POSITION_NED) {
      mavlink_local_position_ned_t d{}; mavlink_msg_local_position_ned_decode(&message,&d); has_local_=true;
      auto odom=make_odom({d.x,d.y,d.z},{d.vx,d.vy,d.vz},stamp);
      if (!has_cov_) odom_pub_->publish(odom);
      publish_uncov(odom);
    } else {
      mavlink_local_position_ned_cov_t d{}; mavlink_msg_local_position_ned_cov_decode(&message,&d); has_cov_=true;
      auto odom=make_odom({d.x,d.y,d.z},{d.vx,d.vy,d.vz},stamp);
      odom.pose.covariance[0]=d.covariance[0]; odom.pose.covariance[7]=d.covariance[9]; odom.pose.covariance[14]=d.covariance[17];
      odom.twist.covariance[0]=d.covariance[24]; odom.twist.covariance[7]=d.covariance[30]; odom.twist.covariance[14]=d.covariance[35];
      odom_pub_->publish(odom);
      geometry_msgs::msg::PoseWithCovarianceStamped pose_cov; pose_cov.header=odom.header; pose_cov.pose=odom.pose; pose_cov_pub_->publish(pose_cov);
      geometry_msgs::msg::TwistWithCovarianceStamped velocity_cov; velocity_cov.header.stamp=stamp; velocity_cov.header.frame_id=child_frame_id_; velocity_cov.twist=odom.twist; velocity_cov_pub_->publish(velocity_cov);
      geometry_msgs::msg::AccelWithCovarianceStamped accel; accel.header=odom.header; accel.accel.accel.linear=vector(mavros::ftf::transform_frame_ned_enu({d.ax,d.ay,d.az}));
      accel.accel.covariance[0]=d.covariance[39]; accel.accel.covariance[7]=d.covariance[42]; accel.accel.covariance[14]=d.covariance[44]; accel_pub_->publish(accel);
      if (!has_local_) publish_uncov(odom);
    }
  }
private:
  nav_msgs::msg::Odometry make_odom(const Eigen::Vector3d & p_ned,const Eigen::Vector3d & v_ned,const rclcpp::Time & stamp) const
  {
    const auto p=mavros::ftf::transform_frame_ned_enu(p_ned), v=mavros::ftf::transform_frame_ned_enu(v_ned);
    nav_msgs::msg::Odometry out; out.header.stamp=stamp; out.header.frame_id=frame_id_; out.child_frame_id=child_frame_id_; out.pose.pose.position=point(p); out.pose.pose.orientation=quaternion(state_->attitude_enu);
    const auto body=state_->attitude_enu.inverse()*v; out.twist.twist.linear=vector(body); out.twist.twist.angular=vector(state_->body_omega_flu); return out;
  }
  void publish_uncov(const nav_msgs::msg::Odometry & odom)
  {
    geometry_msgs::msg::PoseStamped pose; pose.header=odom.header; pose.pose=odom.pose.pose; pose_pub_->publish(pose);
    geometry_msgs::msg::TwistStamped body; body.header.stamp=odom.header.stamp; body.header.frame_id=child_frame_id_; body.twist=odom.twist.twist; velocity_body_pub_->publish(body);
    geometry_msgs::msg::TwistStamped local=body; local.header.frame_id=frame_id_; local.twist.linear=vector(state_->attitude_enu*Eigen::Vector3d(body.twist.linear.x,body.twist.linear.y,body.twist.linear.z)); velocity_local_pub_->publish(local);
  }
  std::string frame_id_,child_frame_id_; bool has_local_{false},has_cov_{false};
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_local_pub_,velocity_body_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_cov_pub_;
  rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr accel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};
std::unique_ptr<Plugin> make_local_position_plugin(rclcpp::Node & node,MavlinkIngress ingress,std::shared_ptr<SharedState> state)
{return std::make_unique<LocalPositionPlugin>(node,std::move(ingress),std::move(state));}
}  // namespace fss_px4_sim::mavros_sim
