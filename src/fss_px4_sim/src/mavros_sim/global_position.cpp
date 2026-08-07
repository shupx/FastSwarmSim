#include "fss_px4_sim/mavros_sim/plugin.hpp"

#include <cmath>
#include <limits>

#include "geographic_msgs/msg/geo_point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mavros/frame_tf.h"
#include "mavros_msgs/msg/home_position.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int32.hpp"

namespace fss_px4_sim::mavros_sim
{
namespace { constexpr uint8_t kSystemId=1,kComponentId=1; geometry_msgs::msg::Quaternion quat(const Eigen::Quaterniond & q){geometry_msgs::msg::Quaternion r;r.w=q.w();r.x=q.x();r.y=q.y();r.z=q.z();return r;} }
class GlobalPositionPlugin final : public Plugin
{
public:
  GlobalPositionPlugin(rclcpp::Node & node,MavlinkIngress ingress,std::shared_ptr<SharedState> state):Plugin(node,std::move(ingress),std::move(state))
  {
    const auto qos=rclcpp::SensorDataQoS(); const auto latched=rclcpp::QoS(1).reliable().transient_local(); frame_id_=parameter("global_position.frame_id",std::string("map")); child_frame_id_=parameter("global_position.tf.child_frame_id",std::string("base_link")); global_frame_id_=parameter("global_position.tf.global_frame_id",std::string("earth"));
    raw_fix_pub_=node_.create_publisher<sensor_msgs::msg::NavSatFix>("mavros/global_position/raw/fix",qos); raw_vel_pub_=node_.create_publisher<geometry_msgs::msg::TwistStamped>("mavros/global_position/raw/gps_vel",qos); raw_sat_pub_=node_.create_publisher<std_msgs::msg::UInt32>("mavros/global_position/raw/satellites",qos);
    global_pub_=node_.create_publisher<sensor_msgs::msg::NavSatFix>("mavros/global_position/global",qos); local_pub_=node_.create_publisher<nav_msgs::msg::Odometry>("mavros/global_position/local",qos); rel_alt_pub_=node_.create_publisher<std_msgs::msg::Float64>("mavros/global_position/rel_alt",qos); heading_pub_=node_.create_publisher<std_msgs::msg::Float64>("mavros/global_position/compass_hdg",qos); origin_pub_=node_.create_publisher<geographic_msgs::msg::GeoPointStamped>("mavros/global_position/gp_origin",latched); offset_pub_=node_.create_publisher<geometry_msgs::msg::PoseStamped>("mavros/global_position/gp_lp_offset",qos);
    origin_sub_=node_.create_subscription<geographic_msgs::msg::GeoPointStamped>("mavros/global_position/set_gp_origin",qos,[this](geographic_msgs::msg::GeoPointStamped::SharedPtr msg){set_origin(*msg);}); home_sub_=node_.create_subscription<mavros_msgs::msg::HomePosition>("mavros/home_position/home",qos,[this](mavros_msgs::msg::HomePosition::SharedPtr msg){home_=msg->geo;});
  }
  bool handles(uint32_t id) const override {return id==MAVLINK_MSG_ID_GPS_RAW_INT||id==MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN||id==MAVLINK_MSG_ID_GLOBAL_POSITION_INT||id==MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET;}
  void handle_message(const mavlink_message_t & message,const rclcpp::Time & stamp) override
  {
    if(message.msgid==MAVLINK_MSG_ID_GPS_RAW_INT){mavlink_gps_raw_int_t d{};mavlink_msg_gps_raw_int_decode(&message,&d);sensor_msgs::msg::NavSatFix fix;fix.header.stamp=stamp;fix.header.frame_id=frame_id_;fix.latitude=d.lat/1e7;fix.longitude=d.lon/1e7;fix.altitude=d.alt/1000.0;fix.status.service=sensor_msgs::msg::NavSatStatus::SERVICE_GPS;fix.status.status=d.fix_type>=3?sensor_msgs::msg::NavSatStatus::STATUS_FIX:sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;fix.position_covariance[0]=fix.position_covariance[4]=std::pow(d.eph/100.0,2);fix.position_covariance[8]=std::pow(d.epv/100.0,2);fix.position_covariance_type=sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;raw_fix_pub_->publish(fix);std_msgs::msg::UInt32 sats;sats.data=d.satellites_visible;raw_sat_pub_->publish(sats);if(d.vel!=UINT16_MAX&&d.cog!=UINT16_MAX){geometry_msgs::msg::TwistStamped vel;vel.header=fix.header;const double speed=d.vel/100.0,course=d.cog/100.0*M_PI/180.0;vel.twist.linear.x=speed*std::sin(course);vel.twist.linear.y=speed*std::cos(course);raw_vel_pub_->publish(vel);}}
    else if(message.msgid==MAVLINK_MSG_ID_GLOBAL_POSITION_INT){mavlink_global_position_int_t d{};mavlink_msg_global_position_int_decode(&message,&d);sensor_msgs::msg::NavSatFix fix;fix.header.stamp=stamp;fix.header.frame_id=frame_id_;fix.latitude=d.lat/1e7;fix.longitude=d.lon/1e7;fix.altitude=d.alt/1000.0;fix.status.service=sensor_msgs::msg::NavSatStatus::SERVICE_GPS;fix.status.status=sensor_msgs::msg::NavSatStatus::STATUS_FIX;global_pub_->publish(fix);std_msgs::msg::Float64 rel;rel.data=d.relative_alt/1000.0;rel_alt_pub_->publish(rel);std_msgs::msg::Float64 hdg;hdg.data=d.hdg==UINT16_MAX?std::numeric_limits<double>::quiet_NaN():d.hdg/100.0;heading_pub_->publish(hdg);nav_msgs::msg::Odometry odom;odom.header=fix.header;odom.child_frame_id=child_frame_id_;odom.twist.twist.linear.x=d.vy/100.0;odom.twist.twist.linear.y=d.vx/100.0;odom.twist.twist.linear.z=d.vz/100.0;odom.pose.pose.orientation=quat(state_->attitude_enu);odom.pose.pose.position.z=rel.data;local_pub_->publish(odom);}
    else if(message.msgid==MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN){mavlink_gps_global_origin_t d{};mavlink_msg_gps_global_origin_decode(&message,&d);geographic_msgs::msg::GeoPointStamped out;out.header.stamp=stamp;out.header.frame_id=global_frame_id_;out.position.latitude=d.latitude/1e7;out.position.longitude=d.longitude/1e7;out.position.altitude=d.altitude/1000.0;origin_pub_->publish(out);}
    else {mavlink_local_position_ned_system_global_offset_t d{};mavlink_msg_local_position_ned_system_global_offset_decode(&message,&d);geometry_msgs::msg::PoseStamped out;out.header.stamp=stamp;out.header.frame_id=global_frame_id_;const auto p=mavros::ftf::transform_frame_ned_enu({d.x,d.y,d.z});out.pose.position.x=p.x();out.pose.position.y=p.y();out.pose.position.z=p.z();const Eigen::Quaterniond yaw(Eigen::AngleAxisd(d.yaw,Eigen::Vector3d::UnitZ()));const auto q=mavros::ftf::transform_orientation_baselink_aircraft(mavros::ftf::transform_orientation_enu_ned(yaw));out.pose.orientation=quat(q);offset_pub_->publish(out);}
  }
private:
  void set_origin(const geographic_msgs::msg::GeoPointStamped & req){mavlink_message_t m{};mavlink_msg_set_gps_global_origin_pack(kSystemId,kComponentId,&m,kSystemId,static_cast<int32_t>(std::llround(req.position.latitude*1e7)),static_cast<int32_t>(std::llround(req.position.longitude*1e7)),static_cast<int32_t>(std::llround(req.position.altitude*1000.0)),0);ingress_(px4::mavlink_receive_handle::SET_GPS_GLOBAL_ORIGIN,m);}
  std::string frame_id_,child_frame_id_,global_frame_id_; geographic_msgs::msg::GeoPoint home_{};
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr raw_fix_pub_,global_pub_;rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr raw_vel_pub_;rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr raw_sat_pub_;rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr local_pub_;rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rel_alt_pub_,heading_pub_;rclcpp::Publisher<geographic_msgs::msg::GeoPointStamped>::SharedPtr origin_pub_;rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr offset_pub_;rclcpp::Subscription<geographic_msgs::msg::GeoPointStamped>::SharedPtr origin_sub_;rclcpp::Subscription<mavros_msgs::msg::HomePosition>::SharedPtr home_sub_;
};
std::unique_ptr<Plugin> make_global_position_plugin(rclcpp::Node & node,MavlinkIngress ingress,std::shared_ptr<SharedState> state){return std::make_unique<GlobalPositionPlugin>(node,std::move(ingress),std::move(state));}
}  // namespace fss_px4_sim::mavros_sim
