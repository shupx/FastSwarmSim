#include "fss_px4_sim/mavros_sim/plugins.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <string>

#include <Eigen/Geometry>

#include "fss_px4_sim/px4_sitl.hpp"
#include "geographic_msgs/msg/geo_point_stamped.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "mavros_msgs/msg/attitude_target.hpp"
#include "mavros_msgs/msg/extended_state.hpp"
#include "mavros_msgs/msg/global_position_target.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/command_int.hpp"
#include "mavros_msgs/srv/command_long.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int32.hpp"

namespace fss_px4_sim::mavros_sim
{
namespace
{
constexpr uint8_t kSystemId = 1;
constexpr uint8_t kComponentId = 1;

Eigen::Vector3d enu_to_ned(const geometry_msgs::msg::Vector3 & v) {return {v.y, v.x, -v.z};}
Eigen::Vector3d enu_to_ned(const geometry_msgs::msg::Point & v) {return {v.y, v.x, -v.z};}
geometry_msgs::msg::Vector3 ned_to_enu(const Eigen::Vector3d & v)
{geometry_msgs::msg::Vector3 out; out.x = v.y(); out.y = v.x(); out.z = -v.z(); return out;}
geometry_msgs::msg::Point ned_to_enu_point(const Eigen::Vector3d & v)
{geometry_msgs::msg::Point out; out.x = v.y(); out.y = v.x(); out.z = -v.z(); return out;}
Eigen::Quaterniond to_eigen(const geometry_msgs::msg::Quaternion & q) {return {q.w, q.x, q.y, q.z};}
geometry_msgs::msg::Quaternion to_msg(const Eigen::Quaterniond & q)
{geometry_msgs::msg::Quaternion out; out.w = q.w(); out.x = q.x(); out.y = q.y(); out.z = q.z(); return out;}
Eigen::Quaterniond enu_flu_to_ned_frd(const Eigen::Quaterniond & q)
{return mavros::ftf::transform_orientation_enu_ned(mavros::ftf::transform_orientation_baselink_aircraft(q));}
Eigen::Quaterniond ned_frd_to_enu_flu(const Eigen::Quaterniond & q)
{return mavros::ftf::transform_orientation_baselink_aircraft(mavros::ftf::transform_orientation_enu_ned(q));}
float yaw_enu_to_ned(float yaw) {return static_cast<float>(M_PI_2 - yaw);}
float yaw_ned_to_enu(float yaw) {return static_cast<float>(M_PI_2 - yaw);}
uint32_t stamp_ms(const builtin_interfaces::msg::Time & stamp)
{return static_cast<uint32_t>(static_cast<uint64_t>(stamp.sec) * 1000ULL + stamp.nanosec / 1000000ULL);}
std::string px4_mode(uint32_t mode)
{
  const auto main = static_cast<uint8_t>((mode >> 16U) & 0xffU);
  const auto sub = static_cast<uint8_t>((mode >> 24U) & 0xffU);
  if (main == 1) return "MANUAL"; if (main == 2) return "ALTCTL"; if (main == 3) return "POSCTL";
  if (main == 5) return "ACRO"; if (main == 6) return "OFFBOARD"; if (main == 7) return "STABILIZED";
  if (main == 8) return "RATTITUDE";
  if (main != 4) return "";
  if (sub == 2) return "AUTO.TAKEOFF"; if (sub == 3) return "AUTO.LOITER";
  if (sub == 4) return "AUTO.MISSION"; if (sub == 5) return "AUTO.RTL";
  if (sub == 6) return "AUTO.LAND"; return "AUTO";
}
}  // namespace

class MavrosSim::Impl
{
public:
  Impl(rclcpp::Node & node, MavlinkIngress ingress) : node_(node), ingress_(std::move(ingress))
  {
    const auto sensor_qos = rclcpp::SensorDataQoS();
    // setpoint_raw plugin
    local_sub_ = node_.create_subscription<mavros_msgs::msg::PositionTarget>("mavros/setpoint_raw/local", sensor_qos,
      [this](mavros_msgs::msg::PositionTarget::SharedPtr msg) {setpoint_local(*msg);});
    global_sub_ = node_.create_subscription<mavros_msgs::msg::GlobalPositionTarget>("mavros/setpoint_raw/global", sensor_qos,
      [this](mavros_msgs::msg::GlobalPositionTarget::SharedPtr msg) {setpoint_global(*msg);});
    attitude_sub_ = node_.create_subscription<mavros_msgs::msg::AttitudeTarget>("mavros/setpoint_raw/attitude", sensor_qos,
      [this](mavros_msgs::msg::AttitudeTarget::SharedPtr msg) {setpoint_attitude(*msg);});
    target_local_pub_ = node_.create_publisher<mavros_msgs::msg::PositionTarget>("mavros/setpoint_raw/target_local", sensor_qos);
    target_global_pub_ = node_.create_publisher<mavros_msgs::msg::GlobalPositionTarget>("mavros/setpoint_raw/target_global", sensor_qos);
    target_attitude_pub_ = node_.create_publisher<mavros_msgs::msg::AttitudeTarget>("mavros/setpoint_raw/target_attitude", sensor_qos);
    // local_position plugin
    pose_pub_ = node_.create_publisher<geometry_msgs::msg::PoseStamped>("mavros/local_position/pose", sensor_qos);
    pose_cov_pub_ = node_.create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("mavros/local_position/pose_cov", sensor_qos);
    velocity_local_pub_ = node_.create_publisher<geometry_msgs::msg::TwistStamped>("mavros/local_position/velocity_local", sensor_qos);
    velocity_body_pub_ = node_.create_publisher<geometry_msgs::msg::TwistStamped>("mavros/local_position/velocity_body", sensor_qos);
    velocity_cov_pub_ = node_.create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("mavros/local_position/velocity_body_cov", sensor_qos);
    odom_pub_ = node_.create_publisher<nav_msgs::msg::Odometry>("mavros/local_position/odom", sensor_qos);
    accel_pub_ = node_.create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>("mavros/local_position/accel", sensor_qos);
    // imu plugin
    imu_pub_ = node_.create_publisher<sensor_msgs::msg::Imu>("mavros/imu/data", sensor_qos);
    imu_raw_pub_ = node_.create_publisher<sensor_msgs::msg::Imu>("mavros/imu/data_raw", sensor_qos);
    magnetic_pub_ = node_.create_publisher<sensor_msgs::msg::MagneticField>("mavros/imu/mag", sensor_qos);
    temperature_imu_pub_ = node_.create_publisher<sensor_msgs::msg::Temperature>("mavros/imu/temperature_imu", sensor_qos);
    temperature_baro_pub_ = node_.create_publisher<sensor_msgs::msg::Temperature>("mavros/imu/temperature_baro", sensor_qos);
    static_pressure_pub_ = node_.create_publisher<sensor_msgs::msg::FluidPressure>("mavros/imu/static_pressure", sensor_qos);
    differential_pressure_pub_ = node_.create_publisher<sensor_msgs::msg::FluidPressure>("mavros/imu/diff_pressure", sensor_qos);
    // sys_status plugin
    state_pub_ = node_.create_publisher<mavros_msgs::msg::State>("mavros/state", rclcpp::QoS(10).transient_local());
    extended_state_pub_ = node_.create_publisher<mavros_msgs::msg::ExtendedState>("mavros/extended_state", sensor_qos);
    battery_pub_ = node_.create_publisher<sensor_msgs::msg::BatteryState>("mavros/battery", sensor_qos);
    // global_position plugin
    global_origin_sub_ = node_.create_subscription<geographic_msgs::msg::GeoPointStamped>("mavros/global_position/set_gp_origin", rclcpp::QoS(10),
      [this](geographic_msgs::msg::GeoPointStamped::SharedPtr msg) {set_global_origin(*msg);});
    global_pub_ = node_.create_publisher<sensor_msgs::msg::NavSatFix>("mavros/global_position/global", sensor_qos);
    global_local_pub_ = node_.create_publisher<nav_msgs::msg::Odometry>("mavros/global_position/local", sensor_qos);
    global_offset_pub_ = node_.create_publisher<geometry_msgs::msg::PoseStamped>("mavros/global_position/gp_lp_offset", sensor_qos);
    raw_fix_pub_ = node_.create_publisher<sensor_msgs::msg::NavSatFix>("mavros/global_position/raw/fix", sensor_qos);
    raw_sat_pub_ = node_.create_publisher<std_msgs::msg::UInt32>("mavros/global_position/raw/satellites", sensor_qos);
    rel_alt_pub_ = node_.create_publisher<std_msgs::msg::Float64>("mavros/global_position/rel_alt", sensor_qos);
    heading_pub_ = node_.create_publisher<std_msgs::msg::Float64>("mavros/global_position/compass_hdg", sensor_qos);
    global_origin_pub_ = node_.create_publisher<geographic_msgs::msg::GeoPointStamped>("mavros/global_position/gp_origin", rclcpp::QoS(1).transient_local());
    // command plugin
    set_mode_srv_ = node_.create_service<mavros_msgs::srv::SetMode>("mavros/set_mode", [this](const std::shared_ptr<mavros_msgs::srv::SetMode::Request> req, const std::shared_ptr<mavros_msgs::srv::SetMode::Response> res) {res->mode_sent = queue_mode(*req);});
    arming_srv_ = node_.create_service<mavros_msgs::srv::CommandBool>("mavros/cmd/arming", [this](const std::shared_ptr<mavros_msgs::srv::CommandBool::Request> req, const std::shared_ptr<mavros_msgs::srv::CommandBool::Response> res) {
      queue_command_long(MAV_CMD_COMPONENT_ARM_DISARM, req->value ? 1.0F : 0.0F, 0, 0, 0, 0, 0, 0, 1); res->success = true; res->result = MAV_RESULT_ACCEPTED;});
    command_long_srv_ = node_.create_service<mavros_msgs::srv::CommandLong>("mavros/cmd/command", [this](const std::shared_ptr<mavros_msgs::srv::CommandLong::Request> req, const std::shared_ptr<mavros_msgs::srv::CommandLong::Response> res) {
      queue_command_long(req->command, req->param1, req->param2, req->param3, req->param4, req->param5, req->param6, req->param7, req->confirmation, req->broadcast); res->success = true; res->result = MAV_RESULT_ACCEPTED;});
    command_int_srv_ = node_.create_service<mavros_msgs::srv::CommandInt>("mavros/cmd/command_int", [this](const std::shared_ptr<mavros_msgs::srv::CommandInt::Request> req, const std::shared_ptr<mavros_msgs::srv::CommandInt::Response> res) {
      mavlink_message_t msg{}; mavlink_msg_command_int_pack(kSystemId, kComponentId, &msg, req->broadcast ? 0 : kSystemId, req->broadcast ? 0 : kComponentId, req->frame, req->command, req->current, req->autocontinue, req->param1, req->param2, req->param3, req->param4, req->x, req->y, req->z); send(px4::mavlink_receive_handle::COMMAND_INT, msg); res->success = true;});
    takeoff_srv_ = node_.create_service<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff", [this](const std::shared_ptr<mavros_msgs::srv::CommandTOL::Request> req, const std::shared_ptr<mavros_msgs::srv::CommandTOL::Response> res) {
      queue_command_long(MAV_CMD_NAV_TAKEOFF, req->min_pitch, 0, 0, req->yaw, req->latitude, req->longitude, req->altitude, 1); res->success = true; res->result = MAV_RESULT_ACCEPTED;});
    land_srv_ = node_.create_service<mavros_msgs::srv::CommandTOL>("mavros/cmd/land", [this](const std::shared_ptr<mavros_msgs::srv::CommandTOL::Request> req, const std::shared_ptr<mavros_msgs::srv::CommandTOL::Response> res) {
      queue_command_long(MAV_CMD_NAV_LAND, 0, 0, 0, req->yaw, req->latitude, req->longitude, req->altitude, 1); res->success = true; res->result = MAV_RESULT_ACCEPTED;});
  }

  void handle(const mavlink_message_t & msg, const rclcpp::Time & stamp)
  {
    switch (msg.msgid) {
      case MAVLINK_MSG_ID_ATTITUDE_QUATERNION: publish_attitude(msg, stamp); break;
      case MAVLINK_MSG_ID_ATTITUDE_TARGET: publish_attitude_target(msg, stamp); break;
      case MAVLINK_MSG_ID_HEARTBEAT: publish_heartbeat(msg, stamp); break;
      case MAVLINK_MSG_ID_LOCAL_POSITION_NED: publish_local_position(msg, stamp); break;
      case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED: publish_local_target(msg, stamp); break;
      case MAVLINK_MSG_ID_SYS_STATUS: publish_battery(msg, stamp); break;
      case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: publish_global_position(msg, stamp); break;
      case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN: publish_global_origin(msg, stamp); break;
      default: break;
    }
  }

private:
  template<typename T> T parameter(const std::string & name, const T & value)
  {if (!node_.has_parameter(name)) node_.declare_parameter<T>(name, value); return node_.get_parameter(name).get_value<T>();}
  void send(px4::mavlink_receive_handle handle, const mavlink_message_t & msg) {ingress_(handle, msg);}
  void setpoint_local(const mavros_msgs::msg::PositionTarget & req)
  {const auto p=enu_to_ned(req.position), v=enu_to_ned(req.velocity), a=enu_to_ned(req.acceleration_or_force); mavlink_message_t msg{}; mavlink_msg_set_position_target_local_ned_pack(kSystemId,kComponentId,&msg,stamp_ms(req.header.stamp),kSystemId,kComponentId,req.coordinate_frame,req.type_mask,p.x(),p.y(),p.z(),v.x(),v.y(),v.z(),a.x(),a.y(),a.z(),yaw_enu_to_ned(req.yaw),-req.yaw_rate); send(px4::mavlink_receive_handle::SET_POSITION_TARGET_LOCAL_NED,msg);}
  void setpoint_global(const mavros_msgs::msg::GlobalPositionTarget & req)
  {const auto v=enu_to_ned(req.velocity), a=enu_to_ned(req.acceleration_or_force); mavlink_message_t msg{}; mavlink_msg_set_position_target_global_int_pack(kSystemId,kComponentId,&msg,stamp_ms(req.header.stamp),kSystemId,kComponentId,req.coordinate_frame,req.type_mask,static_cast<int32_t>(std::llround(req.latitude*1e7)),static_cast<int32_t>(std::llround(req.longitude*1e7)),req.altitude,v.x(),v.y(),v.z(),a.x(),a.y(),a.z(),yaw_enu_to_ned(req.yaw),-req.yaw_rate); send(px4::mavlink_receive_handle::SET_POSITION_TARGET_GLOBAL_INT,msg); target_global_pub_->publish(req);}
  void setpoint_attitude(const mavros_msgs::msg::AttitudeTarget & req)
  {const auto q=enu_flu_to_ned_frd(to_eigen(req.orientation)); const auto r=mavros::ftf::transform_frame_baselink_aircraft(Eigen::Vector3d(req.body_rate.x,req.body_rate.y,req.body_rate.z)); float qa[4]{static_cast<float>(q.w()),static_cast<float>(q.x()),static_cast<float>(q.y()),static_cast<float>(q.z())}; float thrust_body[3]{}; mavlink_message_t msg{}; mavlink_msg_set_attitude_target_pack(kSystemId,kComponentId,&msg,stamp_ms(req.header.stamp),kSystemId,kComponentId,req.type_mask,qa,r.x(),r.y(),r.z(),req.thrust,thrust_body); send(px4::mavlink_receive_handle::SET_ATTITUDE_TARGET,msg);}
  bool queue_mode(const mavros_msgs::srv::SetMode::Request & req)
  {uint32_t mode=0; if (!req.custom_mode.empty()) {const std::array<std::pair<const char*,uint32_t>,10> modes{{{"MANUAL",1U<<16U},{"ALTCTL",2U<<16U},{"POSCTL",3U<<16U},{"AUTO.MISSION",(4U<<16U)|(4U<<24U)},{"AUTO.LOITER",(4U<<16U)|(3U<<24U)},{"AUTO.RTL",(4U<<16U)|(5U<<24U)},{"AUTO.LAND",(4U<<16U)|(6U<<24U)},{"AUTO.TAKEOFF",(4U<<16U)|(2U<<24U)},{"ACRO",5U<<16U},{"OFFBOARD",6U<<16U}}}; bool found=false; for (const auto & item:modes) if (req.custom_mode==item.first) {mode=item.second;found=true;break;} if (!found) return false;} mavlink_message_t msg{}; mavlink_msg_set_mode_pack(kSystemId,kComponentId,&msg,kSystemId,static_cast<uint8_t>(req.base_mode|MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),mode); send(px4::mavlink_receive_handle::SET_MODE,msg); return true;}
  void queue_command_long(uint16_t c,float p1,float p2,float p3,float p4,float p5,float p6,float p7,uint8_t confirmation,bool broadcast=false)
  {mavlink_message_t msg{}; mavlink_msg_command_long_pack(kSystemId,kComponentId,&msg,broadcast?0:kSystemId,broadcast?0:kComponentId,c,broadcast?0:confirmation,p1,p2,p3,p4,p5,p6,p7); send(px4::mavlink_receive_handle::COMMAND_LONG,msg);}
  void set_global_origin(const geographic_msgs::msg::GeoPointStamped & req)
  {mavlink_message_t msg{}; mavlink_msg_set_gps_global_origin_pack(kSystemId,kComponentId,&msg,kSystemId,static_cast<int32_t>(std::llround(req.position.latitude*1e7)),static_cast<int32_t>(std::llround(req.position.longitude*1e7)),static_cast<int32_t>(std::llround(req.position.altitude*1000.0)),0); send(px4::mavlink_receive_handle::SET_GPS_GLOBAL_ORIGIN,msg);}
  void publish_attitude(const mavlink_message_t & msg,const rclcpp::Time & stamp)
  {mavlink_attitude_quaternion_t d{};mavlink_msg_attitude_quaternion_decode(&msg,&d);const auto q=ned_frd_to_enu_flu(Eigen::Quaterniond(d.q1,d.q2,d.q3,d.q4));const auto omega=mavros::ftf::transform_frame_aircraft_baselink(Eigen::Vector3d(d.rollspeed,d.pitchspeed,d.yawspeed));attitude_enu_=q;body_omega_=omega;sensor_msgs::msg::Imu imu;imu.header.stamp=stamp;imu.header.frame_id=parameter("imu.frame_id",std::string("base_link"));imu.orientation=to_msg(q);imu.angular_velocity.x=omega.x();imu.angular_velocity.y=omega.y();imu.angular_velocity.z=omega.z();imu_pub_->publish(imu);sensor_msgs::msg::Imu raw=imu;raw.orientation_covariance[0]=-1.0;imu_raw_pub_->publish(raw);sensor_msgs::msg::MagneticField mag;mag.header=imu.header;magnetic_pub_->publish(mag);sensor_msgs::msg::Temperature temp;temp.header=imu.header;temp.temperature=std::numeric_limits<float>::quiet_NaN();temperature_imu_pub_->publish(temp);temperature_baro_pub_->publish(temp);sensor_msgs::msg::FluidPressure pressure;pressure.header=imu.header;pressure.fluid_pressure=std::numeric_limits<float>::quiet_NaN();static_pressure_pub_->publish(pressure);differential_pressure_pub_->publish(pressure);}
  void publish_attitude_target(const mavlink_message_t & msg,const rclcpp::Time & stamp)
  {mavlink_attitude_target_t d{};mavlink_msg_attitude_target_decode(&msg,&d);mavros_msgs::msg::AttitudeTarget t;t.header.stamp=stamp;t.type_mask=d.type_mask;t.orientation=to_msg(ned_frd_to_enu_flu(Eigen::Quaterniond(d.q[0],d.q[1],d.q[2],d.q[3])));const auto r=mavros::ftf::transform_frame_aircraft_baselink(Eigen::Vector3d(d.body_roll_rate,d.body_pitch_rate,d.body_yaw_rate));t.body_rate.x=r.x();t.body_rate.y=r.y();t.body_rate.z=r.z();t.thrust=d.thrust;target_attitude_pub_->publish(t);}
  void publish_heartbeat(const mavlink_message_t & msg,const rclcpp::Time & stamp)
  {mavlink_heartbeat_t d{};mavlink_msg_heartbeat_decode(&msg,&d);mavros_msgs::msg::State s;s.header.stamp=stamp;s.connected=true;s.armed=(d.base_mode&MAV_MODE_FLAG_SAFETY_ARMED)!=0;s.guided=(d.base_mode&MAV_MODE_FLAG_GUIDED_ENABLED)!=0;s.manual_input=(d.base_mode&MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)!=0;s.mode=px4_mode(d.custom_mode);s.system_status=d.system_status;state_pub_->publish(s);mavros_msgs::msg::ExtendedState e;e.header.stamp=stamp;e.vtol_state=mavros_msgs::msg::ExtendedState::VTOL_STATE_MC;e.landed_state=s.armed?mavros_msgs::msg::ExtendedState::LANDED_STATE_IN_AIR:mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND;extended_state_pub_->publish(e);}
  void publish_local_position(const mavlink_message_t & msg,const rclcpp::Time & stamp)
  {mavlink_local_position_ned_t d{};mavlink_msg_local_position_ned_decode(&msg,&d);const auto p=ned_to_enu_point(Eigen::Vector3d(d.x,d.y,d.z));const auto v=ned_to_enu(Eigen::Vector3d(d.vx,d.vy,d.vz));geometry_msgs::msg::PoseStamped pose;pose.header.stamp=stamp;pose.header.frame_id=parameter("local_position.frame_id",std::string("map"));pose.pose.position=p;pose.pose.orientation=to_msg(attitude_enu_);pose_pub_->publish(pose);geometry_msgs::msg::PoseWithCovarianceStamped pose_cov;pose_cov.header=pose.header;pose_cov.pose.pose=pose.pose;pose_cov_pub_->publish(pose_cov);geometry_msgs::msg::TwistStamped vl;vl.header=pose.header;vl.twist.linear=v;vl.twist.angular.x=body_omega_.x();vl.twist.angular.y=body_omega_.y();vl.twist.angular.z=body_omega_.z();velocity_local_pub_->publish(vl);geometry_msgs::msg::TwistStamped vb=vl;vb.header.frame_id=parameter("local_position.tf.child_frame_id",std::string("base_link"));const Eigen::Vector3d bv=attitude_enu_.inverse()*Eigen::Vector3d(v.x,v.y,v.z);vb.twist.linear.x=bv.x();vb.twist.linear.y=bv.y();vb.twist.linear.z=bv.z();velocity_body_pub_->publish(vb);geometry_msgs::msg::TwistWithCovarianceStamped vc;vc.header=vb.header;vc.twist.twist=vb.twist;velocity_cov_pub_->publish(vc);geometry_msgs::msg::AccelWithCovarianceStamped a;a.header=pose.header;accel_pub_->publish(a);nav_msgs::msg::Odometry odom;odom.header=pose.header;odom.child_frame_id=vb.header.frame_id;odom.pose.pose=pose.pose;odom.twist.twist=vb.twist;odom_pub_->publish(odom);latest_local_odom_=odom;}
  void publish_local_target(const mavlink_message_t & msg,const rclcpp::Time & stamp)
  {mavlink_position_target_local_ned_t d{};mavlink_msg_position_target_local_ned_decode(&msg,&d);mavros_msgs::msg::PositionTarget t;t.header.stamp=stamp;t.coordinate_frame=d.coordinate_frame;t.type_mask=d.type_mask;t.position=ned_to_enu_point(Eigen::Vector3d(d.x,d.y,d.z));t.velocity=ned_to_enu(Eigen::Vector3d(d.vx,d.vy,d.vz));t.acceleration_or_force=ned_to_enu(Eigen::Vector3d(d.afx,d.afy,d.afz));t.yaw=yaw_ned_to_enu(d.yaw);t.yaw_rate=-d.yaw_rate;target_local_pub_->publish(t);}
  void publish_battery(const mavlink_message_t & msg,const rclcpp::Time & stamp)
  {mavlink_sys_status_t d{};mavlink_msg_sys_status_decode(&msg,&d);sensor_msgs::msg::BatteryState b;b.header.stamp=stamp;b.voltage=d.voltage_battery==UINT16_MAX?std::numeric_limits<float>::quiet_NaN():d.voltage_battery/1000.0F;b.current=d.current_battery<0?std::numeric_limits<float>::quiet_NaN():-d.current_battery/100.0F;b.percentage=d.battery_remaining<0?std::numeric_limits<float>::quiet_NaN():d.battery_remaining/100.0F;b.present=d.voltage_battery!=UINT16_MAX;battery_pub_->publish(b);}
  void publish_global_position(const mavlink_message_t & msg,const rclcpp::Time & stamp)
  {mavlink_global_position_int_t d{};mavlink_msg_global_position_int_decode(&msg,&d);sensor_msgs::msg::NavSatFix fix;fix.header.stamp=stamp;fix.header.frame_id=parameter("global_position.frame_id",std::string("map"));fix.latitude=d.lat/1e7;fix.longitude=d.lon/1e7;fix.altitude=d.alt/1000.0;fix.status.service=sensor_msgs::msg::NavSatStatus::SERVICE_GPS;fix.status.status=sensor_msgs::msg::NavSatStatus::STATUS_FIX;global_pub_->publish(fix);raw_fix_pub_->publish(fix);std_msgs::msg::Float64 rel;rel.data=d.relative_alt/1000.0;rel_alt_pub_->publish(rel);std_msgs::msg::Float64 heading;heading.data=d.hdg==UINT16_MAX?std::numeric_limits<double>::quiet_NaN():d.hdg/100.0;heading_pub_->publish(heading);std_msgs::msg::UInt32 sats;sats.data=0;raw_sat_pub_->publish(sats);auto odom=latest_local_odom_;odom.header.stamp=stamp;odom.header.frame_id=parameter("global_position.frame_id",std::string("map"));odom.child_frame_id=parameter("global_position.tf.child_frame_id",std::string("base_link"));global_local_pub_->publish(odom);geometry_msgs::msg::PoseStamped offset;offset.header=odom.header;offset.pose.orientation.w=1.0;global_offset_pub_->publish(offset);}
  void publish_global_origin(const mavlink_message_t & msg,const rclcpp::Time & stamp)
  {mavlink_gps_global_origin_t d{};mavlink_msg_gps_global_origin_decode(&msg,&d);geographic_msgs::msg::GeoPointStamped o;o.header.stamp=stamp;o.header.frame_id=parameter("global_position.tf.global_frame_id",std::string("earth"));o.position.latitude=d.latitude/1e7;o.position.longitude=d.longitude/1e7;o.position.altitude=d.altitude/1000.0;global_origin_pub_->publish(o);}

  rclcpp::Node & node_; MavlinkIngress ingress_;
  Eigen::Quaterniond attitude_enu_{Eigen::Quaterniond::Identity()}; Eigen::Vector3d body_omega_{Eigen::Vector3d::Zero()}; nav_msgs::msg::Odometry latest_local_odom_;
  rclcpp::Subscription<mavros_msgs::msg::PositionTarget>::SharedPtr local_sub_; rclcpp::Subscription<mavros_msgs::msg::GlobalPositionTarget>::SharedPtr global_sub_; rclcpp::Subscription<mavros_msgs::msg::AttitudeTarget>::SharedPtr attitude_sub_; rclcpp::Subscription<geographic_msgs::msg::GeoPointStamped>::SharedPtr global_origin_sub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr target_local_pub_; rclcpp::Publisher<mavros_msgs::msg::GlobalPositionTarget>::SharedPtr target_global_pub_; rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr target_attitude_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_,global_offset_pub_; rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_pub_; rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_local_pub_,velocity_body_pub_; rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_cov_pub_; rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_,global_local_pub_; rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr accel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_,imu_raw_pub_; rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magnetic_pub_; rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_imu_pub_,temperature_baro_pub_; rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr static_pressure_pub_,differential_pressure_pub_; rclcpp::Publisher<mavros_msgs::msg::State>::SharedPtr state_pub_; rclcpp::Publisher<mavros_msgs::msg::ExtendedState>::SharedPtr extended_state_pub_; rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_; rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr global_pub_,raw_fix_pub_; rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr raw_sat_pub_; rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rel_alt_pub_,heading_pub_; rclcpp::Publisher<geographic_msgs::msg::GeoPointStamped>::SharedPtr global_origin_pub_;
  rclcpp::Service<mavros_msgs::srv::SetMode>::SharedPtr set_mode_srv_; rclcpp::Service<mavros_msgs::srv::CommandBool>::SharedPtr arming_srv_; rclcpp::Service<mavros_msgs::srv::CommandLong>::SharedPtr command_long_srv_; rclcpp::Service<mavros_msgs::srv::CommandInt>::SharedPtr command_int_srv_; rclcpp::Service<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_srv_,land_srv_;
};

MavrosSim::MavrosSim(rclcpp::Node & node, MavlinkIngress ingress) : impl_(std::make_unique<Impl>(node, std::move(ingress))) {}
MavrosSim::~MavrosSim() = default;
void MavrosSim::handle_message(const mavlink_message_t & message, const rclcpp::Time & stamp) {impl_->handle(message, stamp);}

}  // namespace fss_px4_sim::mavros_sim
