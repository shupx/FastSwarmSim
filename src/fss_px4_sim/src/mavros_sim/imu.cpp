#include "fss_px4_sim/mavros_sim/plugin.hpp"

#include <limits>

#include "mavros/frame_tf.h"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"

namespace fss_px4_sim::mavros_sim
{
namespace
{
constexpr double kMilliGToMs2=9.80665/1000.0, kMilliRadToRad=1e-3, kMilliTeslaToTesla=1e-3, kGaussToTesla=1e-4;
geometry_msgs::msg::Quaternion quat(const Eigen::Quaterniond & q) {geometry_msgs::msg::Quaternion r; r.w=q.w();r.x=q.x();r.y=q.y();r.z=q.z();return r;}
geometry_msgs::msg::Vector3 vec(const Eigen::Vector3d & v) {geometry_msgs::msg::Vector3 r;r.x=v.x();r.y=v.y();r.z=v.z();return r;}
}
class ImuPlugin final : public Plugin
{
public:
  using Plugin::Plugin;
  ImuPlugin(rclcpp::Node & node,MavlinkIngress ingress,std::shared_ptr<SharedState> state):Plugin(node,std::move(ingress),std::move(state))
  {
    const auto qos=rclcpp::SensorDataQoS(); frame_id_=parameter("imu.frame_id",std::string("base_link"));
    imu_pub_=node_.create_publisher<sensor_msgs::msg::Imu>("mavros/imu/data",qos); raw_pub_=node_.create_publisher<sensor_msgs::msg::Imu>("mavros/imu/data_raw",qos);
    mag_pub_=node_.create_publisher<sensor_msgs::msg::MagneticField>("mavros/imu/mag",qos); temp_imu_pub_=node_.create_publisher<sensor_msgs::msg::Temperature>("mavros/imu/temperature_imu",qos); temp_baro_pub_=node_.create_publisher<sensor_msgs::msg::Temperature>("mavros/imu/temperature_baro",qos);
    static_pub_=node_.create_publisher<sensor_msgs::msg::FluidPressure>("mavros/imu/static_pressure",qos); diff_pub_=node_.create_publisher<sensor_msgs::msg::FluidPressure>("mavros/imu/diff_pressure",qos);
  }
  bool handles(uint32_t id) const override {return id==MAVLINK_MSG_ID_ATTITUDE || id==MAVLINK_MSG_ID_ATTITUDE_QUATERNION || id==MAVLINK_MSG_ID_HIGHRES_IMU || id==MAVLINK_MSG_ID_RAW_IMU || id==MAVLINK_MSG_ID_SCALED_IMU || id==MAVLINK_MSG_ID_SCALED_PRESSURE;}
  void handle_message(const mavlink_message_t & message,const rclcpp::Time & stamp) override
  {
    switch (message.msgid) {
      case MAVLINK_MSG_ID_ATTITUDE: {mavlink_attitude_t d{};mavlink_msg_attitude_decode(&message,&d);const Eigen::AngleAxisd r(d.roll,Eigen::Vector3d::UnitX()),p(d.pitch,Eigen::Vector3d::UnitY()),y(d.yaw,Eigen::Vector3d::UnitZ());publish_attitude(r*p*y,{d.rollspeed,d.pitchspeed,d.yawspeed},stamp);break;}
      case MAVLINK_MSG_ID_ATTITUDE_QUATERNION: {mavlink_attitude_quaternion_t d{};mavlink_msg_attitude_quaternion_decode(&message,&d);publish_attitude({d.q1,d.q2,d.q3,d.q4},{d.rollspeed,d.pitchspeed,d.yawspeed},stamp);break;}
      case MAVLINK_MSG_ID_HIGHRES_IMU: {mavlink_highres_imu_t d{};mavlink_msg_highres_imu_decode(&message,&d);has_hr_=true;if (d.fields_updated&0x3f) publish_raw({d.xgyro,d.ygyro,d.zgyro},{d.xacc,d.yacc,d.zacc},stamp);if(d.fields_updated&(7<<6)) publish_mag({d.xmag*kGaussToTesla,d.ymag*kGaussToTesla,d.zmag*kGaussToTesla},stamp);if(d.fields_updated&(1<<9)) pressure(static_pub_,d.abs_pressure,stamp);if(d.fields_updated&(1<<10)) pressure(diff_pub_,d.diff_pressure,stamp);if(d.fields_updated&(1<<12)) temperature(temp_imu_pub_,d.temperature,stamp);break;}
      case MAVLINK_MSG_ID_RAW_IMU: {mavlink_raw_imu_t d{};mavlink_msg_raw_imu_decode(&message,&d);if(!has_hr_&&!has_scaled_) {publish_raw({d.xgyro*kMilliRadToRad,d.ygyro*kMilliRadToRad,d.zgyro*kMilliRadToRad},{d.xacc*1e-3,d.yacc*1e-3,d.zacc*1e-3},stamp);publish_mag({d.xmag*kMilliTeslaToTesla,d.ymag*kMilliTeslaToTesla,d.zmag*kMilliTeslaToTesla},stamp);}break;}
      case MAVLINK_MSG_ID_SCALED_IMU: {mavlink_scaled_imu_t d{};mavlink_msg_scaled_imu_decode(&message,&d);if(!has_hr_){has_scaled_=true;publish_raw({d.xgyro*kMilliRadToRad,d.ygyro*kMilliRadToRad,d.zgyro*kMilliRadToRad},{d.xacc*kMilliGToMs2,d.yacc*kMilliGToMs2,d.zacc*kMilliGToMs2},stamp);publish_mag({d.xmag*kMilliTeslaToTesla,d.ymag*kMilliTeslaToTesla,d.zmag*kMilliTeslaToTesla},stamp);}break;}
      case MAVLINK_MSG_ID_SCALED_PRESSURE: {mavlink_scaled_pressure_t d{};mavlink_msg_scaled_pressure_decode(&message,&d);if(!has_hr_){temperature(temp_baro_pub_,d.temperature/100.0,stamp);pressure(static_pub_,d.press_abs*100.0,stamp);pressure(diff_pub_,d.press_diff*100.0,stamp);}break;}
    }
  }
private:
  void publish_attitude(const Eigen::Quaterniond & q_ned,const Eigen::Vector3d & omega_frd,const rclcpp::Time & stamp)
  {const auto q=mavros::ftf::transform_orientation_baselink_aircraft(mavros::ftf::transform_orientation_enu_ned(q_ned));const auto omega=mavros::ftf::transform_frame_aircraft_baselink(omega_frd);state_->attitude_enu=q;state_->body_omega_flu=omega;sensor_msgs::msg::Imu imu;imu.header.stamp=stamp;imu.header.frame_id=frame_id_;imu.orientation=quat(q);imu.angular_velocity=vec(omega);imu.linear_acceleration=vec(linear_accel_);imu_pub_->publish(imu);}
  void publish_raw(const Eigen::Vector3d & gyro_frd,const Eigen::Vector3d & accel_frd,const rclcpp::Time & stamp)
  {linear_accel_=mavros::ftf::transform_frame_aircraft_baselink(accel_frd);sensor_msgs::msg::Imu out;out.header.stamp=stamp;out.header.frame_id=frame_id_;out.orientation_covariance[0]=-1.0;out.angular_velocity=vec(mavros::ftf::transform_frame_aircraft_baselink(gyro_frd));out.linear_acceleration=vec(linear_accel_);raw_pub_->publish(out);}
  void publish_mag(const Eigen::Vector3d & mag_frd,const rclcpp::Time & stamp) {sensor_msgs::msg::MagneticField out;out.header.stamp=stamp;out.header.frame_id=frame_id_;out.magnetic_field=vec(mavros::ftf::transform_frame_aircraft_baselink(mag_frd));mag_pub_->publish(out);}
  void temperature(const rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr & pub,double value,const rclcpp::Time & stamp) {sensor_msgs::msg::Temperature out;out.header.stamp=stamp;out.header.frame_id=frame_id_;out.temperature=value;pub->publish(out);}
  void pressure(const rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr & pub,double value,const rclcpp::Time & stamp) {sensor_msgs::msg::FluidPressure out;out.header.stamp=stamp;out.header.frame_id=frame_id_;out.fluid_pressure=value;pub->publish(out);}
  std::string frame_id_;bool has_hr_{false},has_scaled_{false};Eigen::Vector3d linear_accel_{Eigen::Vector3d::Zero()};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_,raw_pub_;rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_imu_pub_,temp_baro_pub_;rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr static_pub_,diff_pub_;
};
std::unique_ptr<Plugin> make_imu_plugin(rclcpp::Node & node,MavlinkIngress ingress,std::shared_ptr<SharedState> state) {return std::make_unique<ImuPlugin>(node,std::move(ingress),std::move(state));}
}  // namespace fss_px4_sim::mavros_sim
