#include "fss_px4_sim/mavros_lite/core.hpp"

#include <mavros/frame_tf.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace fss_px4_sim::mavros_lite
{
class GlobalPosition final : public Module
{
public:
  explicit GlobalPosition(Core & core)
  : Module(core)
  {
    frame_id_ = core.declare_parameter<std::string>("global_position.frame_id", "map");
    child_frame_id_ = core.declare_parameter<std::string>("global_position.child_frame_id", "base_link");
    const auto qos = rclcpp::SensorDataQoS();
    raw_fix_pub_ = core.create_publisher<sensor_msgs::msg::NavSatFix>("global_position/raw/fix", qos);
    raw_vel_pub_ = core.create_publisher<geometry_msgs::msg::TwistStamped>("global_position/raw/gps_vel", qos);
    raw_sat_pub_ = core.create_publisher<std_msgs::msg::UInt32>("global_position/raw/satellites", qos);
    global_pub_ = core.create_publisher<sensor_msgs::msg::NavSatFix>("global_position/global", qos);
    local_pub_ = core.create_publisher<nav_msgs::msg::Odometry>("global_position/local", qos);
    rel_alt_pub_ = core.create_publisher<std_msgs::msg::Float64>("global_position/rel_alt", qos);
    heading_pub_ = core.create_publisher<std_msgs::msg::Float64>("global_position/compass_hdg", qos);
  }

  void handle_message(const mavlink_message_t & message) override
  {
    if (message.msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
      mavlink_gps_raw_int_t value{};
      mavlink_msg_gps_raw_int_decode(&message, &value);
      sensor_msgs::msg::NavSatFix fix;
      fix.header.stamp = core_.stamp();
      fix.header.frame_id = child_frame_id_;
      fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
      fix.status.status = value.fix_type > 2 ? sensor_msgs::msg::NavSatStatus::STATUS_FIX :
        sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      fix.latitude = value.lat / 1e7;
      fix.longitude = value.lon / 1e7;
      fix.altitude = value.alt / 1e3;
      if (value.eph != UINT16_MAX && value.epv != UINT16_MAX) {
        fix.position_covariance[0] = fix.position_covariance[4] =
          std::pow(value.eph / 100.0, 2);
        fix.position_covariance[8] = std::pow(value.epv / 100.0, 2);
        fix.position_covariance_type =
          sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      } else {
        fix.position_covariance[0] = -1.0;
        fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
      }
      {std::lock_guard<std::mutex> lock(core_.state().mutex); core_.state().gps_fix = fix; core_.state().have_gps_fix = true;}
      raw_fix_pub_->publish(fix);
      std_msgs::msg::UInt32 satellites; satellites.data = value.satellites_visible; raw_sat_pub_->publish(satellites);
      if (value.vel != UINT16_MAX && value.cog != UINT16_MAX) {
        geometry_msgs::msg::TwistStamped velocity; velocity.header = fix.header;
        const double course = value.cog / 100.0 * M_PI / 180.0;
        velocity.twist.linear.x = value.vel / 100.0 * std::sin(course);
        velocity.twist.linear.y = value.vel / 100.0 * std::cos(course);
        raw_vel_pub_->publish(velocity);
      }
    } else if (message.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
      mavlink_global_position_int_t value{};
      mavlink_msg_global_position_int_decode(&message, &value);
      sensor_msgs::msg::NavSatFix fix;
      fix.header.stamp = core_.stamp(); fix.header.frame_id = child_frame_id_;
      fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
      fix.latitude = value.lat / 1e7; fix.longitude = value.lon / 1e7; fix.altitude = value.alt / 1e3;
      sensor_msgs::msg::NavSatFix raw_fix;
      {
        std::lock_guard<std::mutex> lock(core_.state().mutex);
        if (core_.state().have_gps_fix) raw_fix = core_.state().gps_fix;
      }
      fix.position_covariance = raw_fix.position_covariance;
      fix.position_covariance_type = raw_fix.position_covariance_type;
      global_pub_->publish(fix);
      nav_msgs::msg::Odometry odom; odom.header = fix.header; odom.header.frame_id = frame_id_;
      odom.child_frame_id = child_frame_id_;
      tf2::toMsg(mavros::ftf::transform_frame_ned_enu(Eigen::Vector3d(value.vx, value.vy, value.vz)) / 100.0,
        odom.twist.twist.linear);
      const auto ecef = geodetic_to_ecef(fix.latitude, fix.longitude, fix.altitude);
      if (!map_initialized_) {
        map_origin_ = {fix.latitude, fix.longitude, fix.altitude};
        ecef_origin_ = ecef;
        map_initialized_ = true;
      }
      const Eigen::Vector3d local_ecef = ecef - ecef_origin_;
      odom.pose.pose.position = tf2::toMsg(
        mavros::ftf::transform_frame_ecef_enu(local_ecef, map_origin_));
      odom.pose.pose.position.z = value.relative_alt / 1000.0;
      {std::lock_guard<std::mutex> lock(core_.state().mutex); odom.pose.pose.orientation = core_.state().orientation;}
      local_pub_->publish(odom);
      std_msgs::msg::Float64 relative; relative.data = value.relative_alt / 1000.0; rel_alt_pub_->publish(relative);
      std_msgs::msg::Float64 heading; heading.data = value.hdg == UINT16_MAX ? NAN : value.hdg / 100.0; heading_pub_->publish(heading);
    }
  }

private:
  static Eigen::Vector3d geodetic_to_ecef(double latitude, double longitude, double altitude)
  {
    constexpr double semi_major = 6378137.0;
    constexpr double eccentricity_squared = 6.69437999014e-3;
    const double latitude_rad = latitude * M_PI / 180.0;
    const double longitude_rad = longitude * M_PI / 180.0;
    const double radius = semi_major /
      std::sqrt(1.0 - eccentricity_squared * std::pow(std::sin(latitude_rad), 2));
    return {
      (radius + altitude) * std::cos(latitude_rad) * std::cos(longitude_rad),
      (radius + altitude) * std::cos(latitude_rad) * std::sin(longitude_rad),
      (radius * (1.0 - eccentricity_squared) + altitude) * std::sin(latitude_rad)};
  }

  std::string frame_id_, child_frame_id_;
  bool map_initialized_{false};
  Eigen::Vector3d map_origin_{}, ecef_origin_{};
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr raw_fix_pub_, global_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr raw_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr raw_sat_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr local_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rel_alt_pub_, heading_pub_;
};
std::unique_ptr<Module> make_global_position(Core & core) {return std::make_unique<GlobalPosition>(core);}
}  // namespace fss_px4_sim::mavros_lite
