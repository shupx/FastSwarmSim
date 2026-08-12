#include "fss_px4_sim/mavros_lite/core.hpp"

#include <array>
#include <cmath>

#include <mavros/frame_tf.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace fss_px4_sim::mavros_lite
{
namespace ftf = mavros::ftf;

class Imu final : public Module
{
public:
  explicit Imu(MavrosLite & core)
  : Module(core)
  {
    frame_id_ = core.declare_parameter<std::string>("imu.frame_id", "base_link");
    const auto qos = rclcpp::SensorDataQoS();
    data_pub_ = core.create_publisher<sensor_msgs::msg::Imu>("imu/data", qos);
    raw_pub_ = core.create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", qos);
    mag_pub_ = core.create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", qos);
    temp_pub_ = core.create_publisher<sensor_msgs::msg::Temperature>("imu/temperature_imu", qos);
    pressure_pub_ = core.create_publisher<sensor_msgs::msg::FluidPressure>(
      "imu/static_pressure", qos);
  }

  void handle_message(const mavlink_message_t & message) override
  {
    if (message.msgid == MAVLINK_MSG_ID_ATTITUDE_QUATERNION) {
      mavlink_attitude_quaternion_t value{};
      mavlink_msg_attitude_quaternion_decode(&message, &value);
      publish_attitude(value.time_boot_ms, Eigen::Quaterniond(value.q1, value.q2, value.q3, value.q4),
        Eigen::Vector3d(value.rollspeed, value.pitchspeed, value.yawspeed));
    } else if (message.msgid == MAVLINK_MSG_ID_ATTITUDE) {
      mavlink_attitude_t value{};
      mavlink_msg_attitude_decode(&message, &value);
      publish_attitude(value.time_boot_ms, ftf::quaternion_from_rpy(value.roll, value.pitch, value.yaw),
        Eigen::Vector3d(value.rollspeed, value.pitchspeed, value.yawspeed));
    } else if (message.msgid == MAVLINK_MSG_ID_HIGHRES_IMU) {
      mavlink_highres_imu_t value{};
      mavlink_msg_highres_imu_decode(&message, &value);
      const auto stamp = core_.stamp();
      if (value.fields_updated & ((7 << 3) | (7 << 0))) {
        publish_raw(stamp,
          ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(Eigen::Vector3d(
            value.xgyro, value.ygyro, value.zgyro)),
          ftf::transform_frame_aircraft_baselink(Eigen::Vector3d(value.xacc, value.yacc, value.zacc)));
      }
      if (value.fields_updated & (7 << 6)) {
        publish_mag(stamp, ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(
          Eigen::Vector3d(value.xmag, value.ymag, value.zmag) * 1e-4));
      }
      if (value.fields_updated & (1 << 9)) {
        sensor_msgs::msg::FluidPressure output;
        output.header = header(stamp);
        output.fluid_pressure = value.abs_pressure * 100.0;
        pressure_pub_->publish(output);
      }
      if (value.fields_updated & (1 << 12)) {
        sensor_msgs::msg::Temperature output;
        output.header = header(stamp);
        output.temperature = value.temperature;
        temp_pub_->publish(output);
      }
    } else if (message.msgid == MAVLINK_MSG_ID_SCALED_IMU) {
      mavlink_scaled_imu_t value{};
      mavlink_msg_scaled_imu_decode(&message, &value);
      const auto stamp = core_.stamp();
      publish_raw(stamp,
        ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(Eigen::Vector3d(
          value.xgyro, value.ygyro, value.zgyro) * 1e-3),
        ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(Eigen::Vector3d(
          value.xacc, value.yacc, value.zacc) * 9.80665e-3));
      publish_mag(stamp, ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(Eigen::Vector3d(
        value.xmag, value.ymag, value.zmag) * 1e-7));
    }
  }

private:
  std_msgs::msg::Header header(const rclcpp::Time & stamp) const
  {
    std_msgs::msg::Header output;
    output.stamp = stamp;
    output.frame_id = frame_id_;
    return output;
  }

  void publish_attitude(uint32_t, const Eigen::Quaterniond & ned_aircraft, const Eigen::Vector3d & gyro_frd)
  {
    const auto orientation = ftf::transform_orientation_aircraft_baselink(
      ftf::transform_orientation_ned_enu(ned_aircraft));
    const auto gyro = ftf::transform_frame_aircraft_baselink(gyro_frd);
    sensor_msgs::msg::Imu output;
    output.header = header(core_.stamp());
    output.orientation = tf2::toMsg(orientation);
    tf2::toMsg(gyro, output.angular_velocity);
    output.linear_acceleration_covariance[0] = -1.0;
    {
      std::lock_guard<std::mutex> lock(core_.state().mutex);
      core_.state().orientation = output.orientation;
      core_.state().angular_velocity = output.angular_velocity;
      core_.state().have_attitude = true;
    }
    data_pub_->publish(output);
  }

  void publish_raw(const rclcpp::Time & stamp, const Eigen::Vector3d & gyro, const Eigen::Vector3d & accel)
  {
    sensor_msgs::msg::Imu output;
    output.header = header(stamp);
    tf2::toMsg(gyro, output.angular_velocity);
    tf2::toMsg(accel, output.linear_acceleration);
    output.orientation_covariance[0] = -1.0;
    raw_pub_->publish(output);
  }

  void publish_mag(const rclcpp::Time & stamp, const Eigen::Vector3d & field)
  {
    sensor_msgs::msg::MagneticField output;
    output.header = header(stamp);
    tf2::toMsg(field, output.magnetic_field);
    mag_pub_->publish(output);
  }

  std::string frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr data_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;
};

std::unique_ptr<Module> make_imu(MavrosLite & core)
{
  return std::make_unique<Imu>(core);
}
}  // namespace fss_px4_sim::mavros_lite
