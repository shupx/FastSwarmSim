#pragma once

#include <cmath>

#include <Eigen/Geometry>

namespace mavros::ftf
{
inline Eigen::Vector3d transform_frame_enu_ned(const Eigen::Vector3d & vector)
{
  return {vector.y(), vector.x(), -vector.z()};
}

inline Eigen::Vector3d transform_frame_ned_enu(const Eigen::Vector3d & vector)
{
  return transform_frame_enu_ned(vector);
}

inline Eigen::Vector3d transform_frame_baselink_aircraft(const Eigen::Vector3d & vector)
{
  return {vector.x(), -vector.y(), -vector.z()};
}

inline Eigen::Vector3d transform_frame_aircraft_baselink(const Eigen::Vector3d & vector)
{
  return transform_frame_baselink_aircraft(vector);
}

inline Eigen::Quaterniond transform_orientation_baselink_aircraft(const Eigen::Quaterniond & orientation)
{
  return orientation * Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0);
}

inline Eigen::Quaterniond transform_orientation_enu_ned(const Eigen::Quaterniond & orientation)
{
  const Eigen::Quaterniond enu_to_ned(0.0, M_SQRT1_2, M_SQRT1_2, 0.0);
  return enu_to_ned * orientation;
}
}  // namespace mavros::ftf
