# Colcon sources package environments before configuring dependent packages.
# Keep the binary ROS resource index available in each CMake invocation.
set(_fss_ros_underlay "$ENV{FSS_ROS_UNDERLAY}")
if(NOT IS_DIRECTORY "${_fss_ros_underlay}/share/ament_index/resource_index")
  message(FATAL_ERROR "FSS_ROS_UNDERLAY must point to an installed ROS distribution")
endif()

string(REPLACE ":" ";" _fss_ament_prefixes "$ENV{AMENT_PREFIX_PATH}")
list(APPEND _fss_ament_prefixes "${_fss_ros_underlay}")
list(REMOVE_DUPLICATES _fss_ament_prefixes)
list(JOIN _fss_ament_prefixes ":" _fss_ament_prefix_path)
set(ENV{AMENT_PREFIX_PATH} "${_fss_ament_prefix_path}")
unset(_fss_ros_underlay)
unset(_fss_ament_prefixes)
unset(_fss_ament_prefix_path)
