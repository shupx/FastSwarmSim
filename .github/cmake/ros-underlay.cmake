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

# Restore the ROS Python modules as well: package environment hooks can lose
# the underlay PYTHONPATH before CMake invokes ament's Python helpers.
file(GLOB _fss_ros_python_paths LIST_DIRECTORIES true
  "${_fss_ros_underlay}/lib/python*/site-packages"
  "${_fss_ros_underlay}/local/lib/python*/dist-packages")
string(REPLACE ":" ";" _fss_python_paths "$ENV{PYTHONPATH}")
list(APPEND _fss_python_paths ${_fss_ros_python_paths})
list(REMOVE_DUPLICATES _fss_python_paths)
list(JOIN _fss_python_paths ":" _fss_python_path)
set(ENV{PYTHONPATH} "${_fss_python_path}")
unset(_fss_ros_python_paths)
unset(_fss_python_paths)
unset(_fss_python_path)
unset(_fss_ros_underlay)
unset(_fss_ament_prefixes)
unset(_fss_ament_prefix_path)
