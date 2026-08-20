cmake_minimum_required(VERSION 3.8)
project(fss_px4_sim)

set(CMAKE_BUILD_TYPE Release)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(fss_time REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(mavros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

install(PROGRAMS scripts/keyboard_control_ros2.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME keyboard_control_ros2
)

set(PX4_CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/px4_core)

add_library(fss_mavros_lite SHARED
  src/mavros_lite/core.cpp
  src/mavros_lite/setpoint_raw.cpp
  src/mavros_lite/local_position.cpp
  src/mavros_lite/imu.cpp
  src/mavros_lite/sys_status.cpp
  src/mavros_lite/command.cpp
  src/mavros_lite/global_position.cpp
)
target_include_directories(fss_mavros_lite PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_options(fss_mavros_lite PRIVATE -Wno-address-of-packed-member -Wno-pedantic)
ament_target_dependencies(fss_mavros_lite
  geographic_msgs geometry_msgs mavros mavros_msgs nav_msgs rclcpp sensor_msgs std_msgs tf2
  tf2_eigen tf2_geometry_msgs
)

add_library(fss_mavros_lite_udp SHARED
  src/mavros_lite/udp_transport.cpp
)
target_include_directories(fss_mavros_lite_udp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_options(fss_mavros_lite_udp PRIVATE -Wno-address-of-packed-member -Wno-pedantic)
ament_target_dependencies(fss_mavros_lite_udp rclcpp)

add_library(fss_px4_core
  ${PX4_CORE_DIR}/quadrotor_dynamics.cpp
  ${PX4_CORE_DIR}/px4_sitl.cpp
  ${PX4_CORE_DIR}/px4_modules/px4_lib/drivers/drv_hrt.cpp
  ${PX4_CORE_DIR}/px4_modules/px4_lib/hysteresis/hysteresis.cpp
  ${PX4_CORE_DIR}/px4_modules/px4_lib/geo/geo.cpp
  ${PX4_CORE_DIR}/px4_modules/px4_lib/parameters/px4_parameters.cpp
  ${PX4_CORE_DIR}/px4_modules/px4_lib/uORB/uORB_sim.cpp
  ${PX4_CORE_DIR}/px4_modules/mc_pos_control/PositionControl/ControlMath.cpp
  ${PX4_CORE_DIR}/px4_modules/mc_pos_control/PositionControl/PositionControl.cpp
  ${PX4_CORE_DIR}/px4_modules/mc_pos_control/Takeoff/Takeoff.cpp
  ${PX4_CORE_DIR}/px4_modules/mc_pos_control/MulticopterPositionControl.cpp
  ${PX4_CORE_DIR}/px4_modules/mc_att_control/AttitudeControl/AttitudeControl.cpp
  ${PX4_CORE_DIR}/px4_modules/mc_att_control/mc_att_control_main.cpp
  ${PX4_CORE_DIR}/px4_modules/commander/Commander.cpp
  ${PX4_CORE_DIR}/px4_modules/commander/state_machine_helper.cpp
  ${PX4_CORE_DIR}/px4_modules/mavlink/mavlink_main.cpp
  ${PX4_CORE_DIR}/px4_modules/mavlink/mavlink_udp_transport.cpp
  ${PX4_CORE_DIR}/px4_modules/mavlink/mavlink_streamer.cpp
  ${PX4_CORE_DIR}/px4_modules/mavlink/mavlink_receiver.cpp
)
set_target_properties(fss_px4_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(fss_px4_core PUBLIC
  include
  ${PX4_CORE_DIR}
  ${PX4_CORE_DIR}/px4_modules/px4_lib
)
target_compile_options(fss_px4_core PRIVATE -Wno-address-of-packed-member -Wno-cast-align)
target_link_libraries(fss_px4_core fss_mavros_lite)
ament_target_dependencies(fss_px4_core
  geographic_msgs
  geometry_msgs
  mavros_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

add_library(fss_px4_sim_components SHARED
  src/px4_rotor_sim_node.cpp
  src/drone_visualizer_node.cpp
  src/perfect_mavros_quadrotor_sim_node.cpp
  src/mavros_lite/mavros_lite_component.cpp
)
target_include_directories(fss_px4_sim_components PRIVATE include)
ament_target_dependencies(fss_px4_sim_components
  fss_time
  geometry_msgs
  mavros_msgs
  nav_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)
target_link_libraries(fss_px4_sim_components fss_px4_core fss_mavros_lite_udp)
rclcpp_components_register_node(fss_px4_sim_components
  PLUGIN "fss_px4_sim::MavrosPx4QuadrotorSim"
  EXECUTABLE px4_rotor_sim_node
  EXECUTOR MultiThreadedExecutor # since may include mavros_lite modules
)
rclcpp_components_register_node(fss_px4_sim_components
  PLUGIN "fss_px4_sim::mavros_lite::MavrosLiteUdp"
  EXECUTABLE mavros_lite_node
  EXECUTOR MultiThreadedExecutor # since mavros_lite may include blocked callbacks
)
rclcpp_components_register_node(fss_px4_sim_components
  PLUGIN "fss_px4_sim::DroneVisualizer"
  EXECUTABLE px4_rotor_visualizer_node
  EXECUTOR SingleThreadedExecutor
)
rclcpp_components_register_node(fss_px4_sim_components
  PLUGIN "fss_px4_sim::PerfectMavrosDrone"
  EXECUTABLE perfect_mavros_quadrotor_sim_node
  EXECUTOR SingleThreadedExecutor
)

install(TARGETS
  fss_mavros_lite
  fss_mavros_lite_udp
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS fss_px4_sim_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config model rviz DESTINATION share/${PROJECT_NAME})
install(FILES MAVROS_LITE.md DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs mavros mavros_msgs nav_msgs rclcpp rclcpp_components sensor_msgs std_msgs tf2 tf2_eigen
  tf2_geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_mavlink_udp_bridge test/test_mavlink_udp_bridge.cpp)
  if(TARGET test_mavlink_udp_bridge)
    target_link_libraries(test_mavlink_udp_bridge fss_px4_core)
    ament_target_dependencies(test_mavlink_udp_bridge rclcpp)
  endif()
  ament_add_pytest_test(test_keyboard_control_ros2 test/test_keyboard_control_ros2.py)
endif()

ament_package()
