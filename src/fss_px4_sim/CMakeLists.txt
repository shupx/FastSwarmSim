cmake_minimum_required(VERSION 3.8)
project(fss_px4_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(fss_time REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(perfect_mavros_quadrotor_sim_node src/perfect_mavros_quadrotor_sim_node.cpp)
ament_target_dependencies(perfect_mavros_quadrotor_sim_node
  geometry_msgs
  fss_time
  mavros_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
)

install(TARGETS perfect_mavros_quadrotor_sim_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
