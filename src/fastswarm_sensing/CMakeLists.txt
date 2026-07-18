cmake_minimum_required(VERSION 3.8)
project(fastswarm_sensing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(marsim_render REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(local_pointcloud_sim_node src/local_pointcloud_sim.cpp)
target_include_directories(local_pointcloud_sim_node PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(local_pointcloud_sim_node
  ament_index_cpp
  geometry_msgs
  marsim_render
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
)
target_link_libraries(local_pointcloud_sim_node ${PCL_LIBRARIES})

install(TARGETS local_pointcloud_sim_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
