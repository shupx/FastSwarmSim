cmake_minimum_required(VERSION 3.8)
project(fss_time)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(fss_time_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(eCAL REQUIRED CONFIG)

add_library(${PROJECT_NAME}
  src/distributed_time_core.cpp
  src/ecal_time_transport.cpp
  src/time_participant.cpp
  src/time_transport.cpp
  src/time_transport_serialization.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  builtin_interfaces
  fss_time_interfaces
  rclcpp
  rosgraph_msgs
)
if(TARGET eCAL::core)
  target_link_libraries(${PROJECT_NAME} eCAL::core)
elseif(TARGET eCAL::ecal_core)
  target_link_libraries(${PROJECT_NAME} eCAL::ecal_core)
elseif(TARGET eCAL::ecal)
  target_link_libraries(${PROJECT_NAME} eCAL::ecal)
else()
  target_link_libraries(${PROJECT_NAME} ecal_core)
endif()

add_executable(clock_bridge_node src/clock_bridge_node.cpp)
target_link_libraries(clock_bridge_node ${PROJECT_NAME})
ament_target_dependencies(clock_bridge_node rclcpp rosgraph_msgs fss_time_interfaces)

ament_add_gtest(test_distributed_time_core test/test_distributed_time_core.cpp)
if(TARGET test_distributed_time_core)
  target_link_libraries(test_distributed_time_core ${PROJECT_NAME})
endif()

ament_add_gtest(test_time_transport_serialization test/test_time_transport_serialization.cpp)
if(TARGET test_time_transport_serialization)
  target_link_libraries(test_time_transport_serialization ${PROJECT_NAME})
endif()

ament_add_gtest(test_ecal_time_transport test/test_ecal_time_transport.cpp)
if(TARGET test_ecal_time_transport)
  target_link_libraries(test_ecal_time_transport ${PROJECT_NAME})
endif()

install(TARGETS ${PROJECT_NAME} clock_bridge_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  builtin_interfaces
  fss_time_interfaces
  rclcpp
  rosgraph_msgs
)
ament_export_dependencies(eCAL)
ament_package()
