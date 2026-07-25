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
find_package(std_msgs REQUIRED)

set(HELICS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(HELICS_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(HELICS_BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
set(HELICS_ENABLE_PACKAGE_BUILD OFF CACHE BOOL "" FORCE)
set(HELICS_BUILD_APP_LIBRARY OFF CACHE BOOL "" FORCE)
set(HELICS_BINARY_ONLY_INSTALL ON CACHE BOOL "" FORCE)
set(HELICS_DISABLE_ASIO ON CACHE BOOL "" FORCE)
set(HELICS_DISABLE_BOOST ON CACHE BOOL "" FORCE)
set(HELICS_DISABLE_GIT_OPERATIONS ON CACHE BOOL "" FORCE)
set(HELICS_DISABLE_VCPKG ON CACHE BOOL "" FORCE)
set(HELICS_USE_EXTERNAL_FMT ON CACHE BOOL "" FORCE)
set(HELICS_USE_EXTERNAL_SPDLOG ON CACHE BOOL "" FORCE)
set(HELICS_USE_SYSTEM_ZEROMQ_ONLY ON CACHE BOOL "" FORCE)
set(HELICS_ZMQ_FORCE_SUBPROJECT OFF CACHE BOOL "" FORCE)
set(HELICS_ZMQ_SUBPROJECT OFF CACHE BOOL "" FORCE)
set(GMLC_NETWORKING_TEST OFF CACHE BOOL "" FORCE)
set(FSS_TIME_PREVIOUS_SKIP_INSTALL_RULES ${CMAKE_SKIP_INSTALL_RULES})
set(CMAKE_SKIP_INSTALL_RULES TRUE)
add_subdirectory(third_party/HELICS EXCLUDE_FROM_ALL)
set(CMAKE_SKIP_INSTALL_RULES ${FSS_TIME_PREVIOUS_SKIP_INSTALL_RULES})

add_library(${PROJECT_NAME} SHARED
  src/helics_broker_backend.cpp
  src/helics_thread_participant_backend.cpp
  src/sim_time_broker.cpp
  src/thread_time_participant.cpp
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
if(TARGET HELICS::helics)
  set(FSS_TIME_HELICS_TARGET HELICS::helics)
elseif(TARGET HELICS::helicscpp)
  set(FSS_TIME_HELICS_TARGET HELICS::helicscpp)
else()
  set(FSS_TIME_HELICS_TARGET helics)
endif()
target_link_libraries(${PROJECT_NAME} ${FSS_TIME_HELICS_TARGET})
set_target_properties(${PROJECT_NAME} PROPERTIES INTERFACE_LINK_LIBRARIES "")

add_executable(ros_clock_publisher_node src/ros_clock_publisher_node.cpp)
target_link_libraries(ros_clock_publisher_node ${PROJECT_NAME})
ament_target_dependencies(ros_clock_publisher_node rclcpp rosgraph_msgs fss_time_interfaces)

add_executable(sim_time_broker_node src/sim_time_broker_node.cpp)
target_link_libraries(sim_time_broker_node ${PROJECT_NAME} ${FSS_TIME_HELICS_TARGET})
ament_target_dependencies(sim_time_broker_node rclcpp fss_time_interfaces)

add_executable(sim_time_test_load_node test_cases/src/sim_time_test_load_node.cpp)
target_link_libraries(sim_time_test_load_node ${PROJECT_NAME})
ament_target_dependencies(
  sim_time_test_load_node
  builtin_interfaces
  rclcpp
  std_msgs
)

ament_add_gtest(test_fss_time test/test_fss_time.cpp)
if(TARGET test_fss_time)
  target_link_libraries(test_fss_time ${PROJECT_NAME} ${FSS_TIME_HELICS_TARGET})
  ament_target_dependencies(test_fss_time rclcpp fss_time_interfaces)
endif()

install(TARGETS ${PROJECT_NAME} ros_clock_publisher_node sim_time_broker_node sim_time_test_load_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/sim_time_broker_ui.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  config
  test_cases
  DESTINATION share/${PROJECT_NAME}
)
if(TARGET helics)
  install(TARGETS helics
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib
  )
endif()
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  builtin_interfaces
  fss_time_interfaces
  rclcpp
  rosgraph_msgs
  std_msgs
)
ament_package()
