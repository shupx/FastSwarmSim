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
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(std_msgs REQUIRED)

pkg_check_modules(ZMQ REQUIRED libzmq)

add_library(${PROJECT_NAME} SHARED
  src/multi_threaded_executor.cpp
  src/rate.cpp
  src/time_coordinator.cpp
  src/single_threaded_executor.cpp
  src/thread_time_participant.cpp
  src/sleep.cpp
  src/zeromq_time_participant_backend.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ZMQ_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} ${ZMQ_LIBRARIES})
target_compile_options(${PROJECT_NAME} PRIVATE ${ZMQ_CFLAGS_OTHER})
if(BUILD_TESTING)
  target_compile_definitions(${PROJECT_NAME} PUBLIC BUILD_TESTING)
endif()
ament_target_dependencies(${PROJECT_NAME}
  builtin_interfaces
  fss_time_interfaces
  rclcpp
  rosgraph_msgs
)

add_executable(time_coordinator src/time_coordinator_node.cpp)
target_link_libraries(time_coordinator ${PROJECT_NAME})
ament_target_dependencies(time_coordinator rclcpp fss_time_interfaces)

if(BUILD_TESTING)
  add_executable(sim_time_test_load_node test_cases/src/sim_time_test_load_node.cpp)
  target_link_libraries(sim_time_test_load_node ${PROJECT_NAME})
  ament_target_dependencies(
    sim_time_test_load_node
    builtin_interfaces
    rclcpp
    std_msgs
  )

  add_executable(executor_time_test_node test_cases/src/executor_time_test_node.cpp)
  target_link_libraries(executor_time_test_node ${PROJECT_NAME})
  ament_target_dependencies(
    executor_time_test_node
    rclcpp
    std_msgs
  )

  add_executable(fss_rate_test_node test_cases/src/fss_rate_test_node.cpp)
  target_link_libraries(fss_rate_test_node ${PROJECT_NAME})
  ament_target_dependencies(
    fss_rate_test_node
    rclcpp
    std_msgs
  )

  add_executable(timer_sleep_test_node test_cases/src/timer_sleep_test_node.cpp)
  target_link_libraries(timer_sleep_test_node ${PROJECT_NAME})
  ament_target_dependencies(
    timer_sleep_test_node
    rclcpp
    std_msgs
  )

  ament_add_gtest(test_fss_time test/test_fss_time.cpp)
  if(TARGET test_fss_time)
    target_link_libraries(test_fss_time ${PROJECT_NAME})
    ament_target_dependencies(test_fss_time rclcpp fss_time_interfaces rosgraph_msgs)
  endif()

  install(TARGETS
    sim_time_test_load_node
    executor_time_test_node
    fss_rate_test_node
    timer_sleep_test_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY
    test_cases
    DESTINATION share/${PROJECT_NAME}
  )
endif()

install(TARGETS
  ${PROJECT_NAME}
  time_coordinator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/time_coordinator_ui.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  scripts/ui
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)
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
