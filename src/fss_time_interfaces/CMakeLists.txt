cmake_minimum_required(VERSION 3.8)
project(fss_time_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TimeIntent.msg"
  "msg/TimeControl.msg"
  "msg/TimeRequest.msg"
  "srv/ClientRegister.srv"
  "srv/ClientUnregister.srv"
  "srv/SimClockControl.srv"
  DEPENDENCIES builtin_interfaces
)

ament_package()
