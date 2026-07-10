cmake_minimum_required(VERSION 3.8)
project(imac_interfaces)

# C++ 표준
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
  "msg/FsmMode.msg"
  "msg/GlobalPath.msg"
  "msg/HilsObstacles.msg"
  "msg/Obstacles.msg"
  "msg/VehicleStatusMav.msg"
  "msg/VehicleStatusHils.msg"
  "msg/VirtualControlCommand.msg"
)

# 메시지 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
