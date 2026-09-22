cmake_minimum_required(VERSION 3.8)
project(imac_interfaces)

# C++ 표준
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)

set(msg_files
  "msg/VirtualControlCommand.msg"
  "msg/PathWithArcLength.msg"
)

# 메시지 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
