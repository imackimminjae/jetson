cmake_minimum_required(VERSION 3.8)
project(imac_control_system)

# C++ 표준
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(imac_interfaces REQUIRED) 


# 메시지 파일 지정

# 메시지 생성



# include 디렉토리 
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

# 실행 노드 목록
add_executable(udp_publisher_node src/comm/udp_publisher_node.cpp)
add_executable(trajectory_plan_node src/application/trajectory_plan_node.cpp)
add_executable(vehicle_controller_node src/application/vehicle_controller_node.cpp)


# 메시지 의존성 포함



# ament 의존성 연결
ament_target_dependencies(udp_publisher_node
  rclcpp
  std_msgs
  imac_interfaces
)

ament_target_dependencies(trajectory_plan_node
  rclcpp
  nav_msgs
  visualization_msgs
  geometry_msgs
  Eigen3
  tf2
  tf2_geometry_msgs
  imac_interfaces
)
ament_target_dependencies(vehicle_controller_node
  rclcpp
  nav_msgs
  tf2
  tf2_geometry_msgs
  imac_interfaces
)



# install targets
install(TARGETS
  udp_publisher_node
  trajectory_plan_node
  vehicle_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# install launch, config, msg
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)


# 메시지 빌드 export
ament_export_dependencies(rosidl_default_runtime)

ament_package()
