cmake_minimum_required(VERSION 3.8)
project(virtual_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#################################
# find dependencies
#################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(imac_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)   # ★ 추가
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mavlink REQUIRED)
find_package(mpclib_vendor REQUIRED)
find_package(Threads REQUIRED)

# Eigen
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

#################################
# Targets
#################################

# 1) Ackermann -> Control 변환 노드
add_executable(ackermann_to_control_node
  src/ackermann_to_control_node.cpp
)
ament_target_dependencies(ackermann_to_control_node
  rclcpp
  nav_msgs
  ackermann_msgs
  imac_interfaces
)

# 2) Odom + Path -> Ackermann (Stanley) 노드
add_executable(leatherback_stanley_node
  src/leatherback_stanley_node.cpp
)
ament_target_dependencies(leatherback_stanley_node
  rclcpp
  nav_msgs
  ackermann_msgs
)

# 3) CSV -> Path 퍼블리셔 노드
add_executable(csv_path_publisher_node
  src/csv_path_publisher_node.cpp
)
ament_target_dependencies(csv_path_publisher_node
  rclcpp
  nav_msgs
  imac_interfaces
)

# 4) GlobalPath + Obstacles → LocalCurve (Trajectory Planner)
add_executable(trajectory_plan_node
  src/trajectory_plan_node.cpp
)
ament_target_dependencies(trajectory_plan_node
  rclcpp
  nav_msgs
  imac_interfaces
)
target_include_directories(trajectory_plan_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# 4-1) Upper Planner Node
add_executable(upper_planner_node
  src/upper_planner_node.cpp
)
ament_target_dependencies(upper_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  ament_index_cpp
  mpclib_vendor
)
target_include_directories(upper_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
if(TARGET mpclib_vendor::MIQPLib)
  target_link_libraries(upper_planner_node mpclib_vendor::MIQPLib)
elseif(TARGET MIQPLib)
  target_link_libraries(upper_planner_node MIQPLib)
endif()
target_link_libraries(upper_planner_node Threads::Threads)

# 5) Tracking / Vehicle Controller Node
#    - src/tracking_controller_node.cpp 안에서
#      imac_ctrl::VehicleControllerNode 를 만들든, TrackingControllerNode를 만들든 상관 없음
add_executable(tracking_controller_node
  src/tracking_controller_node.cpp
)
ament_target_dependencies(tracking_controller_node
  rclcpp
  nav_msgs
  geometry_msgs
  imac_interfaces
  tf2
  sensor_msgs        # ★ Joy 메시지 때문에 추가
  std_msgs
)
target_include_directories(tracking_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# 6) Tracking Control Node (MIQP)
set(TRACKING_CONTROL_SOURCE src/tracking_control.cpp)
set(TRACKING_CONTROL_PATCH_B64
  "${CMAKE_CURRENT_SOURCE_DIR}/.codex_tracking_control_split.patch.gz.b64")
if(EXISTS "${TRACKING_CONTROL_PATCH_B64}")
  set(TRACKING_CONTROL_GENERATED_ROOT
    "${CMAKE_CURRENT_BINARY_DIR}/generated_tracking")
  set(TRACKING_CONTROL_GENERATED_SOURCE
    "${TRACKING_CONTROL_GENERATED_ROOT}/virtual_control/src/tracking_control.cpp")
  set(TRACKING_CONTROL_GENERATED_PATCH
    "${TRACKING_CONTROL_GENERATED_ROOT}/tracking_control_split.patch")

  file(MAKE_DIRECTORY
    "${TRACKING_CONTROL_GENERATED_ROOT}/virtual_control/src")
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/src/tracking_control.cpp"
    "${TRACKING_CONTROL_GENERATED_SOURCE}"
    COPYONLY)
  execute_process(
    COMMAND /bin/sh -c
      "base64 -d '${TRACKING_CONTROL_PATCH_B64}' | gzip -d > '${TRACKING_CONTROL_GENERATED_PATCH}' && cd '${TRACKING_CONTROL_GENERATED_ROOT}' && git apply --unidiff-zero '${TRACKING_CONTROL_GENERATED_PATCH}'"
    RESULT_VARIABLE TRACKING_CONTROL_PATCH_RESULT)
  if(NOT TRACKING_CONTROL_PATCH_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to apply tracking_control split patch")
  endif()
  set(TRACKING_CONTROL_SOURCE "${TRACKING_CONTROL_GENERATED_SOURCE}")
endif()

add_executable(tracking_control_node
  ${TRACKING_CONTROL_SOURCE}
)
ament_target_dependencies(tracking_control_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  std_msgs
  mpclib_vendor
)
target_include_directories(tracking_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
if(TARGET mpclib_vendor::MIQPLib)
  target_link_libraries(tracking_control_node mpclib_vendor::MIQPLib)
elseif(TARGET MIQPLib)
  target_link_libraries(tracking_control_node MIQPLib)
endif()
target_link_libraries(tracking_control_node Threads::Threads)

# 6-1) Lower duty / MAVLink output node
add_executable(lower_duty_node
  src/lower_duty_node.cpp
)
ament_target_dependencies(lower_duty_node
  rclcpp
  geometry_msgs
  imac_interfaces
  std_msgs
  mavlink
)
target_include_directories(lower_duty_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(lower_duty_node Threads::Threads)

# 7) Mocap straight PWM MAVLink test node
add_executable(straight_mocap_pwm_node
  src/straight_mocap_pwm_node.cpp
)
ament_target_dependencies(straight_mocap_pwm_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_srvs
  mavlink
)
target_include_directories(straight_mocap_pwm_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(straight_mocap_pwm_node Threads::Threads)

# Backward-compatible executable name for the previous bench-test target.
add_executable(mc_test_node
  src/straight_mocap_pwm_node.cpp
)
ament_target_dependencies(mc_test_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_srvs
  mavlink
)
target_include_directories(mc_test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mc_test_node Threads::Threads)

#################################
# Install
#################################
install(
  TARGETS
    ackermann_to_control_node
    leatherback_stanley_node
    csv_path_publisher_node
    trajectory_plan_node
    upper_planner_node
    tracking_controller_node
    tracking_control_node
    lower_duty_node
    straight_mocap_pwm_node
    mc_test_node
  DESTINATION lib/${PROJECT_NAME}
)

# 헤더(trajectory_plan_node.hpp, tracking_controller_node.hpp, vehicle_controller_node.hpp 등)
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config data
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES local_map.csv
  DESTINATION share/${PROJECT_NAME}/data
)

#################################
# Testing
#################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmakecopyright_FOUND TRUE)
  set(ament_cmakecpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
