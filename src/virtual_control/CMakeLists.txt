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
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(imac_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(ament_index_cpp REQUIRED)
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

# Shared, ROS-independent PX4-local to KNU-map rigid transform. Keeping the
# alignment state here prevents upper/lower consumers from selecting different
# first PX4 samples as their origins.
add_library(px4_odom_map_transform
  src/px4_odom_map_transform.cpp
)
target_include_directories(px4_odom_map_transform PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# PX4 yaw-scalar PoseStamped -> map-frame quaternion Odometry bridge.
add_executable(px4_odom_map_bridge
  src/px4_odom_map_bridge_node.cpp
)
ament_target_dependencies(px4_odom_map_bridge
  rclcpp
  builtin_interfaces
  geometry_msgs
  nav_msgs
  std_srvs
)
target_include_directories(px4_odom_map_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(px4_odom_map_bridge px4_odom_map_transform)

# Egocentric OccupancyGrid + MIQP upper planner.
add_executable(upper_planner_node
  src/tracking_control.cpp
)
ament_target_dependencies(upper_planner_node
  imac_interfaces
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

# High-rate lower lateral QP-MPC + all actuator output backends.
add_executable(lower_tracking_mpc_node
  src/lower_tracking_mpc_node.cpp
)
ament_target_dependencies(lower_tracking_mpc_node
  rclcpp
  nav_msgs
  geometry_msgs
  imac_interfaces
  tf2
  std_msgs
  mavlink
  mpclib_vendor
)
target_include_directories(lower_tracking_mpc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
if(TARGET mpclib_vendor::MIQPLib)
  target_link_libraries(lower_tracking_mpc_node mpclib_vendor::MIQPLib)
elseif(TARGET MIQPLib)
  target_link_libraries(lower_tracking_mpc_node MIQPLib)
endif()
target_link_libraries(lower_tracking_mpc_node Threads::Threads)

#################################
# Install
#################################
install(
  TARGETS
    px4_odom_map_bridge
    upper_planner_node
    lower_tracking_mpc_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS px4_odom_map_transform
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  PROGRAMS
    scripts/px4_ekf_bridge.py
    scripts/knu_global_path_publisher.py
    scripts/knu_batch_runner.py
    scripts/knu_batch_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

# HILS controller headers.
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config data
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES
    local_map.csv
    local_map_1.csv
    local_map_2.csv
    local_map_3.csv
  DESTINATION share/${PROJECT_NAME}/data
)

#################################
# Testing
#################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_knu_global_path
    test/test_knu_global_path.py
  )
  ament_add_pytest_test(test_knu_batch
    test/test_knu_batch.py
  )
  ament_add_pytest_test(test_knu_batch_ros
    test/test_knu_batch_ros.py
  )
  set(ament_cmakecopyright_FOUND TRUE)
  set(ament_cmakecpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_px4_odom_map_transform
    test/test_px4_odom_map_transform.cpp
  )
  target_link_libraries(test_px4_odom_map_transform px4_odom_map_transform)
  target_include_directories(test_px4_odom_map_transform PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  ament_add_gtest(test_egocentric_planner_geometry
    test/test_egocentric_planner_geometry.cpp
  )
  target_include_directories(test_egocentric_planner_geometry PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${EIGEN3_INCLUDE_DIRS}
  )

  ament_add_gtest(test_interval_centering_state
    test/test_interval_centering_state.cpp
    src/tracking_control.cpp
  )
  target_compile_definitions(test_interval_centering_state PRIVATE
    VIRTUAL_CONTROL_UPPER_PLANNER_NO_MAIN
  )
  ament_target_dependencies(test_interval_centering_state
    rclcpp nav_msgs geometry_msgs imac_interfaces std_msgs tf2 ament_index_cpp mpclib_vendor
  )
  target_include_directories(test_interval_centering_state PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${EIGEN3_INCLUDE_DIRS}
  )
  if(TARGET mpclib_vendor::MIQPLib)
    target_link_libraries(test_interval_centering_state mpclib_vendor::MIQPLib)
  elseif(TARGET MIQPLib)
    target_link_libraries(test_interval_centering_state MIQPLib)
  endif()
  target_link_libraries(test_interval_centering_state Threads::Threads)

  ament_add_gtest(test_lower_path_generation
    test/test_lower_path_generation.cpp src/tracking_control.cpp)
  target_compile_definitions(test_lower_path_generation PRIVATE VIRTUAL_CONTROL_UPPER_PLANNER_NO_MAIN)
  ament_target_dependencies(test_lower_path_generation
    rclcpp nav_msgs geometry_msgs imac_interfaces std_msgs tf2 ament_index_cpp mpclib_vendor)
  target_include_directories(test_lower_path_generation PRIVATE include ${EIGEN3_INCLUDE_DIRS})
  if(TARGET mpclib_vendor::MIQPLib)
    target_link_libraries(test_lower_path_generation mpclib_vendor::MIQPLib)
  elseif(TARGET MIQPLib)
    target_link_libraries(test_lower_path_generation MIQPLib)
  endif()
  target_link_libraries(test_lower_path_generation Threads::Threads)
  set_tests_properties(test_lower_path_generation PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=175;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=/tmp/lower_mpc_test_logs")

  ament_add_gtest(test_lower_tracking_mpc test/test_lower_tracking_mpc.cpp)
  target_compile_definitions(test_lower_tracking_mpc PRIVATE
    VIRTUAL_CONTROL_LOWER_MPC_NO_MAIN VIRTUAL_CONTROL_LOWER_MPC_TESTING)
  ament_target_dependencies(test_lower_tracking_mpc
    rclcpp nav_msgs geometry_msgs imac_interfaces tf2 std_msgs mavlink mpclib_vendor)
  target_include_directories(test_lower_tracking_mpc PRIVATE include ${EIGEN3_INCLUDE_DIRS})
  if(TARGET mpclib_vendor::MIQPLib)
    target_link_libraries(test_lower_tracking_mpc mpclib_vendor::MIQPLib)
  elseif(TARGET MIQPLib)
    target_link_libraries(test_lower_tracking_mpc MIQPLib)
  endif()
  target_link_libraries(test_lower_tracking_mpc Threads::Threads)
  set_tests_properties(test_lower_tracking_mpc PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=174;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=/tmp/lower_mpc_test_logs")

  ament_add_gtest(test_lower_steering_actuator_model
    test/test_lower_steering_actuator_model.cpp
  )
  target_include_directories(test_lower_steering_actuator_model PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  ament_add_gtest(test_miqp_constraint_enforcement
    test/test_miqp_constraint_enforcement.cpp
  )
  target_include_directories(test_miqp_constraint_enforcement PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  if(TARGET mpclib_vendor::MIQPLib)
    target_link_libraries(test_miqp_constraint_enforcement mpclib_vendor::MIQPLib)
  elseif(TARGET MIQPLib)
    target_link_libraries(test_miqp_constraint_enforcement MIQPLib)
  endif()
endif()

ament_package()
