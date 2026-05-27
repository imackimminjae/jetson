cmake_minimum_required(VERSION 3.8)
project(virtual_control_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# launch 디렉토리 설치만 해주면 됨
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

