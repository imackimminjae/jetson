cmake_minimum_required(VERSION 3.8)
project(launch_pkg)

find_package(ament_cmake REQUIRED)

# launch 디렉토리 설치 (중요)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

