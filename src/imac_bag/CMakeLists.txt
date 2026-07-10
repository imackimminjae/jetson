cmake_minimum_required(VERSION 3.8)
project(imac_bag)

# C++ 표준
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)



# 런치 파일 설치
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
