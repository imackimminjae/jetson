cmake_minimum_required(VERSION 3.8)
project(mpclib_vendor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_subdirectory(third_party/miqplib)

set(_miqrlib_target "")
if(TARGET miqplib)
  set(_miqplib_target miqplib)
elseif(TARGET MIQPLib)
  set(_miqplib_target MIQPLib)
else()
  message(FATAL_ERROR "miqplib target name not found. Check miqp CMakeLists.")
endif()

get_target_property(_aliased ${_miqplib_target} ALIASED_TARGET)
if(_aliased)
  set(_miqplib_install_target ${_aliased})
else()
  set(_miqplib_install_target ${_miqplib_target})
endif()

set(MIQPLIB_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/miqplib/src")
file(GLOB MIQPLIB_SOURCES
  "${MIQPLIB_SRC_DIR}/*.cc"
  "${MIQPLIB_SRC_DIR}/*.cpp"
)

add_library(mpclib_vendor_miqplib INTERFACE)
target_link_libraries(mpclib_vendor_miqplib INTERFACE ${_miqplib_target})

target_include_directories(MIQPLib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/daqp/include>
  $<BUILD_INTERFACE:${MIQPLIB_SRC_DIR}>
  $<INSTALL_INTERFACE:include/miqplib>
  $<INSTALL_INTERFACE:include>
)


install(DIRECTORY third_party/miqplib/src/
  DESTINATION include/miqplib
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hh" PATTERN "*.hpp"
)
install(DIRECTORY third_party/miqplib/src/daqp/include/
  DESTINATION include
)

install(TARGETS ${_miqplib_install_target}
  EXPORT mpclib_vendor_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS MIQPLib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


ament_export_libraries(MIQPLib)
install(DIRECTORY ${MIQPLIB_SRC_DIR}/
  DESTINATION include/miqplib
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hh"
)

# daqp도 필요하면 같이 export 
if(TARGET daqp)
  install(TARGETS daqp
    EXPORT mpclib_vendor_targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endif()
if(TARGET daqpstat)
  install(TARGETS daqpstat
    EXPORT mpclib_vendor_targets
    ARCHIVE DESTINATION lib
  )
endif()

# 인터페이스 타겟(export용)
install(TARGETS mpclib_vendor_miqplib
  EXPORT mpclib_vendor_targets
)

install(EXPORT mpclib_vendor_targets
  NAMESPACE mpclib_vendor::
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_targets(mpclib_vendor_targets HAS_LIBRARY_TARGET)
ament_package()
