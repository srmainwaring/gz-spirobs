cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)
project(gz_spirob_gazebo)

# --------------------------------------------------------------------------- #
# If ament_cmake is found build as an ament package, otherwise ignore.
# This is so the system may be built for Gazebo only, if ROS is not available.
find_package(ament_cmake QUIET)
if(${ament_cmake_FOUND})
  message("Building ${PROJECT_NAME} as an `ament_cmake` project.")
endif()

# --------------------------------------------------------------------------- #
# Compile as C++17.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --------------------------------------------------------------------------- #
# Find gz-sim and dependencies.

if("$ENV{GZ_VERSION}" STREQUAL "jetty" OR NOT DEFINED "ENV{GZ_VERSION}")
  # Jetty (default)
  find_package(gz-cmake REQUIRED)
  set(GZ_CMAKE_VER "")
  
  gz_find_package(gz-sim REQUIRED)
  set(GZ_SIM_VER "")

  message(STATUS "Compiling against Gazebo Jetty")
else()  
  message(FATAL_ERROR "Unsupported GZ_VERSION: $ENV{GZ_VERSION}")
endif()

# --------------------------------------------------------------------------- #
# Build plugin.

add_library(gz-sim-tendon-system
    SHARED
    src/tendon/Tendon.cc
)
target_include_directories(gz-sim-tendon-system PRIVATE
  include
)
target_link_libraries(gz-sim-tendon-system PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)

# --------------------------------------------------------------------------- #
# Install.

install(
  TARGETS
  gz-sim-tendon-system
  DESTINATION lib/${PROJECT_NAME}
)

# --------------------------------------------------------------------------- #
# Register as an ament package if ament_cmake is available.
if(${ament_cmake_FOUND})
  ament_environment_hooks(
    "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
  ament_environment_hooks(
    "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

  ament_package()
endif()
