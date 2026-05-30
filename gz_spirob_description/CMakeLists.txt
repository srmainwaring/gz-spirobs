cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)
project(gz_spirob_description)

# --------------------------------------------------------------------------- #
# If ament_cmake is found build as an ament package, otherwise ignore.
# This is so the system may be built for Gazebo only, if ROS is not available.
find_package(ament_cmake QUIET)
if(${ament_cmake_FOUND})
  message("Building ${PROJECT_NAME} as an `ament_cmake` project.")
endif()

# --------------------------------------------------------------------------- #
# Install.

#install(
#  DIRECTORY
#  config/
#  DESTINATION share/${PROJECT_NAME}/config
#)

install(
  DIRECTORY
  models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(
  DIRECTORY
  worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
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
