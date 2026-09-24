cmake_minimum_required(VERSION 3.8)
project(robocraft_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch models rviz urdf worlds
  DESTINATION share/${PROJECT_NAME})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")
ament_package()
