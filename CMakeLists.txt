cmake_minimum_required(VERSION 3.8)
project(roborock_s5max)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
