cmake_minimum_required(VERSION 3.8)
project(ur5e_cell_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf rviz config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
