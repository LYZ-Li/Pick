cmake_minimum_required(VERSION 3.8)
project(ur5e_workcell_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch config meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
