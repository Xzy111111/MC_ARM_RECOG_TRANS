cmake_minimum_required(VERSION 3.8)
project(piper_control)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch config urdf DESTINATION share/${PROJECT_NAME})
