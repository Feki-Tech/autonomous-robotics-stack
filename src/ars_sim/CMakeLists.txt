cmake_minimum_required(VERSION 3.25)
project(ars_sim VERSION 0.6.0)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds config DESTINATION share/${PROJECT_NAME})

ament_package()
