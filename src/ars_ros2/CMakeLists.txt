cmake_minimum_required(VERSION 3.25)
project(ars_ros2 VERSION 0.5.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# ROS-agnostic core, consumed as a plain header-only target.
add_library(ars_core INTERFACE)
target_include_directories(ars_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../ars_core/include>
)

# --- conversions library (header-only) ---
add_library(ars_ros2_conversions INTERFACE)
target_include_directories(ars_ros2_conversions INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ars_ros2_conversions INTERFACE ars_core)
ament_target_dependencies(ars_ros2_conversions INTERFACE geometry_msgs nav_msgs)

# --- odometry lifecycle node ---
add_library(odometry_node SHARED src/odometry_node.cpp)
target_include_directories(odometry_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(odometry_node PUBLIC ars_core ars_ros2_conversions)
ament_target_dependencies(odometry_node PUBLIC
  rclcpp rclcpp_lifecycle nav_msgs sensor_msgs geometry_msgs)

add_executable(odometry_node_main src/odometry_node_main.cpp)
target_link_libraries(odometry_node_main PRIVATE odometry_node)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS odometry_node ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(TARGETS odometry_node_main RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_conversions test/test_conversions.cpp)
  target_link_libraries(test_conversions ars_ros2_conversions)
  ament_add_gtest(test_odometry_node test/test_odometry_node.cpp)
  target_link_libraries(test_odometry_node odometry_node)
endif()

ament_package()
