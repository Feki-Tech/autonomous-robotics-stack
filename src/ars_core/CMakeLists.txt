cmake_minimum_required(VERSION 3.25)
project(ars_core
  VERSION 0.1.0
  LANGUAGES CXX
  DESCRIPTION "Autonomous Robotics Stack — core libraries (pure C++20)"
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Header-only library (for now)
add_library(ars_core INTERFACE)
target_include_directories(ars_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Testing
include(CTest)
if(BUILD_TESTING)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
  )
  FetchContent_MakeAvailable(googletest)

  # Registers a unit test executable linked against ars_core and GTest.
  function(ars_add_test name source)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE ars_core GTest::gtest_main)
    gtest_discover_tests(${name})
  endfunction()

  include(GoogleTest)
  ars_add_test(test_vec2 test/geometry/test_vec2.cpp)
  ars_add_test(test_angle test/geometry/test_angle.cpp)
  ars_add_test(test_pose2 test/geometry/test_pose2.cpp)
  ars_add_test(test_twist2 test/geometry/test_twist2.cpp)
  ars_add_test(test_matrix test/math/test_matrix.cpp)
endif()
