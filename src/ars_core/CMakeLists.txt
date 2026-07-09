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

  add_executable(test_vec2 test/geometry/test_vec2.cpp)
  target_link_libraries(test_vec2 PRIVATE ars_core GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(test_vec2)
endif()
