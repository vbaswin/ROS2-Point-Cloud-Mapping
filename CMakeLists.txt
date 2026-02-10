cmake_minimum_required(VERSION 3.8)
project(minimal_mapping_tool)

# Standard C++17 is required for ROS2 Humble
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler warnings for maintainable code
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find ROS2 Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# 2. Find System Dependencies (Qt5 & VTK/PCL)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui OpenGL)
find_package(PCL 1.12 REQUIRED)
find_package(VTK 9.1 REQUIRED)

# 3. Qt Automation (MOC, UIC)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# 4. Include Paths
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
)

# 5. Define Executable
add_executable(${PROJECT_NAME}
  src/main.cpp
  src/main_window.cpp
  src/ros_node.cpp
  src/cloud_mapper.cpp
  include/minimal_mapping_tool/main_window.hpp
  include/minimal_mapping_tool/ros_node.hpp
  include/minimal_mapping_tool/cloud_mapper.hpp
)

# 6. Link Libraries
target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Widgets
  Qt5::Gui
  Qt5::OpenGL
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
)
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/src/
    ${CMAKE_CURRENT_SOURCE_DIR}/src/
)


# Fix for PCL/VTK integration on Ubuntu 22.04
add_definitions(-DVTK_QT_VERSION=5)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})



# ============================================================
# TESTING (Google Test)
# ============================================================
# enable_testing() activates CTest, CMake's built-in test runner.
# This lets you run tests via `ctest` command, which integrates
# with CI/CD systems (GitHub Actions, Jenkins, etc.)
enable_testing()
find_package(GTest REQUIRED)


# Test executable for CloudMapper
# Note: We only link against PCL and GTest — NOT Qt or ROS2.
# This proves CloudMapper has proper separation of concerns (SRP).
add_executable(test_cloud_mapper
      tests/test_cloud_mapper.cpp
      src/cloud_mapper.cpp                          # the code under test
      include/minimal_mapping_tool/cloud_mapper.hpp
)

target_include_directories(test_cloud_mapper PRIVATE
include
${PCL_INCLUDE_DIRS}
)

target_link_libraries(test_cloud_mapper
    GTest::gtest
    GTest::gtest_main  # provides main() so you don't write it yourself
    ${PCL_LIBRARIES}
)
add_test(NAME CloudMapperTests COMMAND test_cloud_mapper)


ament_package()
