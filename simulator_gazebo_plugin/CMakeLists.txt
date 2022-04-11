cmake_minimum_required(VERSION 3.5)

project(simulator_gazebo_plugin)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(simulator_msgs REQUIRED)
find_package(nav_msgs)
find_package(autoware_auto_vehicle_msgs)

add_library(simulator_gazebo_plugin SHARED src/simulator_gazebo_plugin.cpp)

target_include_directories(simulator_gazebo_plugin PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_target_dependencies(simulator_gazebo_plugin
        "gazebo_ros"
        "rclcpp"
        "sensor_msgs"
        "tf2_ros"
        "tf2"
        "simulator_msgs"
        "nav_msgs"
        "autoware_auto_vehicle_msgs")

install(TARGETS
        simulator_gazebo_plugin
        DESTINATION share/${PROJECT_NAME})

if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
