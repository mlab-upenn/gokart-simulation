cmake_minimum_required(VERSION 3.5)

project(simulator)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Ensure Gazebo env variables are set
#
# See https://gazebosim.org/tutorials?tut=components#EnvironmentVariables
# > If you want to modify Gazebo's behavior, e.g., by extending the path it searches for models,
#   you should first source the shell script listed above, then modify the variables that it sets.
#
# The /usr/share/gazebo/setup.bash is NOT sourced automatically.
#
# Our custom paths are added using the gazebo_ros_paths.py script (which is a part of gazebo_ros package).
# In order to ensure correct working we need to source /usr/share/gazebo/setup.bash first.
# See also https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Gazebo-ROS-Paths.
#
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh.in")

install(PROGRAMS
        scripts/convert_gnss_to_xyz.py
        scripts/gen_racetrack.py
        scripts/send_map.py
        scripts/spawn_cones.py
        DESTINATION lib/${PROJECT_NAME})

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  config
  launch
  materials
  meshes
  models
  rviz
  urdf
  worlds
)
