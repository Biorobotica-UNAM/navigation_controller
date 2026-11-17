cmake_minimum_required(VERSION 3.8)
project(navigation_start)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch config maps rviz
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(launch_configuration test/test_launch_configuration.py TIMEOUT 30
    ENV "ROS_LOG_DIR=/tmp/navigation_launch_logs")
  ament_add_pytest_test(planning_maps test/test_planning_maps.py TIMEOUT 90
    ENV "ROS_LOCALHOST_ONLY=1" "ROS_DOMAIN_ID=174" "ROS_LOG_DIR=/tmp/navigation_planning_ros_logs")
endif()
ament_package()
