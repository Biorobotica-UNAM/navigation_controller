cmake_minimum_required(VERSION 3.8)
project(nav_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(known_locations_interfaces REQUIRED)


# ===== Executable =====
add_executable(nav_controller_node
  src/nav_controller_node.cpp
)

ament_target_dependencies(nav_controller_node
  rclcpp
  geometry_msgs
  actionlib_msgs
  known_locations_interfaces
)

# ===== Install =====
install(TARGETS
  nav_controller_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()