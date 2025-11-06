cmake_minimum_required(VERSION 3.8)
project(known_locations_interfaces)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)  # Add this line

# Generate interfaces with dependencies
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LocationsServer.srv"
  "srv/LocationsGet.srv"
  DEPENDENCIES geometry_msgs  # Add this line
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(geometry_msgs)  # Add this line

ament_export_include_directories(include)
ament_package()