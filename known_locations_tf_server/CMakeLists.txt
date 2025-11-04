cmake_minimum_required(VERSION 3.8)
project(known_locations_tf_server)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

ament_python_install_package(${PROJECT_NAME})

ament_package()




