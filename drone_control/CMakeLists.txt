cmake_minimum_required(VERSION 3.8)
project(drone_control)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)

ament_export_include_directories(
  include
)

install(DIRECTORY include/
  DESTINATION include
)

include_directories(include)

# Export old-style CMake variables
ament_export_include_directories(
  include
)

add_executable(altitude_control_pid src/altitude_control_PID.cpp src/PID_controller.cpp)
ament_target_dependencies(altitude_control_pid rclcpp ament_index_cpp nav_msgs ros_gz_interfaces)

target_link_libraries(altitude_control_pid
  Eigen3::Eigen
  nlohmann_json::nlohmann_json
)

install(TARGETS
  altitude_control_pid
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)



ament_package()
