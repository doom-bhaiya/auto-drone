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
find_package(spdlog REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)

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

add_executable(altitude_control_pid src/altitude_control_PID.cpp src/PID_controller.cpp src/Drone.cpp)
ament_target_dependencies(altitude_control_pid rclcpp ament_index_cpp nav_msgs ros_gz_interfaces)

add_executable(full_control_pid src/full_control_PID.cpp src/PID_controller.cpp src/Drone.cpp)
ament_target_dependencies(full_control_pid rclcpp ament_index_cpp nav_msgs ros_gz_interfaces tf2 tf2_ros tf2_geometry_msgs geometry_msgs)

target_link_libraries(altitude_control_pid
  Eigen3::Eigen
  nlohmann_json::nlohmann_json
  spdlog::spdlog
)

target_link_libraries(full_control_pid
  Eigen3::Eigen
  nlohmann_json::nlohmann_json
  spdlog::spdlog
)

install(TARGETS
  altitude_control_pid
  full_control_pid
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)



ament_package()
