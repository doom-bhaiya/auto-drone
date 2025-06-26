cmake_minimum_required(VERSION 3.8)
project(drone_sim)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(drone_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(drone_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(drone_plugin SHARED
  src/drone_plugin.cpp
)

target_include_directories(drone_plugin PUBLIC include)
ament_target_dependencies(drone_plugin
  "gazebo_ros"
  "drone_msgs"
  "geometry_msgs"
)
ament_export_libraries(drone_plugin)

install(TARGETS
    drone_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)



install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


# Export old-style CMake variables
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)


ament_package()
