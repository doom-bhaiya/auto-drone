cmake_minimum_required(VERSION 3.8)
project(drone_sim)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(nlohmann_json REQUIRED)


install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

include_directories(include)

# Export old-style CMake variables
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)



        
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

find_package(ignition-gazebo6 REQUIRED)

add_library(DronePlugin SHARED src/DronePlugin.cc)
set_property(TARGET DronePlugin PROPERTY CXX_STANDARD 17)


target_link_libraries(DronePlugin
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo6::core
  PRIVATE nlohmann_json::nlohmann_json)


install(TARGETS
    DronePlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
