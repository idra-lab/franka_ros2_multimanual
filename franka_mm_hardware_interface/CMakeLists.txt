cmake_minimum_required(VERSION 3.8)
project(franka_mm_hardware_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Check system libraries that should be installed
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

find_package(Franka 0.13.2 REQUIRED)
find_package(franka_msgs REQUIRED)


# Used files to build the library
add_library(
  ${PROJECT_NAME} SHARED
  src/franka_mm_hardware_interface.cpp
  src/franka_wrapper.cpp
  src/lambda_control.cpp
  src/mode_switch_plan.cpp
  src/franka_param_service_server.cpp
  )
  
# Libraries used in the files
ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  franka_msgs
  Franka
  )
    
# Export include directories
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)
    
# Export the plugin if needed (for hardware interface)
pluginlib_export_plugin_description_file(hardware_interface franka_mm_hardware_interface.xml)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include/
)

# Install library
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

# Export the package
ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
)

ament_package()