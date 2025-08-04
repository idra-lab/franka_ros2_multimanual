cmake_minimum_required(VERSION 3.8)
project(idra_franka_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/gear_pick.py
    scripts/coffee_tray.py
    scripts/gripper_grasp_test.py
    scripts/gripper_homing_test.py
    scripts/stiffness_test.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch urdf rviz
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
