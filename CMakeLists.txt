cmake_minimum_required(VERSION 2.8.3)
project(sergio_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  diagnostic_updater
  ethercat_interface
  hardware_interface
  joint_state_controller
  roscpp
  transmission_interface
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(transmission_manager src/transmission_manager.cpp)

target_link_libraries(transmission_manager
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
  src/sergio_hardware_interface.cpp
)

target_link_libraries(${PROJECT_NAME}_node transmission_manager ${catkin_LIBRARIES})

# Test
add_executable(test_transmission_manager test/test_transmission_manager.cpp)
target_link_libraries(test_transmission_manager transmission_manager)
