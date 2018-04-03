cmake_minimum_required(VERSION 2.8.3)
project(sergio_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  diagnostic_updater
  ethercat_interface
  hardware_interface
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

add_library(ethercat_hardware_interface src/ethercat_hardware_interface.cpp)
target_link_libraries(ethercat_hardware_interface
  transmission_manager
  ${catkin_LIBRARIES}
)

# Test
add_executable(test_transmission_manager test/test_transmission_manager.cpp)
target_link_libraries(test_transmission_manager transmission_manager)

add_executable(test_actuator test/test_actuator.cpp)
target_link_libraries(test_actuator ${catkin_LIBRARIES})

add_executable(test_ethercat_hardware_interface test/test_ethercat_hardware_interface.cpp)
target_link_libraries(test_ethercat_hardware_interface ethercat_hardware_interface ${catkin_LIBRARIES})
