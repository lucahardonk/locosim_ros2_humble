cmake_minimum_required(VERSION 3.8)
project(base_hardware_interface)

# ROS2 (Humble) port of the WoLF base hardware interface.

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(urdf REQUIRED)
find_package(srdfdom REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(base_robot_hw SHARED src/base_robot_hw.cpp)
target_include_directories(base_robot_hw PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(base_robot_hw
  rclcpp
  hardware_interface
  pluginlib
  urdf
  srdfdom
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS base_robot_hw
  EXPORT export_base_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(base_robot_hw)
ament_export_dependencies(rclcpp hardware_interface pluginlib urdf srdfdom)

ament_package()
