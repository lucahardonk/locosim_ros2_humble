cmake_minimum_required(VERSION 3.10)
project(go1_description)

find_package(ament_cmake REQUIRED)

# Install all resource directories so they are discoverable at runtime via
# get_package_share_directory('go1_description').
install(DIRECTORY
  config
  gazebo
  launch
  meshes
  robots
  rviz
  urdfs
  DESTINATION share/${PROJECT_NAME}
)

# Install the go0 helper script (chmod +x preserved via PROGRAMS)
install(PROGRAMS
  scripts/go0
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
