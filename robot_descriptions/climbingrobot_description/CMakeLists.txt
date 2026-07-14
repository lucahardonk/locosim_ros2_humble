cmake_minimum_required(VERSION 3.10)
project(climbingrobot_description)

find_package(ament_cmake REQUIRED)

# Install all resource directories so they are discoverable at runtime via
# get_package_share_directory('climbingrobot_description').
install(DIRECTORY
  config
  gazebo
  launch
  meshes
  media
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
