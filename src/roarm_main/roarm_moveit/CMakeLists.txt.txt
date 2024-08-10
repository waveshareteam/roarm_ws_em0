cmake_minimum_required(VERSION 3.22)
project(roarm_moveit)

find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)

# Generate ROS interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetPoseCmd.srv"
  "srv/MovePointCmd.srv"
  "srv/MoveCircleCmd.srv"
)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
