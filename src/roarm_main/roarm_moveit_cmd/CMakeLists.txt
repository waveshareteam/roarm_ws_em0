cmake_minimum_required(VERSION 3.22)
project(roarm_moveit_cmd)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(roarm_moveit REQUIRED)
find_package(ompl REQUIRED)
find_package(pluginlib REQUIRED)
find_package(trajectory_msgs REQUIRED)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ompl
  moveit_visual_tools
  ament_cmake
  rclcpp
  rclcpp_action
  tf2_geometry_msgs
  tf2_ros
  moveit_core
  moveit_ros_planning_interface
  interactive_markers
  moveit_ros_planning
  pluginlib
  Eigen3
  Boost
  control_msgs
  moveit_servo
  geometry_msgs
  roarm_moveit
  moveit_msgs
  trajectory_msgs
  pluginlib
)

# Include directories
include_directories(
  include
  
  ${ament_INCLUDE_DIRS}
  ${THIS_PACKAGE_INCLUDE_DIRS}
 
)

# Declare dependencies
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(include)

# Define executables
add_executable(getposecmd_tf2 
    src/getposecmd_tf2.cpp)
target_include_directories(getposecmd_tf2 
   PUBLIC include)
ament_target_dependencies(getposecmd_tf2 
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

#add_executable(getposecmd_moveit2 
#    src/getposecmd_moveit2.cpp)
#target_include_directories(getposecmd_moveit2 
#   PUBLIC include)
#ament_target_dependencies(getposecmd_moveit2
#    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

add_executable(getposecmd
    src/getposecmd_moveit2.cpp)
target_include_directories(getposecmd
   PUBLIC include)
ament_target_dependencies(getposecmd
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
        
add_executable(keyboardcontrol
    src/keyboardcontrol.cpp)
target_include_directories(keyboardcontrol
    PUBLIC include)
ament_target_dependencies(keyboardcontrol 
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

# Define executables
add_executable(movepointcmd 
    src/movepointcmd.cpp)
target_include_directories(movepointcmd 
    PUBLIC include)
ament_target_dependencies(movepointcmd 
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
  
add_executable(movecirclecmd 
    src/movecirclecmd.cpp)
target_include_directories(movecirclecmd 
    PUBLIC include)
ament_target_dependencies(movecirclecmd 
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
    
add_executable(webappcontrol
    src/webappcontrol.cpp)
target_include_directories(webappcontrol
    PUBLIC include)
ament_target_dependencies(webappcontrol 
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

add_executable(setgrippercmd
    src/setgrippercmd.cpp)
target_include_directories(setgrippercmd
    PUBLIC include)
ament_target_dependencies(setgrippercmd 
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
             
# Install targets
install(TARGETS getposecmd getposecmd_tf2 keyboardcontrol movepointcmd movecirclecmd webappcontrol setgrippercmd
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
  
ament_package()


