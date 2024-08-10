#This configuration is a bit cumbersome
#If you need it, ask questions on github, and I will reply, so that everyone can use and communicate

search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=roarm_description.srdf
robot_name_in_srdf=roarm_description
moveit_config_pkg=roarm_moveit
robot_name=roarm_description
planning_group_name=hand
ikfast_plugin_pkg=roarm_moveit_ikfast_plugins
base_link_name=base_link
eef_link_name=hand_tcp
ikfast_output_path=/home/ws/roarm_ws_em0/src/roarm_main/roarm_moveit_ikfast_plugins/src/roarm_description_hand_ikfast_solver.cpp

ros2 run moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
