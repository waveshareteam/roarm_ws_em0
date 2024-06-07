cd ~/roarm_ws_em0
colcon build
colcon build --packages-select roarm_web_app launch_api ros2web_app ros2web_widgets ros2web ros2web_example_py --symlink-install 
echo "source ~/roarm_ws_em0/install/setup.bash" >> ~/.bashrc
source ~/.bashrc 
