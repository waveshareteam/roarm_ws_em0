# ros2web_example_cpp

## Installation

```bash
sudo apt install nlohmann-json3-dev

cd ~/ros2_ws/src/ros2web/examples/ros2web_example_cpp
rm COLCON_IGNORE

cd ~/ros2_ws
colcon build --packages-select ros2web_example_cpp
. ./install/local_setup.bash
```

## Usage

```bash
ros2 web server --no-auth
ros2 run ros2web_example_cpp api
```

http://localhost:8000/add_two_ints?a=1&b=2