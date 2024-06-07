# ros2web_app

## Overview

`ros2web_app` is a web application framework that enables the development 
of web applications through YAML configuration and the creation of ROS2 packages.


[![Watch the video](https://img.youtube.com/vi/3-dwc0EN9TI/hqdefault.jpg)](https://www.youtube.com/embed/3-dwc0EN9TI)


## Installation

First install the [`ros2web`](https://github.com/ros2web/ros2web).

```bash
python3 -m pip install -r requirements.txt

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/ros2web/ros2web_app.git
cd ~/ros2_ws
colcon build
. ./install/local_setup.bash
```

### Troubleshooting

If you get the following error with the colcon build, please update numpy.

`TypeError: 'numpy._DTypeMeta' object is not subscriptable`

```shell
pip install numpy --upgrade
```


## Usage

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create package
# ros2 web create <Package Name>
ros2 web create hello 
cd ~/ros2_ws
colcon build --symlink-install
. ./install/local_setup.bash

ros2 run hello hello

# Star ros2web (another terminal)
ros2 web server --no-auth
```

Access the following URL.

http://localhost:8080/hello

## Examples
- [ros2web_widgets](https://github.com/ros2web/ros2web_app/tree/main/examples)
- [ros2web_turtlesim](https://github.com/ros2web/ros2web_turtlesim)
- [ros2web_gz_examples](https://github.com/ros2web/ros2web_gz_examples)
