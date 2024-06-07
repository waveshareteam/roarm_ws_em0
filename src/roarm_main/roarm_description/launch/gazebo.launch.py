import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    gazebo_launch_file = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(gazebo_launch_file),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_footprint_base',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint', '40']
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', 'roarm_description', '-entity', 'roarm_description']
        )
    ])

