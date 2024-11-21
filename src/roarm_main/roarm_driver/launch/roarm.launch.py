from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the launch argument for serial port
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port to be used by the node'
    )

    # Declare the node with the serial port parameter
    roarm_driver_node = Node(
        package='roarm_driver',
        executable='roarm_driver',
        name='roarm_driver',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': 115200
        }]
    )

    return LaunchDescription([
        serial_port_arg,
        roarm_driver_node
    ])
