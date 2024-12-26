import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = LaunchDescription()

    # Declare prefix for namespace and tf tree elements
    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='/mybot1/',
        description='Prefix for the arm links and joints'
    )
    # When not namespacing the arm, changing the base link name gives a way to prepare the arm for merging onto a larger robot description 
    arm_base_link_name_arg = DeclareLaunchArgument(
        'arm_base_link_name',
        default_value='base_link',
        description="Name of the arm's base link"
    )

    # For arms where the end-effector has been rotated from the factory configuration
    end_rot_arg = DeclareLaunchArgument(
        'end_rot',
        default_value='0',
        description="CCW rotation of the arm's end-effector (looking at it), can be 0, 90, 180, 270"
    )

    # Get the launch configuration variables
    prefix = LaunchConfiguration('prefix')
    base_link_name = LaunchConfiguration('arm_base_link_name')
    end_rot = LaunchConfiguration('end_rot')

    # Define the path to the builder Xacro file
    xacro_file = os.path.join(
        get_package_share_directory('roarm_description'),
        'urdf',
        #'roarm_urdf_builder.xacro' # use this if building your own cross-package robot description
        'mybot1.xacro' # use this for a minimal example of mounting the arm on a cylinder
    )

    # Process the Xacro file to generate the robot description parameter
    robot_description_content = Command([
        'xacro ', xacro_file,
        ' prefix:=', prefix,
        ' base_link_name:=', base_link_name,
        ' end_rot:=', end_rot,
    ])

    # Wrap robot_description_content with ParameterValue
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Construct the child_frame_id by concatenating prefix and base_link_name using PythonExpression
    child_frame_id = PythonExpression(["'", prefix, "' + '", base_link_name, "'"])


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=[prefix],
        output='screen',
        parameters=[robot_description],
        #arguments=['--ros-args', '--log-level', 'debug'], #if debugging needed
        )

    # some kind of joint state publisher is required for the arm joints or the 
    # tf tree won't be complete
    # here using the gui tool
    # otherwise joint states would be provided by the package used to control the arm
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        namespace=[prefix],
        parameters=[robot_description],
        )
    
    # RViz
    rviz_config_file = (
        get_package_share_directory("roarm_description") + "/config/mounted_roarm_description.rviz"
    )
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=["-d", rviz_config_file],
        )

    ld.add_action(prefix_arg)
    ld.add_action(arm_base_link_name_arg)
    ld.add_action(end_rot_arg)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz2_node)

    return ld

