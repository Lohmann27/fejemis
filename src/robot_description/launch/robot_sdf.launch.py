import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution



def generate_launch_description():

    package_name = 'robot_description'

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')


    # Process the sdf file
    robot_description_share = FindPackageShare(package_name).find(package_name)
    robot_description_path = PathJoinSubstitution([
        robot_description_share,
        'model',
        'robot_model.sdf'
    ])
    
    with open(robot_description_path.perform(None), 'r') as infp:
        robot_description = infp.read()

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )


    # Launch!
    return LaunchDescription([
        joint_state_publisher_node,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2 control if true'),

        node_robot_state_publisher
    ])
