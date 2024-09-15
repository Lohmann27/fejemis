from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Set paths and file names
    package_name = 'robot_description'  # replace with your package name
    urdf_file_name = 'robot.urdf.xacro'  # replace with your URDF file name
    urdf_file_path = os.path.join(
        get_package_share_directory(package_name), 'xacro', urdf_file_name)

    # Declare a launch argument for using GUI or not
    use_gui_arg = DeclareLaunchArgument(
        'use_gui', default_value='true', description='Use joint_state_publisher_gui')

    # Get the value of the use_gui launch argument
    use_gui = LaunchConfiguration('use_gui')

    # Create the joint_state_publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],  # Use simulation time if true
        condition=LaunchConfiguration('use_gui').not_equals('true')  # Launch JSP if GUI is not used
    )

    # Create the joint_state_publisher_gui node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': True}],  # Use simulation time if true
        condition=LaunchConfiguration('use_gui').equals('true')  # Launch JSP GUI if GUI is used
    )

    # Create the robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': urdf_file_path}]
    )

    return LaunchDescription([
        use_gui_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
    ])
