import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    package_name = 'robot_simulation'

   # Path to the package where the included launch file is located
    robot_description_path = 'robot_description'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(robot_description_path),'launch','robot_xacro.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Path to the world file
    world_file_path = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'firstWorld.sdf'
    )

    # Ensure the world file exists
    if not os.path.exists(world_file_path):
        raise FileNotFoundError(f"World file not found: {world_file_path}")

    # Launch Gazebo with the specified world file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': f'-r -v 4 {world_file_path}'}.items()
    )

    # spawn robot in gazebo
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'fejemis', '-allow_renaming', 'true'],
    )
    
    # Path to the configuration file
    config_file_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'gz_bridge.yaml'
    )

    # Node to run the parameter bridge
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        parameters=[{'config_file': config_file_path}],
        output='screen'
    )

    
    
    return LaunchDescription([
        rsp,
        gazebo,
        gz_spawn_entity,
        gz_bridge,
    ])
