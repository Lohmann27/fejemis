import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    package_name = 'robot_simulation'
 

    slam_params = os.path.join(get_package_share_directory(package_name), 'config','mapper_params_online_async.yaml')
    
    slam = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')]),
                    launch_arguments={'slam_params_file:=' + slam_params, 'use_sim_time:=true'}.items(),
            )



    return LaunchDescription([
        slam,
    ])
