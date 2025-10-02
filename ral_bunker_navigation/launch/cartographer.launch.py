import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default=False)

    # Setup project paths
    ral_bunker_navigation_path = os.path.join(get_package_share_directory('ral_bunker_navigation'))

    # SLAM TOOLBOX
    slam_toolbox_config_path = os.path.join(ral_bunker_navigation_path, 'config', 'slam_toolbox.yaml')

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_toolbox_config_path
        }.items()
    )

    return LaunchDescription([
        slam_toolbox_launch
    ])