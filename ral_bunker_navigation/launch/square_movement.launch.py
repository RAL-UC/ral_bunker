from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():


    ral_bunker_controller_pkg = get_package_share_directory('ral_bunker_controller')
    velodyne = get_package_share_directory('velodyne')

    bunker_controller = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(ral_bunker_controller_pkg, 'launch', 'bunker_controller.launch.py')))

    ekf_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(ral_bunker_controller_pkg, 'launch', 'odom_ekf.launch.py')))

    velodyne_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(velodyne, 'launch', 'velodyne-all-nodes-VLP16-launch.py')))
    # I had to modify the source code ip, located in velodyne_driver/config/VLP16-velodyne_driver_node-params.yaml

    path_planner = Node(
            package='ral_bunker_navigation',
            name='ral_bunker_navigation',
            executable='path_planner',
            output='screen',
            parameters=[{'path_type': 'rectangle'}]
        )

    pid_controller_square = Node(
            package='ral_bunker_navigation',
            name='pid_controller_square',
            executable='pid_controller_square',
            output='screen',
        )
    
    wgs84_to_utm = Node(
            package='ral_gps',
            name='wgs84_to_utm',
            executable='wgs84_to_utm',
            output='screen'
        )
    
    return LaunchDescription([
        bunker_controller,
        ekf_launch,
        path_planner,
        pid_controller_square,
        wgs84_to_utm,
        velodyne_launch
    ])
