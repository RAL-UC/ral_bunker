from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():


    ral_bunker_controller_pkg = get_package_share_directory('ral_bunker_controller')
    nmea_navsat_pkg = get_package_share_directory('nmea_navsat_driver')

    bunker_controller = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(ral_bunker_controller_pkg, 'launch', 'bunker.launch.py')))
    ekf_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(ral_bunker_controller_pkg, 'launch', 'dual_ekf_navsat.launch.py')))

    gps_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(nmea_navsat_pkg, 'launch', 'nmea_serial_driver.launch.py'))) 

    imu = Node(
            package='ral_bunker_vectornav',
            name='imu',
            executable='vectornav_imu',
            output='screen'
        )

    path_planner = Node(
            package='ral_bunker_navigation',
            name='ral_bunker_navigation',
            executable='path_planner',
            output='screen',
            parameters=[{'path_type': 'square'}]
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
        imu,
        gps_launch,
        bunker_controller,
        ekf_launch,
        path_planner,
        pid_controller_square,
        wgs84_to_utm
    ])
