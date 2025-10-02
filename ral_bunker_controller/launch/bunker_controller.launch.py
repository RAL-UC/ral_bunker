import os
import xacro
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    """
    In order to make the execute process work, you must add this at the end of the visudo file:

    octa ALL=(ALL) NOPASSWD: /sbin/ip link set can0 up type can bitrate 500000

    using the command:

    sudo visudo
    """
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    
    bunker_base_share_dir = get_package_share_directory('bunker_base')
    bunker_launch_path = os.path.join(bunker_base_share_dir, 'launch', 'bunker_base.launch.py')
    ral_bunker_controller_path = get_package_share_directory('ral_bunker_controller')
    nmea_navsat_pkg = get_package_share_directory('nmea_navsat_driver')

    ssmm_gnc_rl_sim_path = os.path.join(get_package_share_directory('ssmm_gnc_rl_sim'))
    xacro_file = os.path.join(ssmm_gnc_rl_sim_path, 'urdf', 'bunker.urdf') # We use the same URDF from sim on purpose

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml(), 'use_sim_time': use_sim_time}

    # Bring up CAN0 for bunker base
    bringup_action = ExecuteProcess(cmd=['sudo', 'ip', 'link', 'set', 'can0', 'up', 'type', 'can', 'bitrate','500000'], output='screen')

    # Bunker base
    bunker_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(bunker_launch_path))

    # LiDAR
    vlp16_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(ral_bunker_controller_path, 'launch', 'vlp16.launch.py')))

    # VectorNav IMU 
    imu = Node(
            package='ral_bunker_vectornav',
            name='imu',
            executable='vectornav_imu',
            output='screen'
        )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    # gps_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(nmea_navsat_pkg, 'launch', 'nmea_tcpclient_driver.launch.py'))) 
    
    return LaunchDescription([
        bringup_action,
        bunker_launch,
        imu,
        vlp16_launch,
        node_robot_state_publisher,
        # gps_launch,
    ])
