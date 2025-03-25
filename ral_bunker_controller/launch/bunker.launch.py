import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    In order to make the execute process work, you must add this at the end of the visudo file:

    octa ALL=(ALL) NOPASSWD: /sbin/ip link set can0 up type can bitrate 500000

    using the command:

    sudo visudo
    """
    
    bunker_base_share_dir = get_package_share_directory('bunker_base')
    bunker_launch_path = os.path.join(bunker_base_share_dir, 'launch', 'bunker_base.launch.py')
    
    bringup_action = ExecuteProcess(cmd=['sudo', 'ip', 'link', 'set', 'can0', 'up', 'type', 'can', 'bitrate','500000'], output='screen')

    bunker_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(bunker_launch_path))
    
    return LaunchDescription([
        bringup_action,
        bunker_launch
    ])
