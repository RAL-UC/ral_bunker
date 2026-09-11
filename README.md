# RAL AgileX Bunker

Everything related to AgileX Bunker developed in RAL :D

## Requeriments

``` bash
sudo apt install ros-humble-robot-localization
```

``` bash
pip install utm

```

If you are using numpy 2.0

``` bash
pip install --upgrade transforms3d 
```

 
Velodyne LiDAR

``` bash
sudo apt install ros-$ROS_DISTRO-velodyne
```

Follow the instructions in ral_bunker_vectornav to use the Vectornav IMU.

For `bunker_controller.launch.py` to bring up `can0` automatically, enable passwordless sudo:
```bash
sudo visudo
# add:
<user> ALL=(ALL) NOPASSWD: /sbin/ip link set can0 up type can bitrate 500000
```

where user is your machine's user name.

# Mapping 

``` bash
ros2 launch ral_bunker_navigation cartographer.launch.py
```

``` bash
ros2 run nav2_map_server map_saver_cli -f <save_path>
```