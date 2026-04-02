# RAL AgileX Bunker

Everything related to AgileX Bunker developed in RAL :D

## Requeriments

``` bash
sudo apt install ros-humble-robot-localization
```

``` bash
pip install utm
```

# Mapping 

``` bash
ros2 launch ral_bunker_navigation cartographer.launch.py
```

``` bash
ros2 run nav2_map_server map_saver_cli -f <save_path>
```