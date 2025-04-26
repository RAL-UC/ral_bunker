# Bunker Navigation Documentation

## Path planning

Currently you can cerate local paths in the odom frame using the path_planner.cpp node. It has three path types that you can use as params:

- line: straitght line
- snake: sinusoidal.
- square: square.


For publishing a path, you use:

``` bash
ros2 run ral_bunker_navigation path_planner  --ros-args -p path_type:=<param>
```

