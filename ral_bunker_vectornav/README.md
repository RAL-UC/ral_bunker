# Bunker Vectornav
Package for running Vectornav VN-200S sensor with ROS2

## Description

This package contains multiple codes for extracting data from VN-200S sensor.


### Hardware (optional)
list of dependencies: (delet this line at the end)
* Vectornav VN200-S

## Install Vectornav library

You have to download Vectornav programming library .zip in the following [link](https://www.vectornav.com/resources/programming-libraries/vectornav-programming-library). Then, you have to install the library using:

```
cd <path_to>/vnproglib/python
pip install .
```


## Demo 

To run the IMU use:
```
ros2 run ral_bunker_vectornav vectornav_imu
```

