 # How to run AgileX Bunker node for ROS2 (Humble) 


## Setup Steps 

Go to your wokspace and then clone the bunker repo in your src folder with the following command: 

```
git clone -b humble https://github.com/agilexrobotics/bunker_ros2.git 
```
Clone the ugv_sdl repo in your src folder with the following command: 

```
git clone https://github.com/agilexrobotics/ugv_sdk.git 
```

Then go back to your workspace folder and colcon build:

```
colcon build
```

**Note:**  Controller must be ON to move the bunker. 

 
## Example

Run in separate terminals: 

Terminal 1: 

```
cd [path_to_your_ws]/src/ugv_sdk/scripts/ 
bash bringup_can2usb_500k.bash 
candump can0 
```
Terminal 2: 

```
ros2 launch bunker_base bunker_base.launch.py 
```

You should now see the topics! PS: you could also create a .sh script to do these steps for you. 

 

## Nice to have (ROS2 with joy) 

You could also use teleop_twist_joy package to move the bunker with an external controller (PS4/5, XBOX) 

```
ros2 launch teleop_twist_joy teleop-launch.py config_filepath:=/home/octa/magister/config_files/joystick.config.yaml 
```

# How to run Dead Reckoning

```
ros2 launch bunker_base bunker_base.launch.py 
```