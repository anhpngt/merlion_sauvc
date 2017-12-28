# merlion_sauvc
### Wake-on-LAN
Follow the instruction from [here](http://kodi.wiki/view/HOW-TO:Set_up_Wake-on-LAN_for_Ubuntu).
IP address 192.168.0.101 was reserved for NUC

From ground PC:
```
sudo apt-get install powerwake
powerwake 192.168.0.101
```

### SSH
From ground PC:
```
ssh ugv@192.168.0.101
```
or
```
ssh ugv@ugv-nuc
```

Add these lines on ground PC into *~/.bashrc* to work with NUC ROS master
```
export ROS_MASTER_URI=http://192.168.0.101:11311
export ROS_IP=192.168.0.xxx
```
Remove (comment) those line if you want to work with a local ROS master

## Manual Motor Control
```
roslaunch bluerov bluerov_r1.launch
```
In ground PC, plug in the joystick and do
```
roslaunch bluerov_apps teleop_f310.launch
```
Also note in the launch file above, change the `dev` parameter to match the usb port of the joystick
```
<param name="dev" value="/dev/input/js0" type="string"/>
```

## Astra Camera

### Astra ROS Driver

- https://github.com/orbbec/ros_astra_camera

- https://github.com/orbbec/ros_astra_launch

Bringup depth camera by
```
roslaunch astra_launch astrapro.launch 
```

### Astra RGB Camera Driver
Install `usb_cam` driver
```
sudo apt-get install ros-*distro*-usb-cam
```
Bring up usb_cam node
```
rosrun usb_cam usb_cam_node _video_device:=/dev/video0 _pixel_format:=yuyv
roslaunch merlion_bringup astra_rgb.launch
TODO: udev rules
```

In case for permission denied on `/dev/video0`:
```
sudo chmod 666 /dev/video0
```
TODO: add udev rule