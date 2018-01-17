# merlion_sauvc

## Introduction
Repository for Team Merlion to participate in [SAUVC 2018](https://sauvc.org/). This is a ROS-based project.

## Requirements
- ROS Indigo (Ubuntu 14.04 LTS) or ROS Kinetic (Ubuntu 16.04 LTS)
- Further dependencies are listed accordingly for each sensors. Replace `*distro*` with `indigo` or `kinetic` accordingly to your specifications.

## Setup
### Wake-on-LAN
Follow the instruction from [here](http://kodi.wiki/view/HOW-TO:Set_up_Wake-on-LAN_for_Ubuntu).

IP address 192.168.0.120 was reserved for NUC

From ground PC:
```
sudo apt-get install powerwake
powerwake 192.168.0.120
```

### udev
Udev rules files are stored in `merlion_setup/udev_rules`.

After adding the necessary rules into `/etc/udev/rules.d/`, do
```
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
```

### SSH
From ground PC:
```
ssh ugv@192.168.0.120
```
or
```
ssh ugv@ugv-nuc
```

Add these lines on ground PC into *~/.bashrc* to work with NUC ROS master
```
export ROS_MASTER_URI=http://192.168.0.120:11311
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

## Kinect Sensors
Install packages
```
sudo apt-get install ros-*distro*-openni-camera ros-*distro*-rgbd-launch ros-*distro*-freenect-stack
```

Bringup
```
roslaunch freenect_launch freenect.launch
```

## Logitech USB Webcam
Install necessary package
```
sudo apt-get install ros-*distro*-usb-cam
```

Bringup
```
roslaunch merlion_bringup logitech_cam.launch
```
