# merlion_sauvc

## Introduction
Repository for Team Merlion to participate in [SAUVC 2018](https://sauvc.org/). This is a ROS-based project.

## Goals

### TODO list
Hardware:
- [ ] Acoustics
- [x] Depth sensor
- [ ] EStop/Kill switch
- [x] Frames
- [ ] Servo

Software:
- [ ] Mission Manager
- [x] Qualification node
- [ ] Mission 1 node
- [ ] Mission 2 node
- [ ] Mission 3 node
- [x] Object detection, shape/color-based
- [ ] Object detection, deep learning
- [x] Vel_controller
- [x] SLAM


## Requirements
- ROS Kinetic (Ubuntu 16.04 LTS). 
  - ROS Indigo (Ubuntu 14.04 LTS) may work but is not supported.

- Install package dependencies:
```
sudo apt-get install ros-kinetic-usb-cam ros-kinetic-mavros ros-kinetic-joy
```

Additionally, install `bluerov` package from [source](https://github.com/bluerobotics/bluerov-ros-pkg#installation-from-source) (apt does not work).

## Setup
### Wake-on-LAN
Follow the instruction from [here](http://kodi.wiki/view/HOW-TO:Set_up_Wake-on-LAN_for_Ubuntu).

IP address 192.168.1.119 was reserved for NUC

From ground PC:
```
sudo apt-get install powerwake
powerwake 192.168.1.119
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
ssh usv@192.168.1.119
```
or
```
ssh usv@usv
```

Add these lines on ground PC into *~/.bashrc* to work with NUC ROS master
```
export ROS_MASTER_URI=http://192.168.1.119:11311
export ROS_IP=192.168.1.xxx # your `hostname -I` result
```
Remove (comment) those lines if you want to work with a local ROS master

### Manual Motor Control
TODO: Update this
```
roslaunch bluerov apm.launch
```

In ground PC, plug in the joystick and use QGroundControl or ros teleop
```
roslaunch bluerov_apps teleop_f310.launch
```
Also note in the launch file above, change the `dev` parameter to match the usb port of the joystick
```
<param name="dev" value="/dev/input/js0" type="string"/>
```

### Logitech USB Webcam
```
roslaunch merlion_bringup sensors.launch
roslaunch merlion_bringup image_view.launch
```