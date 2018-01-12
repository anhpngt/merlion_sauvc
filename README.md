# merlion_sauvc
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

## Astra Camera

### Astra ROS Driver

- https://github.com/orbbec/ros_astra_camera

- https://github.com/orbbec/ros_astra_launch

Dependencies
```
sudo apt-get update && sudo apt-get install ros-*distro*-astra-camera ros-*distro*-astra-launch
```

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
roslaunch merlion_bringup astra_rgb.launch
```

In case for permission denied on `/dev/video0` (or `/dev/video1`)
```
sudo chmod 666 /dev/video0
```

## Logitech USB Webcam
Bringup
```
roslaunch merlion_bringup logitech_cam.launch
```
