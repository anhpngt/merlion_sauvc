## udev rules

Find info for udev
```
udevadm info /dev/video1
```

Copy these `.rules` files into `/etc/udev/rules.d`
```
cd ~/catkin_ws/src/merlion_sauvc/merlion_setup/udev_rules
sudo cp ./* /etc/udev/rules.d/
```

Restart `udev` service
```
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
```

## Astra Camera

Orbbec Astra Camera and Kinect Sensor will no longer be used. This is archived for future reference.

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
sudo apt-get install ros-kinetic-usb-cam
```
Bring up usb_cam node
```
roslaunch merlion_bringup astra_rgb.launch
```

In case for permission denied on `/dev/video0` (or `/dev/video1`)
```
sudo chmod 666 /dev/video0
```

## Kinect Sensors
Install packages
```
sudo apt-get install ros-kinetic-openni-camera ros-kinetic-rgbd-launch ros-kinetic-freenect-stack
```

Bringup
```
roslaunch freenect_launch freenect.launch
```