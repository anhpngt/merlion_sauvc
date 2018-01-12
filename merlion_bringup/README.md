## Astra Camera

Orbbec Astra Camera will no longer be used. This is archived for future reference.

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