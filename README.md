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

Add these lines on ground PC into *~/.bashrc* to work with NUC ROS
```
export ROS_MASTER_URI=http://192.168.0.101:11311
export ROS_IP=192.168.0.xxx
```

### Bring up Astra camera
Install `usb_cam` driver first
```
sudo apt-get install ros-*distro*-usb-cam
```
Bring up usb_cam node
```
rosrun usb_cam usb_cam_node _video_device:=/dev/video0 _pixel_format:=yuyv
```
From ground PC, use rviz or rqt_image_view to visualize the image topic.

In case for permission denied on `/dev/video0`:
```
sudo chmod 666 /dev/video0
```
TODO: add udev rule