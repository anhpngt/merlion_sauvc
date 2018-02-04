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