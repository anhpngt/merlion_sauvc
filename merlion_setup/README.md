## udev rules

Copy these `.rules` files into `/etc/udev/rules.d`
```
cd ~/catkin_ws/src/merlion_sauvc/merlion_setup/udev_rules
sudo cp ./* /etc/udev/rules.d/
```

Restart `udev` service
```
sudo service udev reload
sudo service udev restart
```