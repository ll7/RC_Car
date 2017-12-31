# RC Car controlled by an Arduino Uno, RaspberryPi 3 and a Ubuntu 16.04 Laptop with ROS Kinetic installed on RPI and Laptop

## Hardware
- Arduino Uno
- RaspberryPi 3
- Ubuntu 16.04 LTS Laptop

## RaspberryPi 3 (RPI) Setup
Install ubuntu mate for RPI. ROS does not support Raspbian Stretch. Therefore, it is easier to use ubuntu mate for the RPI.
https://www.intorobotics.com/how-to-install-ros-kinetic-on-raspberry-pi-3-ubuntu-mate/
Follow the ROS kinetic installation, too.

### rosserial-arduino
Install ros-kinetic-rosserial-arduino and the Arduino IDE
http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

### raspicam_node
Install raspicam_node from source. A few additional changes need to be made, which are not explained in the binary installation. Follow the build instructions carefully.
https://github.com/UbiquityRobotics/raspicam_node
Remember to enable the camera in your raspi-config.
```
sudo raspi-config
-> advanced-options -> enable camera -> reboot
```
It is also beneficial to install from source, because you can modify your launch files. Chose your camera model version, reduce resolution and framerate.

## ROS

### ros multiple machines
http://wiki.ros.org/ROS/Tutorials/MultipleMachines
http://wiki.ros.org/ROS/NetworkSetup
Don't forget to export the ROS_MASTER_URI in each shell. Check the connection to each device with the terminal command `ping ssh-name`.

