carl_phidgets [![Build Status](https://api.travis-ci.org/WPI-RAIL/carl_bot.png)](https://travis-ci.org/WPI-RAIL/carl_bot)
========

#### CARL (Crowdsourcing for Autonomous Robot Learning)
For full documentation, see [the ROS wiki](http://ros.org/wiki/carl_bot).

### carl_phidgets
This package is based on CCNY's [phidgets_drivers](http://wiki.ros.org/phidgets_drivers) package.

### First Time Setup
To use the Phidgets IMU as a user other than root, you need to create a udev rule.  Create a file called `99-phidgets.rules`, with the following line:
 * `SUBSYSTEMS=="usb", ACTION=="add", ATTRS{idVendor}=="06c2", ATTRS{idProduct}=="00[3-a][0-f]", MODE="666"`

Copy this file into the /etc/udev/rules.d directory.  You may need to reboot your computer for this to take effect.