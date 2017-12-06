
# How to set-up an SD for a zynq board

## Set-up icub user

(as root)

> groups icub 

> visudo

* add line: "icub ALL=(ALL:ALL) ALL"

> mkdir /usr/local/src/robot

> chown icub /usr/local/src/robot

## Set up repositories

(as icub)

> mkdir /usr/local/src/robot/icub-contrib-install

> vim ~/.bashrc

add lines:

* export ROBOT_CODE=/usr/local/src/robot
* export YARP_DIR=$ROBOT_CODE/yarp/build
* export ICUBcontrib_DIR=$ROBOT_CODE/icub-contrib-install
* export YARP_DATA_DIRS=$YARP_DIR/share/yarp:$ICUBcontrib_DIR/share/ICUBcontrib
* export PATH=$YARP_DIR/bin:$ICUBcontrib_DIR/bin:$PATH
* export ED_ROOT=$ROBOT_CODE/event-driven
  
git clone:

yarp (https://github.com/robotology/yarp)

* make only (not install)

icub-contrib-common (https://github.com/robotology/icub-contrib-common)

* set install directory as $ICUBcontrib_DIR
* make install

event-driven (https://github.com/robotology/event-driven)

* (cmake should have found install directory as $ICUBcontrib_DIR automatically)
* make install
    
## Set up device drivers

(as icub)

> sudo usermod -a -G i2c icub

> sudo vim /lib/udev/rules.d/77-iit-hpu.rules

add lines:

* SUBSYSTEM=="iit-hpu-class", GROUP="i2c"

> sudo vim /etc/rc.local

add lines:

* insmod $PATH_TO_HPU_DRIVER.ko ps=4096
* $PATH_TO_ZYNQGRABBER --biaswrite > /usr/local/src/robot/zynqGrabber.log
  
## Misc

check the device driver meta data

> udevadm info -q all -a /dev/iit-hpu0

check the device driver parameters

> cat /sys/module/iit_hpucore_dma/parameters/ps

