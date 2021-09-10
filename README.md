LiDAR Scanner Driver
===============================

The driver is developed by C++ and based upon boost ASIO library for TCP/UDP communication.

The driver contains a ROS-Node interface to the Robot Operating System, and the driver example, was tested on Ubuntu 16 and ROS Kinetic environment.




Usage with ROS
---------------------------


### Published Topics

- `scan` (sensor_msgs/Laserscan) A standard ROS Laserscan message containing the measured data. 

### Parameters

- `frame_id` The frame-ID in the Header of the published `sensor_msgs/Laserscan` messages
- `scanner_ip` IP of the laser scanner

### How to use

Firstly install a ROS environment on Ubuntu, copy the driver `bea_scanner` to `~/catkin_ws/src` , run the following command to compile it in ROS workspace:

    cd ~/catkin_ws/
    catkin_make
    source devel/setup.bash
    roslaunch bea_scanner scanner.launch

This starts `RViz` and the driver and you should see the measuring output of the scanner.



Basic Usage without ROS
---------------------------


Run driver example and test the driver library, firstly compile the driver:

    $ cd bea_scanner
    $ cd build
    $ cmake ..
    $ make

This builds a SHARED library which can be used in your program. 


