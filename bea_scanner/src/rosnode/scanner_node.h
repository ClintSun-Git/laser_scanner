// BEA
// Clint Sun, 2021
//

#ifndef SCANNER_NODE_H
#define SCANNER_NODE_H

#include <ros/ros.h>
#include <std_msgs/String.h>

namespace bea {

class ScannerDriver;

class ScannerNode
{
public:
  //! Initialize and connect to laser scanner
  ScannerNode();

  //! Callback function for control commands
  void cmdMsgCallback( const std_msgs::StringConstPtr& msg );

private:
  //! Connect to the laser scanner
  bool connect();

  //! Time callback function for getting data from the driver and sending them out
  void getScanData( const ros::TimerEvent& e);

  //! Internal ROS node handle
  ros::NodeHandle nh_;

  //! Callback timer for getScanData(...)
  ros::Timer get_scan_data_timer_;

  //! ROS publisher for publishing scan data
  ros::Publisher scan_publisher_;

  //! ROS subscriber for receiving control commands
  ros::Subscriber cmd_subscriber_;

  //! frame_id of sensor_msgs/Laserscan messages
  std::string frame_id_;

  std::string scanner_ip_;

  //! Pointer to driver
  ScannerDriver* driver_;

};

}

#endif
