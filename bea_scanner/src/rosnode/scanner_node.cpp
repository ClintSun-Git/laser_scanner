// BEA
// Clint Sun, 2021
//

/*
sensor_msgs::LaserScan

Header header             # timestamp in the header is the acquisition time of
                          # the first ray in the scan.
                          #
                          # in frame frame_id, angles are measured around
                          # the positive Z axis (counterclockwise, if Z is up)
                          # with zero angle being forward along the x axis

float32 angle_min         # start angle of the scan [rad]
float32 angle_max         # end angle of the scan [rad]
float32 angle_increment   # angular distance between measurements [rad]

float32 time_increment    # time between measurements [seconds] - if your scanner
                          # is moving, this will be used in interpolating position
                          # of 3d points
float32 scan_time         # time between scans [seconds]

float32 range_min         # minimum range value [m]
float32 range_max         # maximum range value [m]

float32[] ranges          # range data [m] (Note: values < range_min or > range_max should be discarded)
float32[] intensities     # intensity data [device-specific units].  If your
                          # device does not provide intensities, please leave
                          # the array empty.
*/

#include "scanner_node.h"
#include <sensor_msgs/LaserScan.h>
#include <bea_scanner/scanner_driver.h>
#include <math.h>

namespace bea {

//-----------------------------------------------------------------------------
ScannerNode::ScannerNode():nh_("~")
{
  driver_ = 0;

  nh_.param("frame_id", frame_id_, std::string("/scan"));
  nh_.param("scanner_ip", scanner_ip_, std::string("192.168.1.250"));

  if( !connect() )
    return;

  // Declare publisher and create timer
  scan_publisher_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 100);
  //cmd_subscriber_ = nh_.subscribe("control_command", 100, &ScannerNode::cmdMsgCallback, this);

  // balance efficiency and CPU workload
  get_scan_data_timer_ = nh_.createTimer(ros::Duration(0.1), &ScannerNode::getScanData, this);

}

//-----------------------------------------------------------------------------
bool ScannerNode::connect()
{
  delete driver_;

  int port = 50020;

  driver_ = new ScannerDriver();
  std::cout << "Connecting to scanner at " << scanner_ip_ << " ... " << std::endl;
  if( driver_->connect(scanner_ip_, port) )
    std::cout << "connect OK" << std::endl;

  else
  {
    std::cerr << "Connection to scanner at " << scanner_ip_ << " failed!" << std::endl;
    return false;
  }

  std::cout << "Starting capturing ... " << std::endl;
  if( driver_->startCapturingUDP(port) )
    std::cout << "start UDP capturing OK" << std::endl;
  else
  {
    std::cout << "start UDP capturing FAILED!" << std::endl;
    return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
void ScannerNode::getScanData(const ros::TimerEvent &e)
{
  ScanData scandata;

  // 0.1s timer, 80Hz scan rate, once 8 complete scan data, get all of scan data to ensure real time
  int scans_available = driver_->getFullScansAvailable();
  for (int i = 0; i < scans_available; i++)
  {
    scandata = driver_->getFullScan();
  }

  if( scandata.distance_data.empty() )
    return;

  float angle_increment = fabs((float)(std::int16_t)driver_->getResolution()/1000);
  std::uint16_t frequency = (std::uint16_t)driver_->getFrequency();
  //std::cout << "angle_increment: " << angle_increment << ", frequency: " << frequency << " " << std::endl;

  sensor_msgs::LaserScan scanmsg;
  scanmsg.header.frame_id = frame_id_;
  scanmsg.header.stamp = ros::Time::now();

  scanmsg.angle_min = -47.6/180 * M_PI;
  scanmsg.angle_max = 227.6/180 * M_PI;
  scanmsg.angle_increment = angle_increment/180 * M_PI;

  scanmsg.time_increment = 1/frequency / scandata.distance_data.size();
  scanmsg.scan_time = 1/frequency;

  scanmsg.range_min = 0.0;
  scanmsg.range_max = 100.0;   // max value of distance

  scanmsg.ranges.resize(scandata.distance_data.size());
  scanmsg.intensities.resize(scandata.distance_data.size());

  for( std::size_t i=0; i<scandata.distance_data.size(); i++ )
  {
    scanmsg.ranges[i] = float(scandata.distance_data[i])/1000.0f;
    scanmsg.intensities[i] = 100;

  }

  scan_publisher_.publish(scanmsg);

  //ROS_INFO("publishing laser message ... ");

}

//-----------------------------------------------------------------------------
void ScannerNode::cmdMsgCallback(const std_msgs::StringConstPtr &msg)
{
// any configuration of parameter
}

//-----------------------------------------------------------------------------
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "scanner_node", ros::init_options::AnonymousName);

  new bea::ScannerNode();
  ROS_INFO("Here is a laser scanner ROS node ");

  ros::spin();

  return 0;

}
