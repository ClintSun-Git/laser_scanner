// BEA
// Clint Sun, 2021
//

#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <bea_scanner/scanner_driver.h>
#include <math.h>

int main(int argc, char **argv)
{
  std::cout << "Hello world!" << std::endl;
  std::string scanner_ip("192.168.1.250");
  int port = 50020;
  bea::ScannerDriver driver;

  std::cout << "Connecting to scanner at " << scanner_ip << " ... " << std::endl;
  if (driver.connect(scanner_ip, port))
    std::cout << "connect OK" << std::endl;
  else
  {
    std::cerr << "Connection to scanner at " << scanner_ip << " failed!" << std::endl;

    return false;
  }

  std::cout << "Starting capturing ..." << std::endl;

  if (driver.startCapturingUDP(port))
    std::cout << "start UDP capturing OK" << std::endl;
  else
  {
    std::cout << "start UDP capturing FAILED!" << std::endl;
    return false;
  }

  for(;;)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    int scans_captured = 0;
    int scans_available = driver.getFullScansAvailable();
    for (int i = 0; i < scans_available; i++)
    {
      auto scandata = driver.getFullScan();
      scans_captured++;
    }

    std::cout << "Received " << scans_captured << " from scanner" << std::endl;

    /*
    float angle_increment = fabs((float)(std::int16_t)driver.getResolution()/1000);
    std::uint16_t frequency = (std::uint16_t)driver.getFrequency();
    std::cout << "angle_increment: " << angle_increment << ", frequency: " << frequency << " " << std::endl;
    */

  }

  std::cout << "Trying to stop capture ..." << std::endl;

  //std::cout << "Stopping capture ... " << driver.stopCapturing() << std::endl;

  std::cout << "Goodbye world!" << std::endl;

  return 0;

}
