// BEA
// Clint Sun, 2021
//

#ifndef SCANNER_DRIVER_H
#define SCANNER_DRIVER_H

#include <string>
#include <bea_scanner/packet_structure.h>

namespace bea {

class TcpCommandInterface;
class ScanDataReceiver;


class ScannerDriver
{
public:
  //! Initialize driver
  ScannerDriver();

  //! Cleanly disconnect in case of destruction
  ~ScannerDriver();

  //! Connect laser scanner
  bool connect(const std::string scanner_ip, int port);

  //! Disconnect from the laser scanner and reset internal state
  void disconnect();

  //! Return connection status
  bool isConnected() {return is_connected_; }

  //! Actively check connection to laserscanner
  bool checkConnection();

  //! Start capturing laser data
  bool startCapturingUDP(int port);

  //! Stop capturing laserdata
  bool stopCapturing();

  //! Return capture status
  bool isCapturing();

  //! Pop a single scan out of the driver's interal FIFO queue
  //! CAUTION: Returns also unfinished scans for which a full rotation is not received yet
  //! Call getFullScansAvailable() first to see how many full scans are available
  //! @returns A ScanData struct with distance / amplitude data as well as the packet headers belonging to the data
  ScanData getScan();

  //! Pop a single full scan out of the driver's internal FIFO queue if there is any
  //! If no full scan is available yet, blocks until a full scan is available
  //! @returns A ScanData struct with distance / amplitude data as well as the packet headers belonging to the data
  ScanData getFullScan();

  //! Get the total number of laserscans available (even scans which are not fully reveived yet)
  std::size_t getScansAvailable() const;

  //! Get the total number of fully received laserscans available
  std::size_t getFullScansAvailable() const;

  //! get scan resolution from UDP receiver
  std::size_t getResolution();

  //! get scan frequency from UDP receiver
  std::size_t getFrequency();

private:

  //! TCP command interface of the scanner
  TcpCommandInterface* command_interface_;

  //! Asynchronous data receiver
  ScanDataReceiver* data_receiver_;

  //! Internal connection state
  bool is_connected_;

  //! Internal capturing state
  bool is_capturing_;

};

}

#endif
