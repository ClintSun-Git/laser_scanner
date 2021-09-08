// BEA
// Clint Sun, 2021
//


#include <bea_scanner/scanner_driver.h>
#include <bea_scanner/packet_structure.h>
#include <bea_scanner/tcp_command_interface.h>
#include <bea_scanner/scan_data_receiver.h>

namespace bea {

//-----------------------------------------------------------------------------
ScannerDriver::ScannerDriver()
{
  command_interface_ = 0;
  data_receiver_ = 0;
  is_connected_ = false;
  is_capturing_ = false;
}

//-----------------------------------------------------------------------------
ScannerDriver::~ScannerDriver()
{
  disconnect();
}

//-----------------------------------------------------------------------------
bool ScannerDriver::connect(const std::string scanner_ip, int port)
{
  command_interface_ = new TcpCommandInterface(scanner_ip, port);

  if(!command_interface_->getVersionInfo())
    return false;

  is_connected_ = true;

  return true;

}


//-----------------------------------------------------------------------------
void ScannerDriver::disconnect()
{
  if( isCapturing() )
    stopCapturing();

  delete data_receiver_;
  delete command_interface_;
  data_receiver_ = 0;
  command_interface_ = 0;

  is_capturing_ = false;
  is_connected_ = false;

}

//-----------------------------------------------------------------------------
bool ScannerDriver::checkConnection()
{
  if( !command_interface_ || !isConnected() )
  {
    std::cerr << "ERROR: No connection to laser scanner or connection lost!" << std::endl;
    return false;
  }
  return true;
}

//-----------------------------------------------------------------------------
bool ScannerDriver::startCapturingUDP(int port)
{
  if( !checkConnection() )
    return false;

  data_receiver_ = new ScanDataReceiver(port);
  if( !data_receiver_->isConnected() )
    return false;

  if(!command_interface_->startScanOutput())
    return false;

  is_capturing_ = true;

  return true;
}


//-----------------------------------------------------------------------------
bool ScannerDriver::stopCapturing()
{
  if( !is_capturing_ || !command_interface_ )
    return false;

  if( !command_interface_->stopScanOutput() )
    return false;

  delete data_receiver_;
  data_receiver_ = 0;

  is_capturing_ = false;

  return true;
}

//-----------------------------------------------------------------------------
bool ScannerDriver::isCapturing()
{
  return is_capturing_ && data_receiver_->isConnected();
}

//-----------------------------------------------------------------------------
ScanData ScannerDriver::getScan()
{
  if( data_receiver_ )
    return data_receiver_->getScan();
  else
  {
    std::cerr << "ERROR: No scan capturing started!" << std::endl;
    return ScanData();
  }
}

//-----------------------------------------------------------------------------
ScanData ScannerDriver::getFullScan()
{
  if( data_receiver_ )
    return data_receiver_->getFullScan();
  else
  {
    std::cerr << "ERROR: No scan capturing started!" << std::endl;
    return ScanData();
  }
}

//-----------------------------------------------------------------------------
std::size_t ScannerDriver::getScansAvailable() const
{
  if( data_receiver_ )
    return data_receiver_->getScansAvailable();
  else
  {
    std::cerr << "ERROR: No scan capturing started!" << std::endl;
    return 0;
  }
}

//-----------------------------------------------------------------------------
std::size_t ScannerDriver::getFullScansAvailable() const
{
  if( data_receiver_ )
    return data_receiver_->getFullScansAvailable();
  else
  {
    std::cerr << "ERROR: No scan capturing started!" << std::endl;
    return 0;
  }
}


//-----------------------------------------------------------------------------
std::size_t ScannerDriver::getResolution()
{
  if( data_receiver_ )
    return data_receiver_->getResolution();
  else
  {
    std::cerr << "ERROR: No scan capturing started!" << std::endl;
    return 0;
  }
}

//-----------------------------------------------------------------------------
std::size_t ScannerDriver::getFrequency()
{
  if( data_receiver_ )
    return data_receiver_->getFrequency();
  else
  {
    std::cerr << "ERROR: No scan capturing started!" << std::endl;
    return 0;
  }
}



//-----------------------------------------------------------------------------

}
