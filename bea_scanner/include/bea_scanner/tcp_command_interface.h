// BEA
// Clint Sun, 2021
//

#ifndef TCP_COMMAND_INTERFACE_H
#define TCP_COMMAND_INTERFACE_H

#include <string>
#include <boost/asio.hpp>

namespace bea {

//! TCP connection for command interface
class TcpCommandInterface
{
public:
  //! configure ip and port
  TcpCommandInterface(const std::string& scanner_ip, int tcp_port);

  ~TcpCommandInterface();

  void disconnect();

  //! Version information of laser scanner
  bool getVersionInfo();

  //! Initiate output of scan data
  bool startScanOutput();

  //! Terminate output of scan data
  bool stopScanOutput();


private:

  //! public interface for any command
  bool sendCommand(std::uint8_t buf[], int size);

  //! Scanner IP
  std::string scanner_ip_;

  //! TCP port
  int tcp_port_;

  //! socket for TCP command interface
  boost::asio::ip::tcp::socket* socket_;

};
}

#endif
