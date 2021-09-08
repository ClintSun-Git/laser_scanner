// BEA
// Clint Sun, 2021
//


#include <bea_scanner/tcp_command_interface.h>
#include <iostream>

namespace bea {

//-----------------------------------------------------------------------------
TcpCommandInterface::TcpCommandInterface(const std::string& scanner_ip, int tcp_port)
{
  using namespace boost::asio;

  scanner_ip_ = scanner_ip;
  tcp_port_ = tcp_port;

  std::cout << "Connecting to TCP command channel at " << scanner_ip_ << ": " << tcp_port_ << " ... " << std::endl;
  try
  {
    io_service io_service_;
    socket_ = new ip::tcp::socket(io_service_);
    ip::address address = ip::address::from_string(scanner_ip_);
    ip::tcp::endpoint ep(address, tcp_port_);

    boost::system::error_code ec;
    socket_->connect(ep, ec);

    if(ec)
      throw boost::system::system_error(ec);

  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << std::endl;
  }

}

//-----------------------------------------------------------------------------
TcpCommandInterface::~TcpCommandInterface()
{
  disconnect();
  delete socket_;
}

//-----------------------------------------------------------------------------
void TcpCommandInterface::disconnect()
{
}

//-----------------------------------------------------------------------------
bool TcpCommandInterface::sendCommand(std::uint8_t buf[], int size)
{
  using namespace boost::asio;

  std::uint8_t recv_buf[64];
  std::vector<char> cmd;

  if(size > 64)
    return false;

  cmd.resize(size);
  memcpy(&cmd[0], buf, size);

  try
  {
    boost::system::error_code ec;

    socket_->write_some(buffer(cmd), ec);

    int len = socket_->read_some(buffer(recv_buf), ec);

    std::cout << "get command response: " << std::endl;
    for(int i=0; i<len; i++)
    {
      std::cout << std::hex << (unsigned int)recv_buf[i] << " ";
    }
    std::cout << std::dec << std::endl;

    if(ec)
      throw boost::system::system_error(ec);

  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << std::endl;
    return false;
  }

  return true;

}

//-----------------------------------------------------------------------------
bool TcpCommandInterface::getVersionInfo()
{
  int CMD_LEN = 19;
  std::uint8_t cmdbuf[CMD_LEN] = {0x02,0x02,0xBE,0xA0,0x12,0x34,0x00,0x0A,0x63,0x52,0x4E,0x20,0x47,0x65,0x74,0x56,0x65,0x72,0x48};

  std::cout << "get version info ... " << std::endl;

  if( !sendCommand(cmdbuf, CMD_LEN) )
    return false;

  return true;

}

//-----------------------------------------------------------------------------
bool TcpCommandInterface::startScanOutput()
{
  int CMD_LEN = 20;
  std::uint8_t cmdbuf[CMD_LEN] = {0x02,0x02,0xBE,0xA0,0x12,0x34,0x00,0x0B,0x63,0x57,0x4E,0x20,0x53,0x65,0x6E,0x64,0x4D,0x44,0x49,0x26};

  std::cout << "start laser scan ... " << std::endl;

  if( !sendCommand(cmdbuf, CMD_LEN) )
    return false;

  return true;

}

//-----------------------------------------------------------------------------
bool TcpCommandInterface::stopScanOutput()
{
  return true;
}


//-----------------------------------------------------------------------------
}
