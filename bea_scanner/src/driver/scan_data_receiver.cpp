// BEA
// Clint Sun, 2021
//

#include <bea_scanner/scan_data_receiver.h>
#include <chrono>
#include <ctime>

namespace bea {

//-----------------------------------------------------------------------------
ScanDataReceiver::ScanDataReceiver(int udp_port):inbuf_(4096),instream_(&inbuf_),ring_buffer_(65536),scan_data_()
{
  udp_socket_ = 0;
  is_connected_ = false;

  try
  {
    udp_socket_ = new boost::asio::ip::udp::socket(io_service_, boost::asio::ip::udp::v4());
    udp_socket_->bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), udp_port));

    // Start async reading
    udp_socket_->async_receive_from(boost::asio::buffer(&udp_buffer_[0], udp_buffer_.size()), udp_endpoint_,
                                    boost::bind(&ScanDataReceiver::handleSocketRead, this,
                                                boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    io_service_thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
    is_connected_ = true;
  }

  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << std::endl;
  }

  std::cout << "Receiving scanner data at UDP port " << udp_port << " ... " << std::endl;

}

//-----------------------------------------------------------------------------
ScanDataReceiver::~ScanDataReceiver()
{
  disconnect();
  delete udp_socket_;
}

// disconnect udp
void ScanDataReceiver::disconnect()
{
  is_connected_ = false;
  try
  {
    if( udp_socket_ )
      udp_socket_->close();

    io_service_.stop();

    if( boost::this_thread::get_id() != io_service_thread_.get_id() )
      io_service_thread_.join();
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << std::endl;
  }
}

// get connection status
bool ScanDataReceiver::checkConnection()
{
  if( !isConnected() )
    return false;
  if( (std::time(0)-last_data_time_) > 2 )
  {
    disconnect();
    return false;
  }

  return true;
}


// get frame from queue scan_data_
ScanData ScanDataReceiver::getScan()
{
  std::unique_lock<std::mutex> lock(data_mutex_);
  ScanData data(std::move(scan_data_.front()));
  scan_data_.pop_front();
  return data;
}

// get complete scan data from queue scan_data_
ScanData ScanDataReceiver::getFullScan()
{
  std::unique_lock<std::mutex> lock(data_mutex_);
  while( checkConnection() && isConnected() && scan_data_.size()<( (pack_num>0)?pack_num:16 ) )
  {
    data_notifier_.wait_for(lock, std::chrono::seconds(1));
  }

  ScanData packet_front;
  ScanData packet_load;

  packet_front = ScanData(std::move(scan_data_.front()));
  scan_data_.pop_front();

  // find front package for a complete scan
  while(packet_front.header.sub_number != 1)
  {
    packet_front = ScanData(std::move(scan_data_.front()));
    scan_data_.pop_front();
  }

  for( std::uint8_t i=0; i<pack_num-1; i++ )
  {
    packet_load = ScanData(std::move(scan_data_.front()));
    scan_data_.pop_front();

    packet_front.distance_data.insert(packet_front.distance_data.end(), packet_load.distance_data.begin(), packet_load.distance_data.end());

  }

  return packet_front;

}


// get queue scan_data_ size
std::size_t ScanDataReceiver::getFullScansAvailable() const
{
  if( scan_data_.size() < pack_num )
    return 0;
  else
    return scan_data_.size()/( (pack_num>0)?pack_num:16 );
}

//
std::size_t ScanDataReceiver::getResolution()
{
  return resolution;
}

//
std::size_t ScanDataReceiver::getFrequency()
{
  return frequency;
}


// udp asynchronous read
void ScanDataReceiver::handleSocketRead(const boost::system::error_code &error, std::size_t bytes_transferred)
{
  if (!error )
  {
    // Read all received data and write it to the internal ring buffer
    writeBufferBack(&udp_buffer_[0],bytes_transferred);

    // Handle (read and parse) packets stored in the internal ring buffer
    while( handleNextPacket() ) {}

    // Read data asynchronously
    udp_socket_->async_receive_from(boost::asio::buffer(&udp_buffer_[0],udp_buffer_.size()), udp_endpoint_,
                                    boost::bind(&ScanDataReceiver::handleSocketRead, this,
                                                boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
  }
  else
  {
    if( error.value() != 995 )
      std::cerr << "ERROR: " << "data connection error: " << error.message() << "(" << error.value() << ")" << std::endl;
    disconnect();
  }
  last_data_time_ = std::time(0);
}

// get distance and amplitude data from circular buffer (ring_buffer_) to queue (scan_data_)
bool ScanDataReceiver::handleNextPacket()
{
  // Search for a packet
  int packet_start = findPacketStart();
  if( packet_start<0 )
    return false;

  // Try to retrieve packet
  char buf[65536];
  PacketScanData* p = (PacketScanData*) buf;

  if( !retrievePacket(packet_start,p) )
    return false;

  // Lock internal outgoing data queue, automatically unlocks at end of function
  std::unique_lock<std::mutex> lock(data_mutex_);

  // Create new scan container if necessary
  //if( htons(p->header.packet_number) == 1 || scan_data_.empty() )
  if( scan_data_.empty() )
  {
    scan_data_.emplace_back();

    if( scan_data_.size()>64 )
    {
      scan_data_.pop_front();

      std::cerr << "Too many scans in receiver queue: Dropping scans!" << std::endl;
    }
    data_notifier_.notify_one();
  }

  ScanData scandata;

  std::uint16_t* p_scan_data = (std::uint16_t*) &buf[27];

  std::uint16_t num_scan_points = htons(p->header.num_points_scan);

  for( int i=0; i<num_scan_points; i++ )
  {
    std::int16_t distance = htons(p_scan_data[i]);
    scandata.distance_data.push_back(distance);
  }

  memcpy( &scandata.header, &p->header, sizeof(scandata.header) );

  scan_data_.push_back(scandata);

  if(!pack_num)
    pack_num = p->header.total_number;

  if(!resolution)
    resolution = htons(p->header.angular_increment);

  if(!frequency)
    frequency = htons(p->header.scan_frequency);

  return true;
}



// find the header of frame from circular buffer (ring_buffer_)
int ScanDataReceiver::findPacketStart()
{
  if( ring_buffer_.size()<27 )
    return -1;
  for( std::size_t i=0; i<ring_buffer_.size()-4; i++)
  {
    if(   ((unsigned char) ring_buffer_[i])   == 0xBE
        && ((unsigned char) ring_buffer_[i+1]) == 0xA0
        && ((unsigned char) ring_buffer_[i+2]) == 0x12
        && ((unsigned char) ring_buffer_[i+3]) == 0x34 )
    {
      return i;
    }
  }

  return -2;
}

// get entire frame from circular buffer(ring_buffer_) (to temporary buffer)
bool ScanDataReceiver::retrievePacket(std::size_t start, PacketScanData *p)
{
  if( ring_buffer_.size()<27 )
    return false;

  // Erase preceding bytes
  ring_buffer_.erase_begin(start);

  char* pp = (char*) p;
  // Read header
  readBufferFront(pp,27);

  if( ring_buffer_.size() < htons(p->header.packet_size) )
    return false;

  // Read header+payload data
  readBufferFront(pp,htons(p->header.packet_size));

  // Erase packet from ring buffer
  ring_buffer_.erase_begin(htons(p->header.packet_size));
  return true;
}


// read from circular buffer(ring_buffer_)
void ScanDataReceiver::readBufferFront(char *dst, std::size_t numbytes)
{
  if( ring_buffer_.size() < numbytes )
    throw std::exception();
  char* pone = ring_buffer_.array_one().first;
  std::size_t pone_size = ring_buffer_.array_one().second;
  char* ptwo = ring_buffer_.array_two().first;
  //std::size_t ptwo_size = ring_buffer_.array_two().second;

  if( pone_size >= numbytes )
  {
    std::memcpy( dst, pone, numbytes );
  }
  else
  {
    std::memcpy( dst, pone, pone_size );
    std::memcpy( dst+pone_size, ptwo, numbytes-pone_size);
  }
}

// write to circular buffer (from array udp_buffer_)
void ScanDataReceiver::writeBufferBack(char *src, std::size_t numbytes)
{
  if( ring_buffer_.size()+numbytes > ring_buffer_.capacity() )
    throw std::exception();
  ring_buffer_.resize(ring_buffer_.size()+numbytes);
  char* pone = ring_buffer_.array_one().first;
  std::size_t pone_size = ring_buffer_.array_one().second;
  char* ptwo = ring_buffer_.array_two().first;
  std::size_t ptwo_size = ring_buffer_.array_two().second;

  if( ptwo_size >= numbytes )
  {
    std::memcpy(ptwo+ptwo_size-numbytes, src, numbytes);
  }
  else
  {
    std::memcpy(pone+pone_size+ptwo_size-numbytes,
                src,
                numbytes-ptwo_size );
    std::memcpy(ptwo,
                src+numbytes-ptwo_size,
                ptwo_size );
  }
}


}
