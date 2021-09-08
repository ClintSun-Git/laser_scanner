// BEA
// Clint Sun, 2021
//

#ifndef PACKET_STRUCTURE_H
#define PACKET_STRUCTURE_H

#include <cstdint>
#include <vector>

/*
MDI response, 80Hz, 0.2 degree resolution
SYNC: be a0 12 34
packet_type: 00
packet_size: 05 95
reserved_a: 00 00
reserved_b: 00 00
reserved_c: 00 00
packet_number: 0b 4b
total_number: 02
sub_number: 01
scan_frequency: 00 50, 80Hz
num_points_scan: 02 bc
first_angle: 58 e8, 227.6 degree
angular_increment: ff 38, -0.2 degree
timestamp_raw: ad 82
*/

/*
MDI response, 10Hz, 0.02 degree resolution
SYNC: be a0 12 34
packet_type: 00
packet_size: 05 95
reserved_a: 00 00
reserved_b: 00 00
reserved_c: 00 00
packet_number: 54 32
total_number: 10
sub_number: 01
scan_frequency: 00 0a, 10Hz
num_points_scan: 02 bc
first_angle: 58 e8
angular_increment: ff ec, -0.02 degree
timestamp_raw: ae cb
*/

namespace bea {

#pragma pack(1)

//-----------------------------------------------------------------------------
struct PacketHeader
{
  //! SYNC bytes, BE A0 12 34 (hex)
  std::uint32_t sync;

  //! Packet type, 0 or 1 (only distance or distance with intensity)
  std::uint8_t packet_type;

  //! Overall packet size with maxmium value 1429 bytes
  std::uint16_t packet_size;

  //! reserve bytes
  std::uint16_t reserved_a;
  std::uint16_t reserved_b;
  std::uint16_t reserved_c;

  //! Sequence number for packet (counting packets of a particular scan, starting with 1)
  std::uint16_t packet_number;

  //! total packet number for a complete scan
  std::uint8_t total_number;

  //! sub packet number
  std::uint8_t sub_number;

  //! Frequency of scan
  std::uint16_t scan_frequency;

  //! Total number of scan points (samples) within complete scan
  std::uint16_t num_points_scan;

  //! Absolute angle of first scan point
  std::int16_t first_angle;

  //! Delta between two succeding scan points
  std::int16_t angular_increment;

  std::uint16_t timestamp_raw;


};

//-----------------------------------------------------------------------------
struct PacketScanData
{
  PacketHeader header;
  std::uint16_t distance_amplitude_payload; // distance and intensity
  //std::uint16_t CRC;
};

#pragma pack()

//! Packet type 0, only distance
struct ScanData
{
  //! Distance data
  std::vector<std::uint16_t> distance_data;

  //! Amplitude data
  //std::vector<std::uint16_t> amplitude_data;

  //! Header received with the distance and amplitude data
  PacketHeader header;

};

}

#endif
