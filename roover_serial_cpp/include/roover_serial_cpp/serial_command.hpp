#ifndef ROOVER_SERIAL_CPP__SERIAL_COMMAND_HPP_
#define ROOVER_SERIAL_CPP__SERIAL_COMMAND_HPP_

#include <cstdint>

// Zelfde start-frame als in je firmware config.h
static constexpr uint16_t SERIAL_START_FRAME = 0xABCD;  // pas aan naar jouw waarde

/// Structuur van het pakket dat we over UART sturen
struct __attribute__((packed)) SerialCommand
{
  uint16_t start;    // startframe
  int16_t  steer;    // -1000…1000
  int16_t  speed;    // -1000…1000
  uint16_t checksum; // simple XOR: start ^ steer ^ speed
};

#endif  // ROOVER_SERIAL_CPP__SERIAL_COMMAND_HPP_
