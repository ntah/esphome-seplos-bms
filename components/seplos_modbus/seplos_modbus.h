#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include <vector>
#include <string>

namespace esphome {
namespace seplos_modbus {

class SeplosModbus : public uart::UARTDevice, public Component {
 public:
  void setup() override;
  void loop() override;

  void send_ascii(uint8_t protocol, uint8_t address, uint8_t function, const std::string &payload);

 protected:
  bool parse_byte_(uint8_t data);
  bool parse_frame_(const std::string &frame);

  // CRC ASCII used by ZTE (CRC16 IBM)
  uint16_t crc16_ascii_(const std::string &ascii);

  void handle_packet_(uint8_t proto, uint8_t addr, uint8_t func, const std::string &frame);

  std::string rx_buffer_;
  bool in_frame_ = false;
};

}  // namespace seplos_modbus
}  // namespace esphome
