#include "zte_fb101.h"

void ZTEFB101Parser::parse(const std::vector<uint8_t>& data) {
  parse_cells(data);
  parse_status(data);
  parse_cycles(data);
}

void ZTEFB101Parser::parse_cycles(const std::vector<uint8_t>& data) {
  // FB101 cycle offset (FIXED)
  int off = DATA_LENGTH - 6; // contoh — akan saya sesuaikan setelah lihat frame
  uint16_t cyc = (data[off] << 8) | data[off+1];
  ESP_LOGD("fb101", "Charging cycles = %u", cyc);
  // publish to sensor
}
