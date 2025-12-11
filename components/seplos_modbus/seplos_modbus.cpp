#include "seplos_modbus.h"
#include "esphome/core/log.h"

namespace esphome {
namespace seplos_modbus {

static const char *const TAG = "seplos_modbus";

// --------------------------------------------------
// SETUP
// --------------------------------------------------
void SeplosModbus::setup() {
  ESP_LOGI(TAG, "SeplosModbus initialized");
}

// --------------------------------------------------
// LOOP → Handle UART input
// --------------------------------------------------
void SeplosModbus::loop() {
  while (this->available()) {
    uint8_t c;
    this->read_byte(&c);
    this->parse_byte_(c);
  }
}

// --------------------------------------------------
// Parse byte-by-byte
// --------------------------------------------------
bool SeplosModbus::parse_byte_(uint8_t data) {
  if (data == '~') {
    this->in_frame_ = true;
    this->rx_buffer_.clear();
    this->rx_buffer_.push_back('~');
    return true;
  }

  if (!this->in_frame_)
    return false;

  this->rx_buffer_.push_back((char)data);

  if (data == '\r') {
    std::string frame = this->rx_buffer_;
    this->in_frame_ = false;
    this->parse_frame_(frame);
    return true;
  }

  return true;
}

// --------------------------------------------------
// Convert ASCII HEX to byte
// --------------------------------------------------
static uint8_t hex2(const char h, const char l) {
  auto cv = [](char c) -> int {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
  };
  int hi = cv(h);
  int lo = cv(l);
  if (hi < 0 || lo < 0) return 0xFF;
  return (hi << 4) | lo;
}

// --------------------------------------------------
// CRC16 IBM for ASCII payload
// --------------------------------------------------
uint16_t SeplosModbus::crc16_ascii_(const std::string &ascii) {
  uint16_t crc = 0xFFFF;

  for (char c : ascii) {
    crc ^= (uint8_t)c;
    for (int i = 0; i < 8; i++) {
      if (crc & 1)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }
  return crc;
}

// --------------------------------------------------
// FRAME PARSER — MODE A (Only proto=0x46, addr=0x03)
// --------------------------------------------------
bool SeplosModbus::parse_frame_(const std::string &frame) {
  if (frame.size() < 10 || frame[0] != '~') {
    ESP_LOGW(TAG, "Rejected frame: header invalid");
    return false;
  }

  uint8_t proto = hex2(frame[1], frame[2]);
  uint8_t addr  = hex2(frame[3], frame[4]);
  uint8_t func  = hex2(frame[5], frame[6]);

  // MODE A FILTER → Accept only packets for our device (proto=0x46 addr=0x03)
  if (proto != 0x46 || addr != 0x03) {
    ESP_LOGV(TAG, "Frame ignored proto=0x%02X addr=0x%02X", proto, addr);
    return false;
  }

  // Extract remote CRC (last 4 ASCII chars)
  uint16_t remote_crc =
      (hex2(frame[frame.size() - 4], frame[frame.size() - 3]) << 8) |
       hex2(frame[frame.size() - 2], frame[frame.size() - 1]);

  // Compute CRC on ASCII payload excluding first '~' and CRC itself
  std::string ascii_payload = frame.substr(1, frame.size() - 5);
  uint16_t calc_crc = this->crc16_ascii_(ascii_payload);

  if (remote_crc != calc_crc) {
    ESP_LOGW(TAG, "CRC mismatch: remote=0x%04X calc=0x%04X", remote_crc, calc_crc);
    return false;
  }

  ESP_LOGI(TAG, "VALID FRAME proto=0x%02X addr=0x%02X func=0x%02X", proto, addr, func);

  this->handle_packet_(proto, addr, func, frame);
  return true;
}

// --------------------------------------------------
// SEND ASCII FRAME
// --------------------------------------------------
void SeplosModbus::send_ascii(uint8_t proto, uint8_t addr, uint8_t func, const std::string &payload) {
  std::string body;

  auto append_hex = [&](uint8_t b) {
    const char *hex = "0123456789ABCDEF";
    body.push_back(hex[(b >> 4) & 0x0F]);
    body.push_back(hex[b & 0x0F]);
  };

  append_hex(proto);
  append_hex(addr);
  append_hex(func);

  body += payload;

  uint16_t crc = this->crc16_ascii_(body);

  char crc_hex[5];
  sprintf(crc_hex, "%04X", crc);

  std::string frame = "~" + body + crc_hex + "\r";

  this->write_str(frame);
  ESP_LOGI(TAG, "TX: %s", frame.c_str());
}

// --------------------------------------------------
// PACKET HANDLER (you can modify later)
// --------------------------------------------------
void SeplosModbus::handle_packet_(uint8_t proto, uint8_t addr, uint8_t func, const std::string &frame) {
  ESP_LOGI(TAG, "Processing frame: %s", frame.c_str());
}

}  // namespace seplos_modbus
}  // namespace esphome
