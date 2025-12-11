#include "seplos_modbus.h"
#include "esphome/core/log.h"

namespace esphome {
namespace seplos_modbus {

static const char *const TAG = "seplos_modbus";

/**
 * CRC16 (already declared in .h)
 */
uint16_t crc16(const uint8_t *data, uint8_t len) {
  uint16_t crc = 0xFFFF;

  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 1)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }
  return crc;
}

void SeplosModbus::setup() {
  ESP_LOGCONFIG(TAG, "Setting up SeplosModbus...");
}

/**
 * SEND FRAME (original API preserved)
 */
void SeplosModbus::send(uint8_t protocol_version, uint8_t address, uint8_t function, uint8_t value) {
  this->last_send_ = millis();

  uint8_t payload[4];
  payload[0] = protocol_version;   // usually 0x21
  payload[1] = address;
  payload[2] = 0x46;
  payload[3] = function;

  uint16_t crc = crc16(payload, 4);
  uint16_t crc_swap = (crc >> 8) | (crc << 8);

  char frame[32];
  sprintf(frame, "~%02X%02X%02X%02X%04X\r",
          payload[0], payload[1], payload[2], payload[3], crc_swap);

  ESP_LOGD(TAG, "TX: %s", frame);
  this->write_str(frame);
}

/**
 * PARSE BYTE RX (ASCII HEX Modbus)
 */
bool SeplosModbus::parse_seplos_modbus_byte_(uint8_t c) {
  uint32_t now = millis();

  if (now - this->last_seplos_modbus_byte_ > this->rx_timeout_) {
    this->rx_buffer_.clear();
  }

  this->last_seplos_modbus_byte_ = now;

  if (c == '~') {
    this->rx_buffer_.clear();
    this->rx_buffer_.push_back(c);
    return false;
  }

  this->rx_buffer_.push_back(c);

  if (c != '\r')
    return false;

  // CLEAN ASCII HEX
  std::string hex;
  for (auto ch : this->rx_buffer_) {
    if (isxdigit(ch))
      hex.push_back(ch);
  }

  if (hex.length() < 12) {
    ESP_LOGW(TAG, "Frame too short: %s", hex.c_str());
    return false;
  }

  size_t len = hex.length() / 2;
  std::vector<uint8_t> data(len);

  for (size_t i = 0; i < len; i++) {
    char hi = hex[i * 2];
    char lo = hex[i * 2 + 1];
    data[i] = (uint8_t) strtol((std::string() + hi + lo).c_str(), nullptr, 16);
  }

  if (len < 6)
    return false;

  uint8_t protocol = data[0];
  uint8_t addr     = data[1];
  uint8_t cid1     = data[2];
  uint8_t cid2     = data[3];

  // CRC CHECK
  uint16_t remote_crc = ((uint16_t)data[len-2] << 8) | data[len-1];
  uint16_t calc_crc = crc16(data.data(), len - 2);

  if (remote_crc != calc_crc) {
    ESP_LOGW(TAG, "CRC error: remote=0x%04X calc=0x%04X", remote_crc, calc_crc);
    return false;
  }

  // FILTER MODE A
  for (auto *dev : this->devices_) {
    if (dev->address_ != addr)
      continue;  // ignore other addresses

    if (cid1 != 0x46)
      continue; // Seplos/ZTE always uses 0x46

    bool cid2_valid =
        (cid2 == 0x00) ||          // telemetry
        (cid2 == dev->protocol_version_); // reply (protocol_version holds CID2 expected)

    if (!cid2_valid) {
      ESP_LOGV(TAG,
               "Ignoring frame A=0x%02X CID1=0x%02X CID2=0x%02X (expected CID2=%02X or 00)",
               addr, cid1, cid2, dev->protocol_version_);
      continue;
    }

    // VALID FRAME → forward to correct device
    dev->on_seplos_modbus_data(data);
  }

  return true;
}

void SeplosModbus::loop() {
  uint8_t c;
  while (this->available()) {
    this->read_byte(&c);
    this->parse_seplos_modbus_byte_(c);
  }
}

void SeplosModbus::dump_config() {
  ESP_LOGCONFIG(TAG, "SeplosModbus:");
  ESP_LOGCONFIG(TAG, "  RX Timeout: %u ms", this->rx_timeout_);
}

float SeplosModbus::get_setup_priority() const {
  return setup_priority::BUS - 1.0f;
}

}  // namespace seplos_modbus
}  // namespace esphome
