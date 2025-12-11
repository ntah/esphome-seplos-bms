#include "seplos_modbus.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <ctype.h>

namespace esphome {
namespace seplos_modbus {

static const char *const TAG = "seplos_modbus";

/**
 * Compute CRC16 (Modbus ASCII style, NOT RTU)
 */
uint16_t SeplosModbus::crc16_(const uint8_t *buffer, uint16_t length) {
  uint16_t crc = 0xFFFF;

  for (uint16_t i = 0; i < length; i++) {
    crc ^= buffer[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }
  return crc;
}

/**
 * Send a Modbus ASCII frame to ZTE/Seplos BMS
 */
void SeplosModbus::send(uint8_t address, uint8_t function, uint16_t value) {
  this->expected_addr_ = address;
  this->expected_cid1_ = 0x46;        // Always 46 for ZTE/Seplos
  this->expected_cid2_ = function;    // Example 0x42

  char frame[32];
  uint8_t raw[4];
  raw[0] = 0x21;      // Header '~' already added by UART debug
  raw[1] = address;
  raw[2] = 0x46;
  raw[3] = function;

  uint16_t crc = this->crc16_(raw, 4);
  uint16_t crc_be = (crc >> 8) | (crc << 8);

  sprintf(frame,
          "~%02X%02X%02X%02X%04X\r",
          raw[0], raw[1], raw[2], raw[3], crc_be);

  ESP_LOGD(TAG, "TX: %s", frame);
  this->parent_->write_str(frame);
}

/**
 * Parse single character (Modbus ASCII stream parser)
 */
bool SeplosModbus::parse_seplos_modbus_byte_(uint8_t c) {
  const uint32_t now = millis();

  if (now - this->last_rx_ts_ > this->rx_timeout_) {
    this->rx_buffer_.clear();
  }
  this->last_rx_ts_ = now;

  // Start of frame '~'
  if (c == '~') {
    this->rx_buffer_.clear();
    this->rx_buffer_.push_back(c);
    return false;
  }

  // Accumulate
  this->rx_buffer_.push_back(c);

  // Frame ends with '\r'
  if (c != '\r')
    return false;

  // Clean ASCII hex payload
  std::string hex;
  hex.reserve(rx_buffer_.size());

  for (char ch : rx_buffer_) {
    if (isxdigit(ch))
      hex.push_back(ch);
  }

  if (hex.length() < 10) {
    ESP_LOGW(TAG, "Frame too short (len=%u)", (unsigned)hex.length());
    return false;
  }

  // Convert ASCII hex → binary
  const size_t len = hex.length() / 2;
  std::vector<uint8_t> data(len);

  for (size_t i = 0; i < len; i++) {
    char hi = hex[i * 2];
    char lo = hex[i * 2 + 1];
    data[i] = (uint8_t) strtol((std::string()+hi+lo).c_str(), nullptr, 16);
  }

  if (len < 6) {
    ESP_LOGW(TAG, "Invalid binary length (%u)", (unsigned)len);
    return false;
  }

  uint8_t addr = data[1];
  uint8_t cid1 = data[2];
  uint8_t cid2 = data[3];

  // Compute CRC
  uint16_t remote_crc = ((uint16_t)data[len-2] << 8) | data[len-1];
  uint16_t calc_crc   = crc16_(data.data(), len - 2);

  if (remote_crc != calc_crc) {
    ESP_LOGW(TAG, "CRC mismatch remote=0x%04X calc=0x%04X", remote_crc, calc_crc);
    return false;
  }

  // MODE A FILTER:
  // Accept telemetry (CID2=00) OR reply (CID2=expected)
  bool cid2_valid =
      (cid2 == 0x00) ||        // Telemetry data frame
      (cid2 == this->expected_cid2_);

  if (addr != this->expected_addr_ || cid1 != this->expected_cid1_ || !cid2_valid) {
    ESP_LOGV(TAG, "Ignoring frame ADDR=0x%02X CID1=0x%02X CID2=0x%02X "
                  "(expect ADDR=0x%02X CID1=0x%02X CID2=%02X or telemetry CID2=00)",
             addr, cid1, cid2,
             this->expected_addr_, this->expected_cid1_, this->expected_cid2_);
    return false;
  }

  // Valid data → forward to device
  if (this->device_ != nullptr)
    this->device_->on_seplos_modbus_data(data);

  return true;
}

/**
 * UART on_receive()
 */
void SeplosModbus::loop() {
  uint8_t c;
  while (this->available()) {
    this->read_byte(&c);
    this->parse_seplos_modbus_byte_(c);
  }
}

/**
 * Config dump (required)
 */
void SeplosModbus::dump_config() {
  ESP_LOGCONFIG(TAG, "SeplosModbus:");
  ESP_LOGCONFIG(TAG, "  RX Timeout: %u ms", this->rx_timeout_);
  if (this->flow_control_pin_)
    LOG_PIN("  Flow Control Pin: ", this->flow_control_pin_);
}

/**
 * Setup priority (required)
 */
float SeplosModbus::get_setup_priority() const {
  return setup_priority::BUS - 1.0f;
}

}  // namespace seplos_modbus
}  // namespace esphome
