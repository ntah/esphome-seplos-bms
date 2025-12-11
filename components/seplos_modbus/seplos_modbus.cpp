#include "seplos_modbus.h"
#include "esphome/core/log.h"
#include <ctype.h>

namespace esphome {
namespace seplos_modbus {

static const char *const TAG = "seplos_modbus";

// Fallback RX timeout (ms) for assembling large frames (Shoto telemetry)
static const uint32_t RX_TIMEOUT_FALLBACK = 40U;

// --------------------------
// CRC helpers (CRC16-MODBUS on ASCII bytes)
// --------------------------
static uint16_t crc16_ascii(const std::string &ascii) {
  uint16_t crc = 0xFFFF;
  for (unsigned char ch : ascii) {
    crc ^= static_cast<uint16_t>(ch);
    for (uint8_t i = 0; i < 8; ++i) {
      if (crc & 1)
        crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
      else
        crc = static_cast<uint16_t>(crc >> 1);
    }
  }
  return crc;
}

static int hexval(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
  if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
  return -1;
}

static uint8_t ascii_hex_to_byte(char a, char b) {
  int hi = hexval(a);
  int lo = hexval(b);
  if (hi < 0 || lo < 0) return 0xFF;
  return static_cast<uint8_t>((hi << 4) | lo);
}

// --------------------------
// Setup / Loop
// --------------------------
void SeplosModbus::setup() {
  ESP_LOGCONFIG(TAG, "SeplosModbus setup()");
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->setup();

  if (this->rx_timeout_ == 0)
    this->rx_timeout_ = RX_TIMEOUT_FALLBACK;
}

void SeplosModbus::loop() {
  uint8_t c;
  while (this->available()) {
    this->read_byte(&c);
    this->parse_seplos_modbus_byte_(c);
  }

  // housekeeping: clear buffer on long inactivity
  const uint32_t now = millis();
  uint32_t to_use = (this->rx_timeout_ == 0 ? RX_TIMEOUT_FALLBACK : this->rx_timeout_);
  if (now - this->last_seplos_modbus_byte_ > to_use) {
    if (!this->rx_buffer_.empty()) {
      ESP_LOGV(TAG, "RX timeout — clearing incomplete buffer (len=%u)", (unsigned)this->rx_buffer_.size());
      this->rx_buffer_.clear();
    }
    this->last_seplos_modbus_byte_ = now;
  }
}

// --------------------------
// Send (kept compatible)
// --------------------------
void SeplosModbus::send(uint8_t protocol_version, uint8_t address, uint8_t function, uint8_t value) {
  uint8_t raw[7];
  raw[0] = protocol_version;
  raw[1] = address;
  raw[2] = 0x46;   // CID1
  raw[3] = function;
  raw[4] = 0xE0;
  raw[5] = 0x02;
  raw[6] = value;

  static const char HEX[] = "0123456789ABCDEF";
  std::string ascii_body;
  ascii_body.reserve(14);
  for (size_t i = 0; i < 7; ++i) {
    ascii_body.push_back(HEX[(raw[i] >> 4) & 0xF]);
    ascii_body.push_back(HEX[raw[i] & 0xF]);
  }

  uint16_t crc = crc16_ascii(ascii_body);
  uint8_t crc_hi = static_cast<uint8_t>((crc >> 8) & 0xFF);
  uint8_t crc_lo = static_cast<uint8_t>(crc & 0xFF);

  std::string frame = "~";
  frame += ascii_body;
  frame.push_back(HEX[(crc_hi >> 4) & 0xF]);
  frame.push_back(HEX[crc_hi & 0xF]);
  frame.push_back(HEX[(crc_lo >> 4) & 0xF]);
  frame.push_back(HEX[crc_lo & 0xF]);
  frame.push_back('\r');

  ESP_LOGD(TAG, "TX: %s", frame.c_str());

  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(true);

  this->write_str(frame.c_str());
  this->flush();

  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(false);

  this->last_send_ = millis();
}

// --------------------------
// Parser: collect bytes until '\r', then validate CRC and forward
// --------------------------
bool SeplosModbus::parse_seplos_modbus_byte_(uint8_t byte) {
  const uint32_t now = millis();

  // reset buffer if rx timeout exceeded
  uint32_t to_use = (this->rx_timeout_ == 0 ? RX_TIMEOUT_FALLBACK : this->rx_timeout_);
  if (now - this->last_seplos_modbus_byte_ > to_use && !this->rx_buffer_.empty()) {
    ESP_LOGV(TAG, "RX timeout detected, clearing buffer before appending new data");
    this->rx_buffer_.clear();
  }
  this->last_seplos_modbus_byte_ = now;

  // Append byte
  this->rx_buffer_.push_back(static_cast<char>(byte));

  // Require first char to be '~'
  if (this->rx_buffer_.size() == 1) {
    if (this->rx_buffer_[0] != '~') {
      ESP_LOGV(TAG, "Dropped leading non-~ byte 0x%02X", (unsigned)byte);
      this->rx_buffer_.clear();
      return false;
    }
    return true;
  }

  // protect runaway
  if (this->rx_buffer_.size() > 4096) {
    ESP_LOGW(TAG, "RX buffer too large (%u), flushing", (unsigned)this->rx_buffer_.size());
    this->rx_buffer_.clear();
    return false;
  }

  // Wait until CR terminator
  if (byte != '\r') {
    return true;
  }

  // We have frame from rx_buffer_[0] == '~' to last == '\r'
  // Build ascii_hex excluding '~' and '\r'
  std::string ascii_hex;
  ascii_hex.reserve(this->rx_buffer_.size());
  for (size_t i = 1; i + 1 < this->rx_buffer_.size(); ++i) {
    char c = this->rx_buffer_[i];
    if (!isxdigit((unsigned char)c)) {
      ESP_LOGW(TAG, "Non-hex char in frame -> drop (0x%02X)", (unsigned char)c);
      this->rx_buffer_.clear();
      return false;
    }
    ascii_hex.push_back(c);
  }

  if (ascii_hex.size() < 10 || (ascii_hex.size() % 2) != 0) {
    ESP_LOGW(TAG, "Invalid ascii_hex length=%u -> drop", (unsigned)ascii_hex.size());
    this->rx_buffer_.clear();
    return false;
  }

  // Determine protocol (first byte of ascii_hex)
  // ascii_hex[0..1] = protocol
  uint8_t proto = ascii_hex.size() >= 2 ? ascii_hex_to_byte(ascii_hex[0], ascii_hex[1]) : 0xFF;

  // dynamic minimum length:
  // - Shoto/Boqiang (proto 0x26) produce long telemetry frames (~60-100 bytes). Enforce higher min.
  // - Default minimum is modest (12).
  size_t min_ascii_len = 12; // ascii hex chars (so 12 -> 6 bytes)
  if (proto == 0x26) {
    min_ascii_len = 120; // 120 hex chars = 60 bytes (binary) — safe lower bound for Shoto telemetry
  }

  if (ascii_hex.size() < min_ascii_len) {
    ESP_LOGV(TAG, "Frame for proto 0x%02X too short (%u < %u) -> drop", proto, (unsigned)ascii_hex.size(), (unsigned)min_ascii_len);
    this->rx_buffer_.clear();
    return false;
  }

  // split ascii into body and CRC (last 4 chars)
  const size_t crc_pos = ascii_hex.size() - 4;
  std::string ascii_without_crc = ascii_hex.substr(0, crc_pos);
  std::string ascii_crc_str = ascii_hex.substr(crc_pos, 4);

  // compute CRC over ascii_without_crc
  uint16_t calc_crc = crc16_ascii(ascii_without_crc);

  // parse remote CRC (big-endian: hi byte then lo byte)
  uint8_t remote_hi = ascii_hex_to_byte(ascii_crc_str[0], ascii_crc_str[1]);
  uint8_t remote_lo = ascii_hex_to_byte(ascii_crc_str[2], ascii_crc_str[3]);
  uint16_t remote_crc = (static_cast<uint16_t>(remote_hi) << 8) | remote_lo;

  if (remote_crc != calc_crc) {
    ESP_LOGW(TAG, "CRC FAILED: remote=0x%04X calc=0x%04X", remote_crc, calc_crc);
    this->rx_buffer_.clear();
    return false;
  }

  // decode ascii_without_crc into binary bytes
  const size_t bin_len = ascii_without_crc.size() / 2;
  std::vector<uint8_t> data;
  data.reserve(bin_len);
  for (size_t i = 0; i < bin_len; ++i) {
    char hi = ascii_without_crc[i * 2];
    char lo = ascii_without_crc[i * 2 + 1];
    uint8_t b = ascii_hex_to_byte(hi, lo);
    data.push_back(b);
  }

  if (data.size() < 4) {
    ESP_LOGW(TAG, "Decoded binary too short -> drop");
    this->rx_buffer_.clear();
    return false;
  }

  // Forward decoded data to registered devices matching address (data[1])
  uint8_t addr = data[1];
  bool forwarded = false;
  for (auto *dev : this->devices_) {
    if (dev->address_ == addr) {
      dev->on_seplos_modbus_data(data);
      forwarded = true;
    }
  }
  if (!forwarded) {
    ESP_LOGV(TAG, "Valid frame for address 0x%02X but no matching device registered", addr);
  }

  // IMPORTANT: clear buffer and return true (do not return false which may reset parser state)
  this->rx_buffer_.clear();
  return true;
}

// --------------------------
// Component required methods
// --------------------------
void SeplosModbus::dump_config() {
  ESP_LOGCONFIG(TAG, "SeplosModbus:");
  LOG_PIN("  Flow Control Pin: ", this->flow_control_pin_);
  ESP_LOGCONFIG(TAG, "  RX timeout: %u ms", (unsigned)this->rx_timeout_);
}

float SeplosModbus::get_setup_priority() const {
  return setup_priority::BUS - 1.0f;
}

}  // namespace seplos_modbus
}  // namespace esphome
