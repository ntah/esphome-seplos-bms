#include "seplos_modbus.h"
#include "esphome/core/log.h"
#include <ctype.h>

namespace esphome {
namespace seplos_modbus {

static const char *const TAG = "seplos_modbus";

// Local expected-reply signature (kept in .cpp so header tidak perlu diubah)
static uint8_t expected_addr_ = 0xFF;
static uint8_t expected_cid1_ = 0xFF;
static uint8_t expected_cid2_ = 0xFF;

// --------------------------
// CRC helpers
// --------------------------
// Standard CRC16-MODBUS on binary data (kept for completeness)
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

// CRC16-MODBUS but computed over ASCII characters (each char's byte value)
static uint16_t crc16_ascii(const std::string &ascii_hex) {
  uint16_t crc = 0xFFFF;
  for (char ch : ascii_hex) {
    crc ^= static_cast<uint8_t>(ch);
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 1)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }
  return crc;
}

// Convert two ASCII hex chars to byte (expects valid hex)
static uint8_t ascii_hex_to_byte(char a, char b) {
  auto val = [](char c) -> uint8_t {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    return 0;
  };
  return static_cast<uint8_t>((val(a) << 4) | val(b));
}

// --------------------------
// Setup & loop
// --------------------------

void SeplosModbus::setup() {
  ESP_LOGCONFIG(TAG, "SeplosModbus setup()");
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->setup();
}

void SeplosModbus::loop() {
  uint8_t c;
  while (this->available()) {
    this->read_byte(&c);
    // parse byte stream (Modbus ASCII style)
    this->parse_seplos_modbus_byte_(c);
  }

  // housekeeping for RX timeout
  const uint32_t now = millis();
  if (now - this->last_seplos_modbus_byte_ > this->rx_timeout_) {
    if (!this->rx_buffer_.empty()) {
      ESP_LOGVV(TAG, "RX timeout — clearing buffer (len=%u)", (unsigned)this->rx_buffer_.size());
      this->rx_buffer_.clear();
    }
    this->last_seplos_modbus_byte_ = now;
  }
}

// --------------------------
// Send frame (original signature preserved)
// - protocol_version: VER (e.g. 0x21)
// - address: device address (0x03)
// - function: CID2 (e.g. 0x42)
// - value: single byte value used by some commands
// --------------------------
void SeplosModbus::send(uint8_t protocol_version, uint8_t address, uint8_t function, uint8_t value) {
  // Save expected reply signature (strict reply matching)
  expected_addr_ = address;
  expected_cid1_ = 0x46;   // as used by Seplos/ZTE in your setup
  expected_cid2_ = function;

  // Build ASCII payload (we'll format as "~" + ascii hex + CRC + "\r")
  // The "body" we compute CRC over is the ASCII hex string without leading '~' and without CRC itself.
  // For consistency with observed frames, we'll construct bytes array then convert to ASCII for CRC.
  // Bytes: VER, ADDR, CID1, CID2, LEN_HI, LEN_LO, VALUE ... (we'll use basic frame with minimal len)
  // The exact content/length depends on higher level; keep frame minimal for control writes.
  uint8_t raw_bytes[7];
  raw_bytes[0] = protocol_version;  // e.g. 0x21
  raw_bytes[1] = address;
  raw_bytes[2] = 0x46;
  raw_bytes[3] = function;
  // use length E0 02 (observed in logs) or compute minimal, here keep E0 02 for compatibility
  raw_bytes[4] = 0xE0;
  raw_bytes[5] = 0x02;
  raw_bytes[6] = value;

  // convert raw_bytes to ASCII hex string (no '~' and no CRC yet)
  std::string ascii_body;
  ascii_body.reserve(2 * 7);
  static const char HEX_CH[] = "0123456789ABCDEF";
  for (size_t i = 0; i < 7; ++i) {
    ascii_body.push_back(HEX_CH[(raw_bytes[i] >> 4) & 0xF]);
    ascii_body.push_back(HEX_CH[raw_bytes[i] & 0xF]);
  }

  // compute CRC on ASCII body
  uint16_t crc = crc16_ascii(ascii_body);
  // remote CRC in frames appears as two bytes high-first (big-endian) when appended as ASCII hex
  uint8_t crc_hi = static_cast<uint8_t>((crc >> 8) & 0xFF);
  uint8_t crc_lo = static_cast<uint8_t>(crc & 0xFF);

  // build full ASCII payload
  std::string payload = "~";
  payload += ascii_body;
  payload.push_back(HEX_CH[(crc_hi >> 4) & 0xF]);
  payload.push_back(HEX_CH[crc_hi & 0xF]);
  payload.push_back(HEX_CH[(crc_lo >> 4) & 0xF]);
  payload.push_back(HEX_CH[crc_lo & 0xF]);
  payload.push_back('\r');

  ESP_LOGD(TAG, "Send frame: %s", payload.c_str());

  // send out
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(true);

  this->write_str(payload.c_str());
  this->flush();

  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(false);

  this->last_send_ = millis();
}

// --------------------------
// Parser: read incoming ASCII stream and validate frames
// --------------------------
bool SeplosModbus::parse_seplos_modbus_byte_(uint8_t byte) {
  const uint32_t now = millis();

  // update last byte ts
  this->last_seplos_modbus_byte_ = now;

  // push byte into rx buffer
  this->rx_buffer_.push_back(byte);

  // start detects
  if (this->rx_buffer_.size() == 1) {
    if (this->rx_buffer_[0] != '~') {
      // not a valid start, drop
      ESP_LOGV(TAG, "Dropped non-~ byte at buffer start: 0x%02X", this->rx_buffer_[0]);
      this->rx_buffer_.clear();
      return false;
    }
    // wait for end
    return true;
  }

  // wait until '\r'
  if (byte != '\r') {
    // protect max size
    if (this->rx_buffer_.size() > 600) {
      ESP_LOGW(TAG, "RX buffer too large, flushing");
      this->rx_buffer_.clear();
      return false;
    }
    return true;
  }

  // At this point frame ended — validate
  // Build ASCII hex string (only hex chars) from rx_buffer_ excluding leading '~' and trailing '\r'
  std::string ascii_hex;
  ascii_hex.reserve(this->rx_buffer_.size());
  for (size_t i = 1; i + 1 < this->rx_buffer_.size(); ++i) { // skip leading '~' and trailing '\r'
    char c = static_cast<char>(this->rx_buffer_[i]);
    if (isxdigit((unsigned char)c))
      ascii_hex.push_back(c);
    else {
      // Non-hex in payload: reject frame
      ESP_LOGW(TAG, "Non-hex char in payload: 0x%02X - discarding frame", (unsigned char)c);
      this->rx_buffer_.clear();
      return false;
    }
  }

  // ascii_hex must be even length and at least 10 chars (VER+ADDR+CID1+CID2+CRC(4))
  if (ascii_hex.size() < 10 || (ascii_hex.size() % 2) != 0) {
    ESP_LOGW(TAG, "Invalid ascii_hex length=%u", (unsigned)ascii_hex.size());
    this->rx_buffer_.clear();
    return false;
  }

  // remote CRC is last 4 ascii hex chars
  std::string ascii_without_crc = ascii_hex.substr(0, ascii_hex.size() - 4);
  std::string ascii_crc_str = ascii_hex.substr(ascii_hex.size() - 4);

  // compute CRC on ascii_without_crc
  uint16_t calc_crc = crc16_ascii(ascii_without_crc);

  // parse remote CRC from ascii_crc_str (big-endian)
  uint8_t rch0 = ascii_crc_str[0], rch1 = ascii_crc_str[1], rch2 = ascii_crc_str[2], rch3 = ascii_crc_str[3];
  uint8_t remote_hi = ascii_hex_to_byte(rch0, rch1);
  uint8_t remote_lo = ascii_hex_to_byte(rch2, rch3);
  uint16_t remote_crc = (static_cast<uint16_t>(remote_hi) << 8) | remote_lo;

  if (remote_crc != calc_crc) {
    ESP_LOGW(TAG, "CRC error: remote=0x%04X calc=0x%04X", remote_crc, calc_crc);
    this->rx_buffer_.clear();
    return false;
  }

  // Convert ascii_without_crc to binary bytes vector
  size_t bin_len = ascii_without_crc.size() / 2;
  std::vector<uint8_t> data;
  data.reserve(bin_len);
  for (size_t i = 0; i < bin_len; ++i) {
    char hi = ascii_without_crc[i * 2];
    char lo = ascii_without_crc[i * 2 + 1];
    data.push_back(ascii_hex_to_byte(hi, lo));
  }

  // Basic sanity: need at least VER, ADDR, CID1, CID2
  if (data.size() < 4) {
    ESP_LOGW(TAG, "Binary data too short after decode");
    this->rx_buffer_.clear();
    return false;
  }

  uint8_t ver = data[0];
  uint8_t addr = data[1];
  uint8_t cid1 = data[2];
  uint8_t cid2 = data[3];

  // MODE A filtering:
  // Accept only frames where addr matches expected_addr_ (set by send or by device list)
  // and CID1==0x46 and (CID2 == 0x00 (telemetry) || CID2 == expected_cid2_)
  bool addr_match = false;
  // If devices_ list exists, check if any device has that addr; otherwise compare to expected_addr_
  for (auto *dev : this->devices_) {
    if (dev->address_ == addr) {
      addr_match = true;
      break;
    }
  }
  if (!addr_match) {
    // optionally allow if expected_addr_ matches
    if (expected_addr_ == addr)
      addr_match = true;
  }

  if (!addr_match) {
    ESP_LOGV(TAG, "Ignoring frame from address 0x%02X (not in device list or not expected)", addr);
    this->rx_buffer_.clear();
    return false;
  }

  if (cid1 != 0x46) {
    ESP_LOGV(TAG, "Ignoring frame with CID1=0x%02X (expected 0x46)", cid1);
    this->rx_buffer_.clear();
    return false;
  }

  bool cid2_ok = (cid2 == 0x00) || (cid2 == expected_cid2_);

  if (!cid2_ok) {
    ESP_LOGV(TAG, "Ignoring frame ADDR=0x%02X CID1=0x%02X CID2=0x%02X (expected CID2=0x%02X or telemetry CID2=0x00)",
             addr, cid1, cid2, expected_cid2_);
    this->rx_buffer_.clear();
    return false;
  }

  // Pass decoded data to the matching device(s)
  for (auto *dev : this->devices_) {
    if (dev->address_ == addr) {
      // forward vector<uint8_t> data to device handler
      dev->on_seplos_modbus_data(data);
    }
  }

  // Clear rx buffer after successful parse
  this->rx_buffer_.clear();
  return true;
}

// --------------------------
// Component required functions
// --------------------------
void SeplosModbus::dump_config() {
  ESP_LOGCONFIG(TAG, "SeplosModbus:");
  LOG_PIN("  Flow Control Pin: ", this->flow_control_pin_);
  ESP_LOGCONFIG(TAG, "  RX timeout: %u ms", this->rx_timeout_);
}

float SeplosModbus::get_setup_priority() const {
  return setup_priority::BUS - 1.0f;
}

}  // namespace seplos_modbus
}  // namespace esphome
