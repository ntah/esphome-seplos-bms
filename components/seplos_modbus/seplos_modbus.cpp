#include "seplos_modbus.h"
#include "esphome/core/log.h"
#include <ctype.h>

namespace esphome {
namespace seplos_modbus {

static const char *const TAG = "seplos_modbus";

/* CRC16-MODBUS computed over ASCII characters (each char's byte value) */
static uint16_t crc16_ascii(const std::string &ascii) {
  uint16_t crc = 0xFFFF;
  for (unsigned char ch : ascii) {
    crc ^= (uint16_t)ch;
    for (uint8_t i = 0; i < 8; ++i) {
      if (crc & 1)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }
  return crc;
}

/* Convert two ASCII hex chars to byte (expects valid hex) */
static uint8_t ascii_hex_to_byte(char a, char b) {
  auto val = [](char c)->uint8_t {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    return 0;
  };
  return static_cast<uint8_t>((val(a) << 4) | val(b));
}

void SeplosModbus::setup() {
  ESP_LOGCONFIG(TAG, "SeplosModbus setup()");
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->setup();
}

/* preserve original send signature */
void SeplosModbus::send(uint8_t protocol_version, uint8_t address, uint8_t function, uint8_t value) {
  // Build minimal raw bytes as observed (VER, ADDR, CID1=0x46, CID2=function, LEN hi/lo, value)
  uint8_t raw[7];
  raw[0] = protocol_version;
  raw[1] = address;
  raw[2] = 0x46;
  raw[3] = function;
  raw[4] = 0xE0;  // keep observed LEN high
  raw[5] = 0x02;  // keep observed LEN low
  raw[6] = value;

  // convert to ASCII hex body (no '~', no CRC)
  static const char HEX[] = "0123456789ABCDEF";
  std::string ascii_body;
  ascii_body.reserve(14);
  for (size_t i = 0; i < 7; ++i) {
    ascii_body.push_back(HEX[(raw[i] >> 4) & 0xF]);
    ascii_body.push_back(HEX[raw[i] & 0xF]);
  }

  // compute CRC over ASCII body
  uint16_t crc = crc16_ascii(ascii_body);
  uint8_t crc_hi = (crc >> 8) & 0xFF;
  uint8_t crc_lo = crc & 0xFF;

  // build final ASCII frame: "~" + ascii_body + CRC(hi,lo) as two bytes each -> 4 hex chars + "\r"
  std::string frame = "~";
  frame += ascii_body;

  // append CRC high byte
  frame.push_back(HEX[(crc_hi >> 4) & 0xF]);
  frame.push_back(HEX[crc_hi & 0xF]);
  // append CRC low byte
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

/* Parse incoming bytes (ASCII Modbus-like). Only CRC check is enforced. */
bool SeplosModbus::parse_seplos_modbus_byte_(uint8_t byte) {
  const uint32_t now = millis();

  // reset on rx timeout
  if (now - this->last_seplos_modbus_byte_ > this->rx_timeout_) {
    this->rx_buffer_.clear();
  }
  this->last_seplos_modbus_byte_ = now;

  // append byte
  this->rx_buffer_.push_back(byte);

  // wait for start char
  if (this->rx_buffer_.size() == 1) {
    if (this->rx_buffer_[0] != '~') {
      // not a frame start, drop buffer
      this->rx_buffer_.clear();
      return false;
    }
    return true;
  }

  // wait until CR
  if (byte != '\r') {
    // prevent runaway
    if (this->rx_buffer_.size() > 1024) {
      ESP_LOGW(TAG, "RX buffer overflow — clearing");
      this->rx_buffer_.clear();
    }
    return true;
  }

  // build ascii_hex string excluding leading '~' and trailing '\r'
  std::string ascii_hex;
  ascii_hex.reserve(this->rx_buffer_.size());
  for (size_t i = 1; i + 1 < this->rx_buffer_.size(); ++i) {
    char c = static_cast<char>(this->rx_buffer_[i]);
    if (!isxdigit((unsigned char)c)) {
      ESP_LOGW(TAG, "Non-hex char in frame -> drop");
      this->rx_buffer_.clear();
      return false;
    }
    ascii_hex.push_back(c);
  }

  // ascii_hex length must be even and >= 10 (VER(2)+ADDR(2)+CID1(2)+CID2(2)+CRC(4) => 12, but be lenient)
  if (ascii_hex.size() < 10 || (ascii_hex.size() % 2) != 0) {
    ESP_LOGW(TAG, "Invalid ascii hex length=%u -> drop", (unsigned)ascii_hex.size());
    this->rx_buffer_.clear();
    return false;
  }

  // split ascii into body and remote CRC (last 4 hex chars)
  if (ascii_hex.size() < 4 + 2*4) { // ensure room for header+crc
    // still reject if too short
    ESP_LOGW(TAG, "Frame too short after clean -> drop");
    this->rx_buffer_.clear();
    return false;
  }

  std::string ascii_without_crc = ascii_hex.substr(0, ascii_hex.size() - 4);
  std::string ascii_crc_str = ascii_hex.substr(ascii_hex.size() - 4);

  // compute crc over ASCII body
  uint16_t calc_crc = crc16_ascii(ascii_without_crc);

  // parse remote crc (big-endian: hi byte then lo byte)
  uint8_t remote_hi = ascii_hex_to_byte(ascii_crc_str[0], ascii_crc_str[1]);
  uint8_t remote_lo = ascii_hex_to_byte(ascii_crc_str[2], ascii_crc_str[3]);
  uint16_t remote_crc = (static_cast<uint16_t>(remote_hi) << 8) | remote_lo;

  if (remote_crc != calc_crc) {
    ESP_LOGW(TAG, "CRC error: remote=0x%04X calc=0x%04X", remote_crc, calc_crc);
    this->rx_buffer_.clear();
    return false;
  }

  // decode binary bytes from ascii_without_crc
  const size_t bin_len = ascii_without_crc.size() / 2;
  std::vector<uint8_t> data;
  data.reserve(bin_len);
  for (size_t i = 0; i < bin_len; ++i) {
    char hi = ascii_without_crc[i*2];
    char lo = ascii_without_crc[i*2 + 1];
    data.push_back(ascii_hex_to_byte(hi, lo));
  }

  // sanity: need at least protocol+addr+cid1+cid2
  if (data.size() < 4) {
    ESP_LOGW(TAG, "Decoded frame too short -> drop");
    this->rx_buffer_.clear();
    return false;
  }

  uint8_t addr = data[1];

  // Forward decoded data vector to matching device(s) by address
  for (auto *dev : this->devices_) {
    if (dev->address_ == addr) {
      dev->on_seplos_modbus_data(data);
    }
  }

  // clear buffer after successful parse
  this->rx_buffer_.clear();
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
  LOG_PIN("  Flow Control Pin: ", this->flow_control_pin_);
  ESP_LOGCONFIG(TAG, "  RX timeout: %u ms", this->rx_timeout_);
}

float SeplosModbus::get_setup_priority() const {
  return setup_priority::BUS - 1.0f;
}

}  // namespace seplos_modbus
}  // namespace esphome
