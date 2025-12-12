#include "seplos_modbus.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace seplos_modbus {

static const char *const TAG = "seplos_modbus";

static const uint16_t MAX_RESPONSE_SIZE = 340;

void SeplosModbus::setup() {
  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->setup();
  }
}

void SeplosModbus::loop() {
  const uint32_t now = millis();

  // ---------------------------------------------------------
  // Receive handling
  // ---------------------------------------------------------
  if (now - this->last_seplos_modbus_byte_ > this->rx_timeout_) {
    ESP_LOGVV(TAG, "Buffer cleared due to timeout: %s",
              format_hex_pretty(&this->rx_buffer_.front(), this->rx_buffer_.size()).c_str());
    this->rx_buffer_.clear();
    this->last_seplos_modbus_byte_ = now;
  }

  while (this->available()) {
    uint8_t byte;
    this->read_byte(&byte);

    if (this->parse_seplos_modbus_byte_(byte)) {
      this->last_seplos_modbus_byte_ = now;
    } else {
      ESP_LOGVV(TAG, "Buffer cleared due to reset: %s",
                format_hex_pretty(&this->rx_buffer_.front(), this->rx_buffer_.size()).c_str());
      this->rx_buffer_.clear();
    }
  }

  // ---------------------------------------------------------
  // ZTE polling loop — dynamically poll all devices from YAML
  // ---------------------------------------------------------
  static uint32_t last_zte_poll = 0;

  if (millis() - last_zte_poll > 1000) {
    last_zte_poll = millis();

    for (auto *device : this->devices_) {
      ESP_LOGD("zte_modbus", "Polling ZTE device address: 0x%02X", device->address_);
      send_zte_request(device->address_);
    }
  }
}

// ---------------------------------------------------------------------------
// Utility functions (CRC + ASCII helpers)
// ---------------------------------------------------------------------------

uint16_t chksum(const uint8_t data[], const uint16_t len) {
  uint16_t checksum = 0x00;
  for (uint16_t i = 0; i < len; i++) {
    checksum += data[i];
  }
  checksum = ~checksum;
  checksum += 1;
  return checksum;
}

uint16_t lchksum(const uint16_t len) {
  uint16_t lchecksum = 0x0000;

  if (len == 0)
    return 0x0000;

  lchecksum = (len & 0xf) + ((len >> 4) & 0xf) + ((len >> 8) & 0xf);
  lchecksum = ~(lchecksum % 16) + 1;

  return (lchecksum << 12) + len;
}

uint8_t ascii_hex_to_byte(char a, char b) {
  a = (a <= '9') ? a - '0' : (a & 0x7) + 9;
  b = (b <= '9') ? b - '0' : (b & 0x7) + 9;

  return (a << 4) + b;
}

static char byte_to_ascii_hex_digit(uint8_t v) {
  return v >= 10 ? 'A' + (v - 10) : '0' + v;
}

std::string byte_to_ascii_hex(const uint8_t *data, size_t length) {
  if (length == 0)
    return "";
  std::string ret;
  ret.resize(2 * length);

  for (size_t i = 0; i < length; i++) {
    ret[2 * i]     = byte_to_ascii_hex_digit((data[i] & 0xF0) >> 4);
    ret[2 * i + 1] = byte_to_ascii_hex_digit(data[i] & 0x0F);
  }
  return ret;
}

// ---------------------------------------------------------------------------
// Frame Parsing (Seplos + ZTE)
// ---------------------------------------------------------------------------

bool SeplosModbus::parse_seplos_modbus_byte_(uint8_t byte) {
  size_t at = this->rx_buffer_.size();
  this->rx_buffer_.push_back(byte);
  const uint8_t *raw = &this->rx_buffer_[0];

  // Start of frame = '~'
  if (at == 0) {
    if (raw[0] != 0x7E) {
      ESP_LOGW(TAG, "Invalid header: 0x%02X", raw[0]);
      return false;
    }
    return true;
  }

  // Wait for end-of-frame = '\r'
  if (raw[at] != 0x0D)
    return true;

  if (at > MAX_RESPONSE_SIZE) {
    ESP_LOGW(TAG, "Maximum response size exceeded. Flushing RX buffer...");
    return false;
  }

  uint16_t data_len = at - 4 - 1;

  uint16_t computed_crc =
      chksum(raw + 1, data_len);

  uint16_t remote_crc =
      (uint16_t(ascii_hex_to_byte(raw[at - 4], raw[at - 3])) << 8) |
      (uint16_t(ascii_hex_to_byte(raw[at - 2], raw[at - 1])) << 0);

  // ZTE frames do NOT use Seplos CRC — accept them anyway
  if (computed_crc != remote_crc) {
    if (raw[1] == '2' && raw[2] == '1') {
      ESP_LOGW(TAG, "CRC mismatch but accepting as ZTE FB101/FB100C1 frame");
    } else {
      ESP_LOGW(TAG, "CRC check failed! 0x%04X != 0x%04X", computed_crc, remote_crc);
      return false;
    }
  }

  // Convert ASCII hex -> bytes
  std::vector<uint8_t> data;
  for (uint16_t i = 1; i < data_len; i += 2) {
    data.push_back(ascii_hex_to_byte(raw[i], raw[i + 1]));
  }

  if (data.size() < 2)
    return false;

  uint8_t address = data[1];

  bool delivered = false;

  for (auto *device : this->devices_) {
    if (device->address_ == address) {
      device->on_seplos_modbus_data(data);
      delivered = true;
    }
  }

  return false;
}

void SeplosModbus::dump_config() {
  ESP_LOGCONFIG(TAG, "SeplosModbus:");
  LOG_PIN("  Flow Control Pin: ", this->flow_control_pin_);
  ESP_LOGCONFIG(TAG, "  RX timeout: %d ms", this->rx_timeout_);
}

float SeplosModbus::get_setup_priority() const {
  return setup_priority::BUS - 1.0f;
}

// ---------------------------------------------------------------------------
// Seplos send()
// ---------------------------------------------------------------------------

void SeplosModbus::send(uint8_t protocol_version, uint8_t address, uint8_t function, uint8_t value) {

  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(true);

  const uint16_t lenid = lchksum(1 * 2);

  std::vector<uint8_t> data;
  data.push_back(protocol_version);
  data.push_back(address);
  data.push_back(0x46);
  data.push_back(function);
  data.push_back(lenid >> 8);
  data.push_back(lenid >> 0);
  data.push_back(value);

  const uint16_t frame_len = data.size();
  std::string payload = "~";
  payload.append(byte_to_ascii_hex(data.data(), frame_len));

  uint16_t crc = chksum((const uint8_t *) payload.data() + 1, payload.size() - 1);

  data.push_back(crc >> 8);
  data.push_back(crc >> 0);

  payload.append(byte_to_ascii_hex(data.data() + frame_len, data.size() - frame_len));
  payload.append("\r");

  ESP_LOGD(TAG, "Send frame: %s", payload.c_str());

  this->write_str(payload.c_str());
  this->flush();

  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(false);
}

// ---------------------------------------------------------------------------
//                 ZTE FB101 / FB100C1 EXTENDED SUPPORT
// ---------------------------------------------------------------------------

// ZTE: checksum = two's complement of sum
uint16_t zte_checksum(const uint8_t *data, size_t len) {
  uint32_t sum = 0;
  for (size_t i = 0; i < len; i++)
    sum += data[i];
  return (~sum + 1) & 0xFFFF;
}

// Build and send ZTE request frame: ~21AA4642E002AA FDxx\r
void SeplosModbus::send_zte_request(uint8_t address) {
  uint8_t frame[16];
  size_t len = 0;

  frame[len++] = 0x21;      // Protocol
  frame[len++] = address;   // Address
  frame[len++] = 0x46;      // CID1
  frame[len++] = 0x42;      // CID2
  frame[len++] = 0xE0;      // LENCHK
  frame[len++] = 0x02;      // LENGTH
  frame[len++] = address;   // VALUE (same as seen in logs)

  uint16_t chk = zte_checksum(frame, len);

  std::string out = "~";
  out += byte_to_ascii_hex(frame, len);

  uint8_t chk_bytes[2] = {
      uint8_t(chk >> 8),
      uint8_t(chk & 0xFF),
  };

  out += byte_to_ascii_hex(chk_bytes, 2);
  out += "\r";

  ESP_LOGD("zte_modbus", "Send ZTE frame: %s", out.c_str());

  this->write_str(out.c_str());
  this->flush();
}

}  // namespace seplos_modbus
}  // namespace esphome
