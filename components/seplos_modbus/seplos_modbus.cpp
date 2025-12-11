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

  if (now - this->last_seplos_modbus_byte_ > this->rx_timeout_) {
    if (!this->rx_buffer_.empty()) {
      ESP_LOGVV(TAG, "Buffer cleared due to timeout: %s",
                format_hex_pretty(&this->rx_buffer_.front(), this->rx_buffer_.size()).c_str());
      this->rx_buffer_.clear();
    }
    this->last_seplos_modbus_byte_ = now;
  }

  while (this->available()) {
    uint8_t byte;
    this->read_byte(&byte);
    if (this->parse_seplos_modbus_byte_(byte)) {
      this->last_seplos_modbus_byte_ = now;
    } else {
      if (!this->rx_buffer_.empty()) {
        ESP_LOGVV(TAG, "Buffer cleared due to reset: %s",
                  format_hex_pretty(&this->rx_buffer_.front(), this->rx_buffer_.size()).c_str());
        this->rx_buffer_.clear();
      }
    }
  }
}

/**
 * Existing checksum function (kept for non-ZTE frames).
 * This computes two's-complement of the sum of bytes in 'data' (i.e. ~sum + 1).
 */
uint16_t chksum(const uint8_t data[], const uint16_t len) {
  uint32_t checksum = 0;
  for (uint16_t i = 0; i < len; i++) {
    checksum += data[i];
  }
  checksum = ~checksum;
  checksum += 1;
  return static_cast<uint16_t>(checksum & 0xFFFF);
}

/**
 * ZTE-specific checksum:
 * Compute sum of ASCII bytes from payload (not the interpreted hex bytes),
 * then return (0x10000 - (sum & 0xFFFF)) & 0xFFFF.
 *
 * This is equivalent to two's-complement negation of the 16-bit sum.
 */
uint16_t zte_ascii_crc(const uint8_t data[], const uint16_t len) {
  uint32_t sum = 0;
  for (uint16_t i = 0; i < len; i++) {
    sum += data[i];
  }
  uint16_t sum16 = static_cast<uint16_t>(sum & 0xFFFF);
  return static_cast<uint16_t>((0x10000 - sum16) & 0xFFFF);
}

/**
 * length checksum helper used when sending frames (unchanged).
 */
uint16_t lchksum(const uint16_t len) {
  uint16_t lchecksum = 0x0000;

  if (len == 0)
    return 0x0000;

  lchecksum = (len & 0xf) + ((len >> 4) & 0xf) + ((len >> 8) & 0xf);
  lchecksum = ~(lchecksum % 16) + 1;

  return (lchecksum << 12) + len;  // 4 nibble checksum + 12 bits length
}

uint8_t ascii_hex_to_byte(char a, char b) {
  a = (a <= '9') ? a - '0' : (a & 0x7) + 9;
  b = (b <= '9') ? b - '0' : (b & 0x7) + 9;

  return (a << 4) + b;
}

static char byte_to_ascii_hex(uint8_t v) { return v >= 10 ? 'A' + (v - 10) : '0' + v; }
std::string byte_to_ascii_hex(const uint8_t *data, size_t length) {
  if (length == 0)
    return "";
  std::string ret;
  ret.resize(2 * length);
  for (size_t i = 0; i < length; i++) {
    ret[2 * i] = byte_to_ascii_hex((data[i] & 0xF0) >> 4);
    ret[2 * i + 1] = byte_to_ascii_hex(data[i] & 0x0F);
  }
  return ret;
}

bool SeplosModbus::parse_seplos_modbus_byte_(uint8_t byte) {
  size_t at = this->rx_buffer_.size();
  this->rx_buffer_.push_back(byte);
  const uint8_t *raw = &this->rx_buffer_[0];

  // Start of frame
  if (at == 0) {
    if (raw[0] != 0x7E) {
      ESP_LOGW(TAG, "Invalid header: 0x%02X", raw[0]);
      // return false to reset buffer
      return false;
    }
    return true;
  }

  // End of frame '\r'
  if (raw[at] != 0x0D)
    return true;

  if (at > MAX_RESPONSE_SIZE) {
    ESP_LOGW(TAG, "Maximum response size exceeded. Flushing RX buffer...");
    return false;
  }

  // data_len = number of ASCII payload bytes (from raw[1] .. raw[1 + data_len - 1])
  uint16_t data_len = static_cast<uint16_t>(at - 4 - 1);

  // Quick sanity checks: payload length must be >= 2 (VER+ADDR...) and even (pairs of hex)
  if (data_len < 2 || (data_len % 2) != 0) {
    ESP_LOGW(TAG, "Invalid payload length (%u). Attempting resync...", data_len);
    // Attempt to resync: find next '~' in buffer (skip current leading ~)
    for (size_t i = 1; i < this->rx_buffer_.size(); ++i) {
      if (this->rx_buffer_[i] == 0x7E) {
        // remove bytes up to the next '~'
        this->rx_buffer_.erase(this->rx_buffer_.begin(), this->rx_buffer_.begin() + i);
        return true; // continue parsing with trimmed buffer
      }
    }
    // no next '~' found -> clear buffer
    return false;
  }

  // Validate payload are ALL ASCII hex chars (0-9 A-F a-f)
  bool valid_payload = true;
  for (uint16_t i = 1; i <= data_len; ++i) {
    uint8_t ch = raw[i];
    bool is_hex = (ch >= '0' && ch <= '9') ||
                  (ch >= 'A' && ch <= 'F') ||
                  (ch >= 'a' && ch <= 'f');
    if (!is_hex) {
      valid_payload = false;
      break;
    }
  }

  if (!valid_payload) {
    ESP_LOGW(TAG, "Non-hex char detected in payload. Attempting resync...");
    // find the next '~' and shift buffer to it
    for (size_t i = 1; i < this->rx_buffer_.size(); ++i) {
      if (this->rx_buffer_[i] == 0x7E) {
        this->rx_buffer_.erase(this->rx_buffer_.begin(), this->rx_buffer_.begin() + i);
        return true; // continue parsing
      }
    }
    // no next '~' -> clear buffer
    return false;
  }

  // Remote CRC is encoded as 4 ASCII hex chars just before CR:
  // positions (at-4, at-3, at-2, at-1)
  uint16_t remote_crc = uint16_t(ascii_hex_to_byte(raw[at - 4], raw[at - 3])) << 8 |
                        (uint16_t(ascii_hex_to_byte(raw[at - 2], raw[at - 1])) << 0);

  // Detect ZTE frames (they typically start with "~21...")
  bool is_zte = false;
  if (data_len >= 2) {
    // raw[1] and raw[2] are ASCII chars
    if (raw[1] == '2' && raw[2] == '1') {
      is_zte = true;
    }
  }

  uint16_t computed_crc = 0;
  if (is_zte) {
    // Compute ASCII-sum CRC over payload (raw[1] .. raw[1+data_len-1])
    computed_crc = zte_ascii_crc(raw + 1, data_len);
    ESP_LOGVV(TAG, "ZTE frame detected. ASCII-sum CRC computed=0x%04X remote=0x%04X", computed_crc, remote_crc);
  } else {
    // Use legacy chksum (bytes interpreted as already-converted binary)
    computed_crc = chksum(raw + 1, data_len);
    ESP_LOGVV(TAG, "Non-ZTE frame. generic chksum computed=0x%04X remote=0x%04X", computed_crc, remote_crc);
  }

  if (computed_crc != remote_crc) {
    // Dump a short hexdump for debugging (first 80 bytes max)
    size_t dump_len = this->rx_buffer_.size() < 80 ? this->rx_buffer_.size() : 80;
    ESP_LOGW(TAG, "CRC check failed! computed=0x%04X remote=0x%04X  raw (hex, first %u bytes): %s",
             computed_crc, remote_crc, static_cast<unsigned>(dump_len),
             format_hex_pretty(&this->rx_buffer_.front(), dump_len).c_str());
    // Attempt resync to next '~' because data might be corrupted
    for (size_t i = 1; i < this->rx_buffer_.size(); ++i) {
      if (this->rx_buffer_[i] == 0x7E) {
        this->rx_buffer_.erase(this->rx_buffer_.begin(), this->rx_buffer_.begin() + i);
        return true;
      }
    }
    return false;
  }

  // Convert ASCII hex payload to binary bytes (payload starts at raw[1], length = data_len)
  std::vector<uint8_t> data;
  data.reserve(data_len / 2);
  for (uint16_t i = 1; i < data_len; i = i + 2) {
    data.push_back(ascii_hex_to_byte(raw[i], raw[i + 1]));
  }

  // NOTE: In the data vector, position mapping assumed by original code:
  // data[0] = VER, data[1] = ADDR, data[2] = CID1, data[3] = CID2, ...
  uint8_t address = 0xFF;
  if (data.size() > 1) {
    address = data[1];
  }

  bool found = false;
  for (auto *device : this->devices_) {
    if (device->address_ == address) {
      device->on_seplos_modbus_data(data);
      found = true;
    }
  }

  if (!found) {
    // Unknown address — keep quiet (original code commented out warning)
    // ESP_LOGW(TAG, "Got SeplosModbus frame from unknown address 0x%02X! ", address);
  }

  // return false to reset buffer (frame consumed)
  return false;
}

void SeplosModbus::dump_config() {
  ESP_LOGCONFIG(TAG, "SeplosModbus:");
  LOG_PIN("  Flow Control Pin: ", this->flow_control_pin_);
  ESP_LOGCONFIG(TAG, "  RX timeout: %d ms", this->rx_timeout_);
}
float SeplosModbus::get_setup_priority() const {
  // After UART bus
  return setup_priority::BUS - 1.0f;
}

void SeplosModbus::send(uint8_t protocol_version, uint8_t address, uint8_t function, uint8_t value) {
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(true);

  const uint16_t lenid = lchksum(1 * 2);
  std::vector<uint8_t> data;
  data.push_back(protocol_version);  // VER
  data.push_back(address);           // ADDR
  data.push_back(0x46);              // CID1
  data.push_back(function);          // CID2 (0x42)
  data.push_back(lenid >> 8);        // LCHKSUM (high byte)
  data.push_back(lenid >> 0);        // LENGTH (low byte)
  data.push_back(value);             // VALUE

  const uint16_t frame_len = data.size();
  std::string payload = "~";  // SOF (0x7E)
  payload.append(byte_to_ascii_hex(data.data(), frame_len));

  // For sending keep using legacy chksum which is two's-complement of sum of bytes
  uint16_t crc = chksum((const uint8_t *) payload.data() + 1, static_cast<uint16_t>(payload.size() - 1));
  data.push_back(static_cast<uint8_t>(crc >> 8));  // CHKSUM high byte
  data.push_back(static_cast<uint8_t>(crc & 0xFF));  // CHKSUM low byte

  payload.append(byte_to_ascii_hex(data.data() + frame_len, data.size() - frame_len));  // Append checksum
  payload.append("\r");                                                                 // EOF (0x0D)

  ESP_LOGD(TAG, "Send frame: %s", payload.c_str());

  this->write_str(payload.c_str());
  this->flush();

  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(false);
}

}  // namespace seplos_modbus
}  // namespace esphome
