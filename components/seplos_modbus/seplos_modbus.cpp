#include "seplos_modbus.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace seplos_modbus {

static const char *const TAG = "seplos_modbus";

static const uint16_t MAX_RESPONSE_SIZE = 340;

// =============================
// TRACK LAST REQUEST (REPLY FILTER)
// =============================
uint8_t expected_addr_  = 0xFF;
uint8_t expected_cid1_  = 0xFF;
uint8_t expected_cid2_  = 0xFF;

void SeplosModbus::setup() {
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->setup();
}

void SeplosModbus::loop() {
  const uint32_t now = millis();

  if (now - this->last_seplos_modbus_byte_ > this->rx_timeout_) {
    this->rx_buffer_.clear();
    this->last_seplos_modbus_byte_ = now;
  }

  while (this->available()) {
    uint8_t byte;
    this->read_byte(&byte);
    if (!this->parse_seplos_modbus_byte_(byte))
      this->rx_buffer_.clear();
    else
      this->last_seplos_modbus_byte_ = now;
  }
}

// CHKSUM — legacy checksum used for outgoing frames
uint16_t chksum(const uint8_t data[], const uint16_t len) {
  uint32_t checksum = 0;
  for (uint16_t i = 0; i < len; i++)
    checksum += data[i];
  checksum = ~checksum + 1;
  return uint16_t(checksum & 0xFFFF);
}

// ZTE ASCII SUM CRC
uint16_t zte_ascii_crc(const uint8_t *data, uint16_t len) {
  uint32_t sum = 0;
  for (uint16_t i = 0; i < len; i++)
    sum += data[i];
  return uint16_t((0x10000 - (sum & 0xFFFF)) & 0xFFFF);
}

uint8_t ascii_hex_to_byte(char a, char b) {
  a = (a <= '9') ? a - '0' : (a & 0x7) + 9;
  b = (b <= '9') ? b - '0' : (b & 0x7) + 9;
  return (a << 4) | b;
}

bool SeplosModbus::parse_seplos_modbus_byte_(uint8_t byte) {

  size_t at = this->rx_buffer_.size();
  this->rx_buffer_.push_back(byte);
  const uint8_t *raw = &this->rx_buffer_[0];

  // Start
  if (at == 0) {
    if (raw[0] != 0x7E)
      return false;
    return true;
  }

  // Not end yet
  if (raw[at] != 0x0D)
    return true;

  // Too long
  if (at > MAX_RESPONSE_SIZE)
    return false;

  uint16_t data_len = at - 5;

  // Clean ASCII hex payload
  std::string cleaned;
  for (uint16_t i = 1; i <= data_len; i++) {
    char c = raw[i];
    if ((c >= '0' && c <= '9') ||
        (c >= 'A' && c <= 'F') ||
        (c >= 'a' && c <= 'f'))
      cleaned.push_back(c);
  }

  if (cleaned.size() < 4 || cleaned.size() % 2 != 0)
    return false;

  // Decode remote CRC
  uint16_t remote_crc =
      (ascii_hex_to_byte(raw[at - 4], raw[at - 3]) << 8) |
      (ascii_hex_to_byte(raw[at - 2], raw[at - 1]) << 0);

  // Detect ZTE frame (starts with "21")
  bool is_zte = cleaned[0] == '2' && cleaned[1] == '1';

  uint16_t computed_crc = 0;
  if (is_zte) {
    computed_crc = zte_ascii_crc(reinterpret_cast<const uint8_t *>(cleaned.data()),
                                 cleaned.size());
  } else {
    std::vector<uint8_t> binary;
    for (int i = 0; i < cleaned.size(); i += 2)
      binary.push_back(ascii_hex_to_byte(cleaned[i], cleaned[i + 1]));
    computed_crc = chksum(binary.data(), binary.size());
  }

  if (computed_crc != remote_crc)
    return false;

  // Convert cleaned ASCII to binary
  std::vector<uint8_t> data;
  for (int i = 0; i < cleaned.size(); i += 2)
    data.push_back(ascii_hex_to_byte(cleaned[i], cleaned[i + 1]));

  // Extract ADDR + CID1 + CID2
  if (data.size() < 4)
    return false;

  uint8_t addr = data[1];
  uint8_t cid1 = data[2];
  uint8_t cid2 = data[3];

  // =============================
  //   REPLY FILTER CHECK HERE!
  // =============================
  if (addr != expected_addr_ ||
      cid1 != expected_cid1_ ||
      cid2 != expected_cid2_) {
    ESP_LOGV(TAG,
      "Ignoring frame: ADDR=0x%02X CID1=0x%02X CID2=0x%02X (waiting reply for ADDR=0x%02X CID1=0x%02X CID2=0x%02X)",
      addr, cid1, cid2,
      expected_addr_, expected_cid1_, expected_cid2_);
    return false;
  }

  // Dispatch to matching device
  for (auto *device : this->devices_) {
    if (device->address_ == addr)
      device->on_seplos_modbus_data(data);
  }

  return false;
}

void SeplosModbus::send(uint8_t protocol_version, uint8_t address, uint8_t function, uint8_t value) {

  // Save request signature for reply filter
  expected_addr_ = address;
  expected_cid1_ = 0x46;
  expected_cid2_ = function;

  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(true);

  const uint16_t lenid = 0xE002;
  std::vector<uint8_t> data;

  data.push_back(protocol_version);
  data.push_back(address);
  data.push_back(0x46);
  data.push_back(function);
  data.push_back(lenid >> 8);
  data.push_back(lenid & 0xFF);
  data.push_back(value);

  std::string payload = "~";
  for (auto b : data)
    payload.push_back("0123456789ABCDEF"[b >> 4]),
    payload.push_back("0123456789ABCDEF"[b & 0x0F]);

  uint16_t crc = chksum((const uint8_t *) payload.c_str() + 1, payload.size() - 1);
  payload.push_back("0123456789ABCDEF"[crc >> 12 & 0xF]);
  payload.push_back("0123456789ABCDEF"[crc >> 8 & 0xF]);
  payload.push_back("0123456789ABCDEF"[crc >> 4 & 0xF]);
  payload.push_back("0123456789ABCDEF"[crc & 0xF]);
  payload.push_back('\r');

  ESP_LOGD(TAG, "SEND → %s", payload.c_str());

  this->write_str(payload.c_str());
  this->flush();

  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(false);
}

} // namespace seplos_modbus
} // namespace esphome
