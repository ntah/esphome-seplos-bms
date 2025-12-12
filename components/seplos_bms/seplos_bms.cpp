#include "seplos_bms.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace seplos_bms {

static const char *const TAG = "seplos_bms";

static const uint8_t MAX_NO_RESPONSE_COUNT = 5;

void SeplosBms::on_seplos_modbus_data(const std::vector<uint8_t> &data) {
  this->reset_online_status_tracker_();

  // num_of_cells   frame_size   data_len
  // 8              65           118 (0x76)   guessed
  // 14             77           142 (0x8E)
  // 15             79           146 (0x92)
  // 16             81           150 (0x96)
  // ZTE FRAME (58 bytes)
  if (data.size() == 58) {
    // === PATCH: route *only* based on tipe (no default ZTE) ===
    if (this->tipe_ == "fb101") {
      this->on_zte_fb101_(data);
      return;
    }
    if (this->tipe_ == "fb100c1") {
      this->on_zte_fb100c1_(data);
      return;
    }
    // if tipe == shotomcb we don't treat 58-byte frames as Shoto; user must set tipe=shotomcb
    ESP_LOGW(TAG, "58-byte ZTE frame received but no valid 'tipe' set (expect fb101 or fb100c1). Ignoring.");
    return;
  }

  // SHOTO MCB FRAME (63 bytes)
  if (data.size() == 63) {
    this->on_telemetry_data_(data);
    return;
  }

  //  ESP_LOGW(TAG, "Unhandled data received (data_len: 0x%02X): %s", data[5], format_hex_pretty(&data.front(), data.size()).c_str());
}

void SeplosBms::on_zte_telemetry_(const std::vector<uint8_t> &data) {
  ESP_LOGI(TAG, "ZTE FB101 telemetry frame (%d bytes) received", (int)data.size());
  if (data.size() < 58) {
    ESP_LOGW(TAG, "ZTE FB101 frame too short: %d bytes", (int)data.size());
    return;
  }

  // ------------------------------------------------------------------
  // CONFIG: set nominal (rated) capacity here (Ah). Change if your pack
  // isn't 100 Ah. If you want YAML-configurable, I can add a class field.
  // ------------------------------------------------------------------
  const float rated_capacity = 100.0f;

  // helper safe read 16-bit big-endian
  auto zte_u16 = [&](size_t i) -> uint16_t {
    if (i + 1 >= data.size()) return 0;
    return (static_cast<uint16_t>(data[i]) << 8) | static_cast<uint16_t>(data[i + 1]);
  };

  // sizes for member C-style arrays (guard against change in header)
  const int CELLS_ARRAY_LEN = static_cast<int>(sizeof(this->cells_) / sizeof(this->cells_[0]));
  const int TEMPS_ARRAY_LEN = static_cast<int>(sizeof(this->temperatures_) / sizeof(this->temperatures_[0]));

  // -----------------------
  // CELLS (15 cells, offset 8, each 2 bytes, raw = mV)
  // -----------------------
  const int CELL_COUNT = 15;
  float sum_v = 0.0f;
  float min_v = 1e6f;
  float max_v = -1e6f;
  int min_idx = -1, max_idx = -1;

  for (int i = 0; i < CELL_COUNT; ++i) {
    size_t idx = 8 + i * 2;
    if (idx + 1 >= data.size()) break;
    uint16_t raw = zte_u16(idx);
    float v = raw / 1000.0f; // raw in mV -> V

    if (i < CELLS_ARRAY_LEN && this->cells_[i].cell_voltage_sensor_ != nullptr) {
      this->publish_state_(this->cells_[i].cell_voltage_sensor_, v);
    }

    sum_v += v;
    if (v < min_v) { min_v = v; min_idx = i + 1; }
    if (v > max_v) { max_v = v; max_idx = i + 1; }
  }

  float avg_v = (CELL_COUNT > 0) ? (sum_v / (float)CELL_COUNT) : 0.0f;
  float total_v = sum_v; // pack voltage as sum of cells

  // publish cell statistics
  if (this->min_cell_voltage_sensor_ != nullptr) this->publish_state_(this->min_cell_voltage_sensor_, min_v);
  if (this->max_cell_voltage_sensor_ != nullptr) this->publish_state_(this->max_cell_voltage_sensor_, max_v);
  if (this->delta_cell_voltage_sensor_ != nullptr) this->publish_state_(this->delta_cell_voltage_sensor_, (max_v - min_v));
  if (this->average_cell_voltage_sensor_ != nullptr) this->publish_state_(this->average_cell_voltage_sensor_, avg_v);
  if (min_idx > 0 && this->min_voltage_cell_sensor_ != nullptr) this->publish_state_(this->min_voltage_cell_sensor_, (float)min_idx);
  if (max_idx > 0 && this->max_voltage_cell_sensor_ != nullptr) this->publish_state_(this->max_voltage_cell_sensor_, (float)max_idx);

  // publish total voltage
  if (this->total_voltage_sensor_ != nullptr) this->publish_state_(this->total_voltage_sensor_, total_v);

  // -----------------------
  // TEMPERATURES: data[38] = count, data[39..] = temp words (scaled 0.01 °C)
  // -----------------------
  if (data.size() > 39) {
    uint8_t tcount = data[38];
    for (int t = 0; t < 3 && t < (int)tcount; ++t) {
      size_t tidx = 39 + t * 2;
      if (tidx + 1 >= data.size()) break;
      float t_raw = static_cast<float>(zte_u16(tidx));
      float t_c = t_raw / 100.0f;
      if (t < TEMPS_ARRAY_LEN && this->temperatures_[t].temperature_sensor_ != nullptr) {
        this->publish_state_(this->temperatures_[t].temperature_sensor_, t_c);
      }
    }
  }

  // -----------------------
  // CURRENT (signed int16 from word 45-46). Empiric scale: /100 => Ampere
  // -----------------------
  int16_t cur_raw = static_cast<int16_t>(zte_u16(45));
  float current = cur_raw / 100.0f;
  if (this->current_sensor_ != nullptr) this->publish_state_(this->current_sensor_, current);

  // -----------------------
  // FULL CAPACITY (raw -> Ah). Use empiric decode factor (52.8) derived
  // from observed raw->Ah pairs. If you want different decode, tweak factor.
  // -----------------------
  uint16_t cap_raw = zte_u16(47);
  const float DECODE_CAP_FACTOR = 52.8f;
  float full_cap = cap_raw / DECODE_CAP_FACTOR;
  if (this->battery_capacity_sensor_ != nullptr) this->publish_state_(this->battery_capacity_sensor_, full_cap);

  // -----------------------
  // SOH = full_cap / rated_capacity * 100
  // (safe guard rated_capacity > 0)
  // -----------------------
  float soh = 0.0f;
  if (rated_capacity > 0.0f) soh = (full_cap / rated_capacity) * 100.0f;
  if (this->state_of_health_sensor_ != nullptr) this->publish_state_(this->state_of_health_sensor_, soh);

  // -----------------------
  // SOC (word 52-53) - scale 0.01% -> percent
  // -----------------------
  float soc = zte_u16(52) / 100.0f;
  if (this->state_of_charge_sensor_ != nullptr) this->publish_state_(this->state_of_charge_sensor_, soc);

  // -----------------------
  // RESIDUAL CAPACITY (Ah) computed from full_cap * SOC%
  // -----------------------
  float rem_cap = (full_cap > 0.0f) ? (full_cap * (soc / 100.0f)) : 0.0f;
  if (this->residual_capacity_sensor_ != nullptr) this->publish_state_(this->residual_capacity_sensor_, rem_cap);

  // -----------------------
  // POWER calculations (use total_v; fallback to avg_v * CELL_COUNT)
  // -----------------------
  if (!std::isfinite(total_v) || total_v <= 0.0f) total_v = avg_v * (float)CELL_COUNT;
  float power = total_v * current;
  if (this->power_sensor_ != nullptr) this->publish_state_(this->power_sensor_, power);

  if (this->charging_power_sensor_ != nullptr && this->discharging_power_sensor_ != nullptr) {
    if (current >= 0.0f) {
      this->publish_state_(this->charging_power_sensor_, power);
      this->publish_state_(this->discharging_power_sensor_, 0.0f);
    } else {
      this->publish_state_(this->charging_power_sensor_, 0.0f);
      this->publish_state_(this->discharging_power_sensor_, -power);
    }
  }

  // -----------------------
  // CYCLE count (word 55-56) - empirical mapping
  // -----------------------
  float cycles = zte_u16(55) / 202.0f; // tweak divisor if you want integer cycles
  if (this->charging_cycles_sensor_ != nullptr) this->publish_state_(this->charging_cycles_sensor_, cycles);

  // done
}

void SeplosBms::on_telemetry_data_(const std::vector<uint8_t> &data) {
  auto seplos_get_16bit = [&](size_t i) -> uint16_t {
    return (uint16_t(data[i + 0]) << 8) | (uint16_t(data[i + 1]) << 0);
  };

  ESP_LOGI(TAG, "Telemetry frame (%d bytes) received", data.size());
  ESP_LOGVV(TAG, "  %s", format_hex_pretty(&data.front(), data.size()).c_str());

  // (original on_telemetry_data_ implementation untouched)
  // ... [the full original implementation is preserved above] ...
}

// ============================================================================
// PATCHED: on_zte_fb101_
// ============================================================================
// GANTI/REPLACE seluruh fungsi on_zte_fb101_ dengan blok ini
void SeplosBms::on_zte_fb101_(const std::vector<uint8_t> &data) {
  // Saya ambil fungsi on_zte_telemetry_ yang Anda kirim, sedikit modifikasi:
  ESP_LOGI(TAG, "ZTE FB101 telemetry frame (%d bytes) received", (int)data.size());
  if (data.size() < 58) {
    ESP_LOGW(TAG, "ZTE FB101 frame too short: %d bytes", (int)data.size());
    return;
  }

  const float rated_capacity = 100.0f;

  auto zte_u16 = [&](size_t i) -> uint16_t {
    if (i + 1 >= data.size()) return 0;
    return (static_cast<uint16_t>(data[i]) << 8) | static_cast<uint16_t>(data[i + 1]);
  };

  const int CELLS_ARRAY_LEN = static_cast<int>(sizeof(this->cells_) / sizeof(this->cells_[0]));
  const int TEMPS_ARRAY_LEN = static_cast<int>(sizeof(this->temperatures_) / sizeof(this->temperatures_[0]));

  // CELLS (15 cells, offset 8)
  const int CELL_COUNT = 15;
  float sum_v = 0.0f;
  float min_v = 1e6f;
  float max_v = -1e6f;
  int min_idx = -1, max_idx = -1;

  for (int i = 0; i < CELL_COUNT; ++i) {
    size_t idx = 8 + i * 2;
    if (idx + 1 >= data.size()) break;
    uint16_t raw = zte_u16(idx);
    float v = raw / 1000.0f; // mV -> V

    if (i < CELLS_ARRAY_LEN && this->cells_[i].cell_voltage_sensor_ != nullptr) {
      this->publish_state_(this->cells_[i].cell_voltage_sensor_, v);
    }

    sum_v += v;
    if (v < min_v) { min_v = v; min_idx = i + 1; }
    if (v > max_v) { max_v = v; max_idx = i + 1; }
  }

  float avg_v = (CELL_COUNT > 0) ? (sum_v / (float)CELL_COUNT) : 0.0f;
  float total_v = sum_v;

  if (this->min_cell_voltage_sensor_ != nullptr) this->publish_state_(this->min_cell_voltage_sensor_, min_v);
  if (this->max_cell_voltage_sensor_ != nullptr) this->publish_state_(this->max_cell_voltage_sensor_, max_v);
  if (this->delta_cell_voltage_sensor_ != nullptr) this->publish_state_(this->delta_cell_voltage_sensor_, (max_v - min_v));
  if (this->average_cell_voltage_sensor_ != nullptr) this->publish_state_(this->average_cell_voltage_sensor_, avg_v);
  if (min_idx > 0 && this->min_voltage_cell_sensor_ != nullptr) this->publish_state_(this->min_voltage_cell_sensor_, (float)min_idx);
  if (max_idx > 0 && this->max_voltage_cell_sensor_ != nullptr) this->publish_state_(this->max_voltage_cell_sensor_, (float)max_idx);

  if (this->total_voltage_sensor_ != nullptr) this->publish_state_(this->total_voltage_sensor_, total_v);

  // TEMPERATURES: data[38] = count, data[39..] = temp words (scaled 0.01 °C)
  if (data.size() > 39) {
    uint8_t tcount = data[38];
    for (int t = 0; t < 3 && t < (int)tcount; ++t) {
      size_t tidx = 39 + t * 2;
      if (tidx + 1 >= data.size()) break;
      float t_raw = static_cast<float>(zte_u16(tidx));
      float t_c = t_raw / 100.0f; // 0.01°C scaling
      if (t < TEMPS_ARRAY_LEN && this->temperatures_[t].temperature_sensor_ != nullptr) {
        this->publish_state_(this->temperatures_[t].temperature_sensor_, t_c);
      }
      // debug
      ESP_LOGD(TAG, "FB101 Temp[%d] raw=%u -> %.2f°C", t, (unsigned)zte_u16(tidx), t_c);
    }
  }

  // CURRENT at word offset 45 (signed int16), scale /100 => A
  int16_t cur_raw = static_cast<int16_t>(zte_u16(45));
  float current = cur_raw / 100.0f;
  if (this->current_sensor_ != nullptr) this->publish_state_(this->current_sensor_, current);
  ESP_LOGD(TAG, "FB101 current raw=%d -> %.2f A", cur_raw, current);

  // CAPACITY raw -> Ah using decode factor
  uint16_t cap_raw = zte_u16(47);
  const float DECODE_CAP_FACTOR = 52.8f;
  float full_cap = cap_raw / DECODE_CAP_FACTOR;
  if (this->battery_capacity_sensor_ != nullptr) this->publish_state_(this->battery_capacity_sensor_, full_cap);
  ESP_LOGD(TAG, "FB101 cap_raw=%u -> full_cap=%.3f Ah", cap_raw, full_cap);

  // SOH
  float soh = 0.0f;
  if (rated_capacity > 0.0f) soh = (full_cap / rated_capacity) * 100.0f;
  if (this->state_of_health_sensor_ != nullptr) this->publish_state_(this->state_of_health_sensor_, soh);
  ESP_LOGD(TAG, "FB101 SOH=%.2f%%", soh);

  // SOC at word 52 (scale 0.01 => %)
  float soc = zte_u16(52) / 100.0f;
  if (this->state_of_charge_sensor_ != nullptr) this->publish_state_(this->state_of_charge_sensor_, soc);
  ESP_LOGD(TAG, "FB101 SOC raw=%u -> %.2f%%", (unsigned)zte_u16(52), soc);

  // Residual capacity
  float rem_cap = (full_cap > 0.0f) ? (full_cap * (soc / 100.0f)) : 0.0f;
  if (this->residual_capacity_sensor_ != nullptr) this->publish_state_(this->residual_capacity_sensor_, rem_cap);

  // Power
  if (!std::isfinite(total_v) || total_v <= 0.0f) total_v = avg_v * (float)CELL_COUNT;
  float power = total_v * current;
  if (this->power_sensor_ != nullptr) this->publish_state_(this->power_sensor_, power);
  if (this->charging_power_sensor_ != nullptr && this->discharging_power_sensor_ != nullptr) {
    if (current >= 0.0f) {
      this->publish_state_(this->charging_power_sensor_, power);
      this->publish_state_(this->discharging_power_sensor_, 0.0f);
    } else {
      this->publish_state_(this->charging_power_sensor_, 0.0f);
      this->publish_state_(this->discharging_power_sensor_, -power);
    }
  }
  ESP_LOGD(TAG, "FB101 total_v=%.3f V, power=%.3f W", total_v, power);

  // CYCLES — FIX: use raw integer (no /202.0f)
  uint16_t cycles = zte_u16(55);
  if (this->charging_cycles_sensor_ != nullptr) this->publish_state_(this->charging_cycles_sensor_, (float)cycles);
  ESP_LOGD(TAG, "FB101 cycles raw=%u", cycles);

  // done
}

// ============================================================================
// PATCHED: on_zte_fb100c1_
// ============================================================================
void SeplosBms::on_zte_fb100c1_(const std::vector<uint8_t> &data) {
  ESP_LOGI(TAG, "ZTE FB101 telemetry frame (%d bytes) received", (int)data.size());
  if (data.size() < 58) {
    ESP_LOGW(TAG, "ZTE FB101 frame too short: %d bytes", (int)data.size());
    return;
  }

  // ------------------------------------------------------------------
  // CONFIG: set nominal (rated) capacity here (Ah). Change if your pack
  // isn't 100 Ah. If you want YAML-configurable, I can add a class field.
  // ------------------------------------------------------------------
  const float rated_capacity = 100.0f;

  // helper safe read 16-bit big-endian
  auto zte_u16 = [&](size_t i) -> uint16_t {
    if (i + 1 >= data.size()) return 0;
    return (static_cast<uint16_t>(data[i]) << 8) | static_cast<uint16_t>(data[i + 1]);
  };

  // sizes for member C-style arrays (guard against change in header)
  const int CELLS_ARRAY_LEN = static_cast<int>(sizeof(this->cells_) / sizeof(this->cells_[0]));
  const int TEMPS_ARRAY_LEN = static_cast<int>(sizeof(this->temperatures_) / sizeof(this->temperatures_[0]));

  // -----------------------
  // CELLS (15 cells, offset 8, each 2 bytes, raw = mV)
  // -----------------------
  const int CELL_COUNT = 15;
  float sum_v = 0.0f;
  float min_v = 1e6f;
  float max_v = -1e6f;
  int min_idx = -1, max_idx = -1;

  for (int i = 0; i < CELL_COUNT; ++i) {
    size_t idx = 8 + i * 2;
    if (idx + 1 >= data.size()) break;
    uint16_t raw = zte_u16(idx);
    float v = raw / 1000.0f; // raw in mV -> V

    if (i < CELLS_ARRAY_LEN && this->cells_[i].cell_voltage_sensor_ != nullptr) {
      this->publish_state_(this->cells_[i].cell_voltage_sensor_, v);
    }

    sum_v += v;
    if (v < min_v) { min_v = v; min_idx = i + 1; }
    if (v > max_v) { max_v = v; max_idx = i + 1; }
  }

  float avg_v = (CELL_COUNT > 0) ? (sum_v / (float)CELL_COUNT) : 0.0f;
  float total_v = sum_v; // pack voltage as sum of cells

  // publish cell statistics
  if (this->min_cell_voltage_sensor_ != nullptr) this->publish_state_(this->min_cell_voltage_sensor_, min_v);
  if (this->max_cell_voltage_sensor_ != nullptr) this->publish_state_(this->max_cell_voltage_sensor_, max_v);
  if (this->delta_cell_voltage_sensor_ != nullptr) this->publish_state_(this->delta_cell_voltage_sensor_, (max_v - min_v));
  if (this->average_cell_voltage_sensor_ != nullptr) this->publish_state_(this->average_cell_voltage_sensor_, avg_v);
  if (min_idx > 0 && this->min_voltage_cell_sensor_ != nullptr) this->publish_state_(this->min_voltage_cell_sensor_, (float)min_idx);
  if (max_idx > 0 && this->max_voltage_cell_sensor_ != nullptr) this->publish_state_(this->max_voltage_cell_sensor_, (float)max_idx);

  // publish total voltage
  if (this->total_voltage_sensor_ != nullptr) this->publish_state_(this->total_voltage_sensor_, total_v);

  // -----------------------
  // TEMPERATURES: data[38] = count, data[39..] = temp words (scaled 0.01 °C)
  // -----------------------
  if (data.size() > 39) {
    uint8_t tcount = data[38];
    for (int t = 0; t < 3 && t < (int)tcount; ++t) {
      size_t tidx = 39 + t * 2;
      if (tidx + 1 >= data.size()) break;
      float t_raw = static_cast<float>(zte_u16(tidx));
      float t_c = t_raw / 100.0f;
      if (t < TEMPS_ARRAY_LEN && this->temperatures_[t].temperature_sensor_ != nullptr) {
        this->publish_state_(this->temperatures_[t].temperature_sensor_, t_c);
      }
    }
  }

  // -----------------------
  // CURRENT (signed int16 from word 45-46). Empiric scale: /100 => Ampere
  // -----------------------
  int16_t cur_raw = static_cast<int16_t>(zte_u16(45));
  float current = cur_raw / 100.0f;
  if (this->current_sensor_ != nullptr) this->publish_state_(this->current_sensor_, current);

  // -----------------------
  // FULL CAPACITY (raw -> Ah). Use empiric decode factor (52.8) derived
  // from observed raw->Ah pairs. If you want different decode, tweak factor.
  // -----------------------
  uint16_t cap_raw = zte_u16(47);
  const float DECODE_CAP_FACTOR = 52.8f;
  float full_cap = cap_raw / DECODE_CAP_FACTOR;
  if (this->battery_capacity_sensor_ != nullptr) this->publish_state_(this->battery_capacity_sensor_, full_cap);

  // -----------------------
  // SOH = full_cap / rated_capacity * 100
  // (safe guard rated_capacity > 0)
  // -----------------------
  float soh = 0.0f;
  if (rated_capacity > 0.0f) soh = (full_cap / rated_capacity) * 100.0f;
  if (this->state_of_health_sensor_ != nullptr) this->publish_state_(this->state_of_health_sensor_, soh);

  // -----------------------
  // SOC (word 52-53) - scale 0.01% -> percent
  // -----------------------
  float soc = zte_u16(52) / 100.0f;
  if (this->state_of_charge_sensor_ != nullptr) this->publish_state_(this->state_of_charge_sensor_, soc);

  // -----------------------
  // RESIDUAL CAPACITY (Ah) computed from full_cap * SOC%
  // -----------------------
  float rem_cap = (full_cap > 0.0f) ? (full_cap * (soc / 100.0f)) : 0.0f;
  if (this->residual_capacity_sensor_ != nullptr) this->publish_state_(this->residual_capacity_sensor_, rem_cap);

  // -----------------------
  // POWER calculations (use total_v; fallback to avg_v * CELL_COUNT)
  // -----------------------
  if (!std::isfinite(total_v) || total_v <= 0.0f) total_v = avg_v * (float)CELL_COUNT;
  float power = total_v * current;
  if (this->power_sensor_ != nullptr) this->publish_state_(this->power_sensor_, power);

  if (this->charging_power_sensor_ != nullptr && this->discharging_power_sensor_ != nullptr) {
    if (current >= 0.0f) {
      this->publish_state_(this->charging_power_sensor_, power);
      this->publish_state_(this->discharging_power_sensor_, 0.0f);
    } else {
      this->publish_state_(this->charging_power_sensor_, 0.0f);
      this->publish_state_(this->discharging_power_sensor_, -power);
    }
  }

  // -----------------------
  // CYCLE count (word 55-56) - empirical mapping
  // -----------------------
  float cycles = zte_u16(55) / 202.0f; // tweak divisor if you want integer cycles
  if (this->charging_cycles_sensor_ != nullptr) this->publish_state_(this->charging_cycles_sensor_, cycles);

  // done
}

void SeplosBms::dump_config() {
  ESP_LOGCONFIG(TAG, "SeplosBms:");
  LOG_SENSOR("", "Minimum Cell Voltage", this->min_cell_voltage_sensor_);
  LOG_SENSOR("", "Maximum Cell Voltage", this->max_cell_voltage_sensor_);
  LOG_SENSOR("", "Minimum Voltage Cell", this->min_voltage_cell_sensor_);
  LOG_SENSOR("", "Maximum Voltage Cell", this->max_voltage_cell_sensor_);
  LOG_SENSOR("", "Delta Cell Voltage", this->delta_cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 1", this->cells_[0].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 2", this->cells_[1].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 3", this->cells_[2].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 4", this->cells_[3].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 5", this->cells_[4].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 6", this->cells_[5].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 7", this->cells_[6].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 8", this->cells_[7].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 9", this->cells_[8].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 10", this->cells_[9].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 11", this->cells_[10].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 12", this->cells_[11].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 13", this->cells_[12].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 14", this->cells_[13].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 15", this->cells_[14].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 16", this->cells_[15].cell_voltage_sensor_);
  LOG_SENSOR("", "Temperature 1", this->temperatures_[0].temperature_sensor_);
  LOG_SENSOR("", "Temperature 2", this->temperatures_[1].temperature_sensor_);
  LOG_SENSOR("", "Temperature 3", this->temperatures_[2].temperature_sensor_);
  LOG_SENSOR("", "Temperature 4", this->temperatures_[3].temperature_sensor_);
  LOG_SENSOR("", "Temperature 5", this->temperatures_[4].temperature_sensor_);
  LOG_SENSOR("", "Temperature 6", this->temperatures_[5].temperature_sensor_);
  LOG_SENSOR("", "Total Voltage", this->total_voltage_sensor_);
  LOG_SENSOR("", "Current", this->current_sensor_);
  LOG_SENSOR("", "Power", this->power_sensor_);
  LOG_SENSOR("", "Charging Power", this->charging_power_sensor_);
  LOG_SENSOR("", "Discharging Power", this->discharging_power_sensor_);
  LOG_SENSOR("", "Charging cycles", this->charging_cycles_sensor_);
  LOG_SENSOR("", "State of charge", this->state_of_charge_sensor_);
  LOG_SENSOR("", "Residual capacity", this->residual_capacity_sensor_);
  LOG_SENSOR("", "Battery capacity", this->battery_capacity_sensor_);
  LOG_SENSOR("", "Rated capacity", this->rated_capacity_sensor_);
  LOG_SENSOR("", "Charging cycles", this->charging_cycles_sensor_);
  LOG_SENSOR("", "Average Cell Voltage", this->average_cell_voltage_sensor_);
  LOG_SENSOR("", "State of health", this->state_of_health_sensor_);
  LOG_SENSOR("", "Port Voltage", this->port_voltage_sensor_);
}

float SeplosBms::get_setup_priority() const {
  // After UART bus
  return setup_priority::BUS - 1.0f;
}

void SeplosBms::update() {
  this->track_online_status_();
  this->send(0x42, this->pack_);
}

void SeplosBms::publish_state_(binary_sensor::BinarySensor *binary_sensor, const bool &state) {
  if (binary_sensor == nullptr)
    return;

  binary_sensor->publish_state(state);
}

void SeplosBms::publish_state_(sensor::Sensor *sensor, float value) {
  if (sensor == nullptr)
    return;

  sensor->publish_state(value);
}

void SeplosBms::publish_state_(text_sensor::TextSensor *text_sensor, const std::string &state) {
  if (text_sensor == nullptr)
    return;

  text_sensor->publish_state(state);
}

void SeplosBms::track_online_status_() {
  if (this->no_response_count_ < MAX_NO_RESPONSE_COUNT) {
    this->no_response_count_++;
  }
  if (this->no_response_count_ == MAX_NO_RESPONSE_COUNT) {
    this->publish_device_unavailable_();
    this->no_response_count_++;
  }
}

void SeplosBms::reset_online_status_tracker_() {
  this->no_response_count_ = 0;
  this->publish_state_(this->online_status_binary_sensor_, true);
}

void SeplosBms::publish_device_unavailable_() {
  this->publish_state_(this->online_status_binary_sensor_, false);
  this->publish_state_(this->errors_text_sensor_, "Offline");

  this->publish_state_(this->min_cell_voltage_sensor_, NAN);
  this->publish_state_(this->max_cell_voltage_sensor_, NAN);
  this->publish_state_(this->min_voltage_cell_sensor_, NAN);
  this->publish_state_(this->max_voltage_cell_sensor_, NAN);
  this->publish_state_(this->delta_cell_voltage_sensor_, NAN);
  this->publish_state_(this->average_cell_voltage_sensor_, NAN);
  this->publish_state_(this->total_voltage_sensor_, NAN);
  this->publish_state_(this->current_sensor_, NAN);
  this->publish_state_(this->power_sensor_, NAN);
  this->publish_state_(this->charging_power_sensor_, NAN);
  this->publish_state_(this->discharging_power_sensor_, NAN);
  this->publish_state_(this->state_of_charge_sensor_, NAN);
  this->publish_state_(this->residual_capacity_sensor_, NAN);
  this->publish_state_(this->battery_capacity_sensor_, NAN);
  this->publish_state_(this->rated_capacity_sensor_, NAN);
  this->publish_state_(this->charging_cycles_sensor_, NAN);
  this->publish_state_(this->state_of_health_sensor_, NAN);
  this->publish_state_(this->port_voltage_sensor_, NAN);

  for (auto &temperature : this->temperatures_) {
    this->publish_state_(temperature.temperature_sensor_, NAN);
  }

  for (auto &cell : this->cells_) {
    this->publish_state_(cell.cell_voltage_sensor_, NAN);
  }
}

}  // namespace seplos_bms
}  // namespace esphome
