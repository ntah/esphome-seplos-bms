// seplos_bms.cpp
#include "seplos_bms.h"
#include "esphome/core/log.h"
#include <vector>
#include <cstdint>
#include <cmath>

namespace esphome {
namespace seplos_bms {

static const char *TAG = "seplos_bms";

void SeplosBms::publish_state_(Sensor *sensor, float value) {
  if (sensor != nullptr) {
    sensor->publish_state(value);
  }
}

void SeplosBms::publish_state_(TextSensor *sensor, const std::string &value) {
  if (sensor != nullptr) {
    sensor->publish_state(value);
  }
}

// -------------------------
// Helper / common utilities
// -------------------------
static inline uint16_t read_u16_be(const std::vector<uint8_t> &data, size_t i) {
  if (i + 1 >= data.size()) return 0;
  return (static_cast<uint16_t>(data[i]) << 8) | static_cast<uint16_t>(data[i + 1]);
}

// -------------------------
// FB101 parser
// -------------------------
void SeplosBms::on_zte_fb101_(const std::vector<uint8_t> &data) {
  ESP_LOGI(TAG, "ZTE FB101 telemetry frame (%d bytes) received", (int)data.size());
  if (data.size() < 58) {
    ESP_LOGW(TAG, "ZTE FB101 frame too short: %d bytes", (int)data.size());
    return;
  }

  const float rated_capacity = this->rated_capacity_ > 0.0f ? this->rated_capacity_ : 100.0f;

  auto zte_u16 = [&](size_t i) -> uint16_t {
    return read_u16_be(data, i);
  };

  const int CELLS = 15;
  float sum_v = 0.0f;
  float min_v = 1e6f, max_v = -1e6f;
  int min_idx = -1, max_idx = -1;

  // Cells (words starting at offset 8)
  for (int i = 0; i < CELLS; ++i) {
    size_t idx = 8 + i * 2;
    if (idx + 1 >= data.size()) break;
    uint16_t raw = zte_u16(idx);
    float v = ((float)raw) / 1000.0f;
    sum_v += v;
    if (v < min_v) { min_v = v; min_idx = i + 1; }
    if (v > max_v) { max_v = v; max_idx = i + 1; }

    if (i < static_cast<int>(sizeof(this->cells_) / sizeof(this->cells_[0])) &&
        this->cells_[i].cell_voltage_sensor_ != nullptr) {
      this->publish_state_(this->cells_[i].cell_voltage_sensor_, v);
    }
  }

  float avg_v = (CELLS > 0) ? (sum_v / (float)CELLS) : 0.0f;
  float total_v = sum_v;

  if (this->min_cell_voltage_sensor_ != nullptr) this->publish_state_(this->min_cell_voltage_sensor_, min_v);
  if (this->max_cell_voltage_sensor_ != nullptr) this->publish_state_(this->max_cell_voltage_sensor_, max_v);
  if (this->delta_cell_voltage_sensor_ != nullptr) this->publish_state_(this->delta_cell_voltage_sensor_, (max_v - min_v));
  if (this->average_cell_voltage_sensor_ != nullptr) this->publish_state_(this->average_cell_voltage_sensor_, avg_v);
  if (min_idx > 0 && this->min_voltage_cell_sensor_ != nullptr) this->publish_state_(this->min_voltage_cell_sensor_, (float)min_idx);
  if (max_idx > 0 && this->max_voltage_cell_sensor_ != nullptr) this->publish_state_(this->max_voltage_cell_voltage_sensor_, (float)max_idx);
  if (this->total_voltage_sensor_ != nullptr) this->publish_state_(this->total_voltage_sensor_, total_v);

  // Temperatures (count at data[38], words start at 39, scaled /100)
  if (data.size() > 39) {
    uint8_t tcount = data[38];
    for (int t = 0; t < 3 && t < (int)tcount; ++t) {
      size_t tidx = 39 + t * 2;
      if (tidx + 1 >= data.size()) break;
      float t_raw = static_cast<float>(zte_u16(tidx));
      float t_c = t_raw / 100.0f;
      if (t < static_cast<int>(sizeof(this->temperatures_) / sizeof(this->temperatures_[0])) &&
          this->temperatures_[t].temperature_sensor_ != nullptr) {
        this->publish_state_(this->temperatures_[t].temperature_sensor_, t_c);
      }
    }
  }

  // Current (signed int16, empiric scale /100 => A)
  int16_t cur_raw = static_cast<int16_t>(zte_u16(45));
  float current = cur_raw / 100.0f;
  if (this->current_sensor_ != nullptr) this->publish_state_(this->current_sensor_, current);

  // Read SOH, SOC, cycles (offsets validated from frames)
  uint16_t soh_raw = zte_u16(54);
  uint16_t soc_raw = zte_u16(52);
  uint16_t cycles_raw = zte_u16(56);

  float soh = soh_raw / 100.0f;
  float soc = soc_raw / 100.0f;

  // Derive full capacity from SOH (preferred), fallback to cap_raw decode
  float full_cap = 0.0f;
  if (rated_capacity > 0.0f && soh_raw > 0) {
    full_cap = rated_capacity * (soh / 100.0f);
  } else {
    uint16_t cap_raw = zte_u16(47);
    const float DECODE_CAP_FACTOR = 52.8f;
    full_cap = cap_raw / DECODE_CAP_FACTOR;
  }

  if (this->battery_capacity_sensor_ != nullptr) this->publish_state_(this->battery_capacity_sensor_, full_cap);
  if (this->state_of_health_sensor_ != nullptr) this->publish_state_(this->state_of_health_sensor_, soh);
  if (this->state_of_charge_sensor_ != nullptr) this->publish_state_(this->state_of_charge_sensor_, soc);

  float rem_cap = (full_cap > 0.0f) ? (full_cap * (soc / 100.0f)) : 0.0f;
  if (this->residual_capacity_sensor_ != nullptr) this->publish_state_(this->residual_capacity_sensor_, rem_cap);

  // Power calculation
  if (!std::isfinite(total_v) || total_v <= 0.0f) total_v = avg_v * (float)CELLS;
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

  // Cycles
  if (this->charging_cycles_sensor_ != nullptr) this->publish_state_(this->charging_cycles_sensor_, (float)cycles_raw);

  ESP_LOGD(TAG,
           "FB101: soh_raw=%u soh=%.2f soc_raw=%u soc=%.2f full_cap=%.2f cycles=%u cur_raw=%d total_v=%.3f",
           (unsigned)soh_raw, soh, (unsigned)soc_raw, soc, full_cap, (unsigned)cycles_raw, (int)cur_raw, total_v);
}

// -------------------------
// FB100C1 parser
// -------------------------
void SeplosBms::on_zte_fb100c1_(const std::vector<uint8_t> &data) {
  ESP_LOGI(TAG, "ZTE FB100C1 telemetry frame (%d bytes) received", (int)data.size());
  if (data.size() < 58) {
    ESP_LOGW(TAG, "FB100C1 frame too short: %d bytes", (int)data.size());
    return;
  }

  const float rated_capacity = this->rated_capacity_ > 0.0f ? this->rated_capacity_ : 100.0f;

  auto u16 = [&](size_t i) -> uint16_t {
    return read_u16_be(data, i);
  };

  const int CELL_COUNT = 15;
  float sum_v = 0.0f;
  float min_v = 1e6f, max_v = -1e6f;
  int min_idx = -1, max_idx = -1;

  for (int i = 0; i < CELL_COUNT; ++i) {
    size_t idx = 8 + i * 2;
    if (idx + 1 >= data.size()) break;
    uint16_t raw = u16(idx);
    float v = ((float)raw) / 1000.0f;
    sum_v += v;
    if (v < min_v) { min_v = v; min_idx = i + 1; }
    if (v > max_v) { max_v = v; max_idx = i + 1; }
    if (i < static_cast<int>(sizeof(this->cells_) / sizeof(this->cells_[0])) &&
        this->cells_[i].cell_voltage_sensor_ != nullptr) {
      this->publish_state_(this->cells_[i].cell_voltage_sensor_, v);
    }
  }

  float avg_v = (CELL_COUNT > 0) ? (sum_v / (float)CELL_COUNT) : 0.0f;
  if (this->min_cell_voltage_sensor_ != nullptr) this->publish_state_(this->min_cell_voltage_sensor_, min_v);
  if (this->max_cell_voltage_sensor_ != nullptr) this->publish_state_(this->max_cell_voltage_sensor_, max_v);
  if (this->delta_cell_voltage_sensor_ != nullptr) this->publish_state_(this->delta_cell_voltage_sensor_, (max_v - min_v));
  if (this->average_cell_voltage_sensor_ != nullptr) this->publish_state_(this->average_cell_voltage_sensor_, avg_v);
  if (min_idx > 0 && this->min_voltage_cell_sensor_ != nullptr) this->publish_state_(this->min_voltage_cell_sensor_, (float)min_idx);
  if (max_idx > 0 && this->max_voltage_cell_sensor_ != nullptr) this->publish_state_(this->max_voltage_cell_sensor_, (float)max_idx);
  if (this->total_voltage_sensor_ != nullptr) this->publish_state_(this->total_voltage_sensor_, sum_v);

  // Temperatures
  uint8_t tcount = data[38];
  for (int t = 0; t < 3 && t < (int)tcount; ++t) {
    size_t tidx = 39 + t * 2;
    if (tidx + 1 >= data.size()) break;
    float t_raw = static_cast<float>(u16(tidx));
    float t_c = t_raw / 100.0f;
    if (t < static_cast<int>(sizeof(this->temperatures_) / sizeof(this->temperatures_[0])) &&
        this->temperatures_[t].temperature_sensor_ != nullptr) {
      this->publish_state_(this->temperatures_[t].temperature_sensor_, t_c);
    }
  }

  // Current signed
  int16_t cur_raw = static_cast<int16_t>(u16(45));
  float current = cur_raw / 100.0f;
  if (this->current_sensor_ != nullptr) this->publish_state_(this->current_sensor_, current);

  // SOH / SOC / FullCap / Residual
  uint16_t soh_raw = u16(54);
  uint16_t soc_raw = u16(52);
  float soh = soh_raw / 100.0f;
  float soc = soc_raw / 100.0f;

  // Prefer deriving full capacity from SOH
  float full_cap = 0.0f;
  if (rated_capacity > 0.0f && soh_raw > 0) {
    full_cap = rated_capacity * (soh / 100.0f);
  } else {
    uint16_t cap_raw = u16(47);
    const float DECODE_CAP_FACTOR = 52.8f;
    full_cap = cap_raw / DECODE_CAP_FACTOR;
  }

  if (this->battery_capacity_sensor_ != nullptr) this->publish_state_(this->battery_capacity_sensor_, full_cap);
  if (this->state_of_health_sensor_ != nullptr) this->publish_state_(this->state_of_health_sensor_, soh);
  if (this->state_of_charge_sensor_ != nullptr) this->publish_state_(this->state_of_charge_sensor_, soc);

  float rem_cap = (full_cap > 0.0f) ? (full_cap * (soc / 100.0f)) : 0.0f;
  if (this->residual_capacity_sensor_ != nullptr) this->publish_state_(this->residual_capacity_sensor_, rem_cap);

  // Power
  float total_v = sum_v;
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

  // Cycles
  uint16_t cycles_raw = u16(56);
  if (this->charging_cycles_sensor_ != nullptr) this->publish_state_(this->charging_cycles_sensor_, (float)cycles_raw);

  ESP_LOGD(TAG,
           "FB100C1: soh_raw=%u soh=%.2f soc_raw=%u soc=%.2f full_cap=%.2f cycles=%u cur_raw=%d total_v=%.3f",
           (unsigned)soh_raw, soh, (unsigned)soc_raw, soc, full_cap, (unsigned)cycles_raw, (int)cur_raw, total_v);
}

// -------------------------
// Shoto MCB parser (generic telemetry parser)
// -------------------------
void SeplosBms::on_telemetry_data_(const std::vector<uint8_t> &data) {
  ESP_LOGI(TAG, "Shoto MCB telemetry frame (%d bytes) received", (int)data.size());
  // A safe generic parser: try to extract voltages, temps, current, soc if present
  if (data.size() < 10) {
    ESP_LOGW(TAG, "Shoto frame too short: %d bytes", (int)data.size());
    return;
  }

  auto r16 = [&](size_t i) -> uint16_t {
    return read_u16_be(data, i);
  };

  // Heuristic: many Shoto frames also use 15 cells starting at offset 8
  const int CELL_COUNT = 15;
  float sum_v = 0.0f;
  float min_v = 1e6f, max_v = -1e6f;
  int min_idx = -1, max_idx = -1;
  for (int i = 0; i < CELL_COUNT; ++i) {
    size_t idx = 8 + i * 2;
    if (idx + 1 >= data.size()) break;
    uint16_t raw = r16(idx);
    float v = ((float)raw) / 1000.0f;
    sum_v += v;
    if (v < min_v) { min_v = v; min_idx = i + 1; }
    if (v > max_v) { max_v = v; max_idx = i + 1; }
    if (i < static_cast<int>(sizeof(this->cells_) / sizeof(this->cells_[0])) &&
        this->cells_[i].cell_voltage_sensor_ != nullptr) {
      this->publish_state_(this->cells_[i].cell_voltage_sensor_, v);
    }
  }

  float avg_v = (CELL_COUNT > 0) ? (sum_v / (float)CELL_COUNT) : 0.0f;
  if (this->min_cell_voltage_sensor_ != nullptr) this->publish_state_(this->min_cell_voltage_sensor_, min_v);
  if (this->max_cell_voltage_sensor_ != nullptr) this->publish_state_(this->max_cell_voltage_sensor_, max_v);
  if (this->delta_cell_voltage_sensor_ != nullptr) this->publish_state_(this->delta_cell_voltage_sensor_, (max_v - min_v));
  if (this->average_cell_voltage_sensor_ != nullptr) this->publish_state_(this->average_cell_voltage_sensor_, avg_v);
  if (this->total_voltage_sensor_ != nullptr) this->publish_state_(this->total_voltage_sensor_, sum_v);

  // Try to read temperatures if present (same offsets as ZTE)
  if (data.size() > 39) {
    uint8_t tcount = data[38];
    for (int t = 0; t < 3 && t < (int)tcount; ++t) {
      size_t tidx = 39 + t * 2;
      if (tidx + 1 >= data.size()) break;
      float t_raw = static_cast<float>(r16(tidx));
      float t_c = t_raw / 100.0f;
      if (t < static_cast<int>(sizeof(this->temperatures_) / sizeof(this->temperatures_[0])) &&
          this->temperatures_[t].temperature_sensor_ != nullptr) {
        this->publish_state_(this->temperatures_[t].temperature_sensor_, t_c);
      }
    }
  }

  // Current if present (heuristic at 45)
  if (45 + 1 < data.size()) {
    int16_t cur_raw = static_cast<int16_t>(r16(45));
    float current = cur_raw / 100.0f;
    if (this->current_sensor_ != nullptr) this->publish_state_(this->current_sensor_, current);

    // compute power
    float power = sum_v * current;
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
  }

  // Try SOC/SOH at same offsets if present
  if (52 + 1 < data.size()) {
    uint16_t soc_raw = r16(52);
    float soc = soc_raw / 100.0f;
    if (this->state_of_charge_sensor_ != nullptr) this->publish_state_(this->state_of_charge_sensor_, soc);
  }
  if (54 + 1 < data.size()) {
    uint16_t soh_raw = r16(54);
    float soh = soh_raw / 100.0f;
    if (this->state_of_health_sensor_ != nullptr) this->publish_state_(this->state_of_health_sensor_, soh);
  }

  ESP_LOGD(TAG, "Shoto: parsed heuristic telemetry (sum_v=%.3f avg_v=%.3f)", sum_v, avg_v);
}

}  // namespace seplos_bms
}  // namespace esphome
