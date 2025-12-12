#pragma once

#include "esphome/core/component.h"
#include "esphome/core/entity_base.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/seplos_modbus/seplos_modbus.h"

namespace esphome {
namespace seplos_bms {

class SeplosBms : public PollingComponent, public seplos_modbus::SeplosModbusDevice {
 public:
  void set_override_cell_count(uint8_t cell_count) { this->override_cell_count_ = cell_count; }

  // === PATCH: tipe BMS ===
  void set_tipe(const std::string &t) { this->tipe_ = t; }

  void dump_config() override;
  void update() override;

 protected:
  void on_seplos_modbus_data(const std::vector<uint8_t> &data) override;

  // Original parsers from Syssi
  void on_telemetry_data_(const std::vector<uint8_t> &data);
  void on_zte_telemetry_(const std::vector<uint8_t> &data);
  void on_shoto_telemetry_(const std::vector<uint8_t> &data);

  // === PATCH: New ZTE model specific parsers ===
  void on_zte_fb101_(const std::vector<uint8_t> &data);
  void on_zte_fb100c1_(const std::vector<uint8_t> &data);

  // YAML “tipe” field
  std::string tipe_;

  void publish_state_(sensor::Sensor *sensor, float value) {
    if (sensor != nullptr)
      sensor->publish_state(value);
  }

  void publish_state_(text_sensor::TextSensor *sensor, const std::string &value) {
    if (sensor != nullptr)
      sensor->publish_state(value);
  }

  uint8_t override_cell_count_{0};

  // === Existing sensor structure (unchanged) ===
  struct CellItem {
    sensor::Sensor *cell_voltage_sensor_{nullptr};
  };
  CellItem cells_[16];

  struct TempItem {
    sensor::Sensor *temperature_sensor_{nullptr};
  };
  TempItem temperatures_[6];

  sensor::Sensor *min_cell_voltage_sensor_{nullptr};
  sensor::Sensor *max_cell_voltage_sensor_{nullptr};
  sensor::Sensor *delta_cell_voltage_sensor_{nullptr};
  sensor::Sensor *average_cell_voltage_sensor_{nullptr};
  sensor::Sensor *total_voltage_sensor_{nullptr};

  sensor::Sensor *current_sensor_{nullptr};
  sensor::Sensor *battery_capacity_sensor_{nullptr};
  sensor::Sensor *state_of_health_sensor_{nullptr};
  sensor::Sensor *state_of_charge_sensor_{nullptr};
  sensor::Sensor *residual_capacity_sensor_{nullptr};

  sensor::Sensor *power_sensor_{nullptr};
  sensor::Sensor *charging_power_sensor_{nullptr};
  sensor::Sensor *discharging_power_sensor_{nullptr};

  sensor::Sensor *charging_cycles_sensor_{nullptr};

  binary_sensor::BinarySensor *online_status_binary_sensor_{nullptr};

  text_sensor::TextSensor *last_update_sensor_{nullptr};
  text_sensor::TextSensor *error_text_sensor_{nullptr};
};

}  // namespace seplos_bms
}  // namespace esphome
