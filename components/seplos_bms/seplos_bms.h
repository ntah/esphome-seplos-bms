#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace seplos_bms {

static const char *const TAG = "seplos_bms";

class SeplosBms : public Component {
 public:
  // ---- CELL STRUCT ----
  struct Cell {
    sensor::Sensor *cell_voltage_sensor_{nullptr};
  };

  // ---- TEMPERATURE STRUCT ----
  struct Temperature {
    sensor::Sensor *temperature_sensor_{nullptr};
  };

  // MAX 16 cells (ZTE 15, SHOTO 15)
  Cell cells_[16];
  Temperature temperatures_[8];  // ZTE uses 4, SHOTO uses 5

  // ---- COMMON SENSORS ----
  sensor::Sensor *min_cell_voltage_sensor_{nullptr};
  sensor::Sensor *max_cell_voltage_sensor_{nullptr};
  sensor::Sensor *delta_cell_voltage_sensor_{nullptr};
  sensor::Sensor *average_cell_voltage_sensor_{nullptr};
  sensor::Sensor *min_voltage_cell_sensor_{nullptr};
  sensor::Sensor *max_voltage_cell_sensor_{nullptr};

  sensor::Sensor *current_sensor_{nullptr};
  sensor::Sensor *power_sensor_{nullptr};
  sensor::Sensor *charging_power_sensor_{nullptr};
  sensor::Sensor *discharging_power_sensor_{nullptr};

  sensor::Sensor *total_voltage_sensor_{nullptr};
  sensor::Sensor *residual_capacity_sensor_{nullptr};
  sensor::Sensor *battery_capacity_sensor_{nullptr};

  sensor::Sensor *state_of_charge_sensor_{nullptr};
  sensor::Sensor *state_of_health_sensor_{nullptr};
  sensor::Sensor *charging_cycles_sensor_{nullptr};

  text_sensor::TextSensor *last_update_sensor_{nullptr};

  // ---- MAIN ENTRY POINT FROM MODBUS ----
  void on_seplos_modbus_data(const std::vector<uint8_t> &data);

  // ---- PROTOCOL HANDLERS ----
  void on_zte_telemetry_(const std::vector<uint8_t> &data);
  void on_shoto_telemetry_(const std::vector<uint8_t> &data);

  void dump_config() override;

 protected:
  void publish_state_(sensor::Sensor *sensor, float value) {
    if (sensor)
      sensor->publish_state(value);
  }

  void publish_state_(text_sensor::TextSensor *sensor, const std::string &value) {
    if (sensor)
      sensor->publish_state(value);
  }

  void reset_online_status_tracker_() {
    // Optional: implement if needed
  }
};

}  // namespace seplos_bms
}  // namespace esphome
