#pragma once
#include <vector>
#include "esphome/core/log.h"

class ZTEFB101Parser {
 public:
  void parse(const std::vector<uint8_t>& data);

 protected:
  void parse_cells(const std::vector<uint8_t>& data);
  void parse_status(const std::vector<uint8_t>& data);
  void parse_cycles(const std::vector<uint8_t>& data);
};
