#include "config_number.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

static const char *const TAG = "wwresi.number";

namespace esphome {
namespace wwresi {

void WWRESITimeoutNumber::control(float timeout) {
  this->publish_state(timeout);
  this->parent_->new_config.timeout = timeout;
}

void WWRESIResistanceNumber::control(float value) {
  if ((int) value > this->parent_->new_config.resistance) {
    value = this->parent_->get_resistance_value();
  } else {
    this->parent_->new_config.resistance = (int) value;
  }
  this->publish_state(value);
}

// void WWRESIMaxDistanceNumber::control(float max_gate) {
//   if ((uint16_t) max_gate < this->parent_->new_config.min_gate) {
//     max_gate = this->parent_->get_max_distance_value();
//   } else {
//     this->parent_->new_config.max_gate = (uint16_t) max_gate;
//   }
//   this->publish_state(max_gate);
// }

// void WWRESIGateSelectNumber::control(float gate_select) {
//   const uint8_t gate = (uint8_t) gate_select;
//   this->publish_state(gate_select);
//   this->parent_->publish_gate_move_threshold(gate);
//   this->parent_->publish_gate_still_threshold(gate);
// }

// void WWRESIMoveSensFactorNumber::control(float move_factor) {
//   this->publish_state(move_factor);
//   this->parent_->gate_move_sensitivity_factor = move_factor;
// }

// void WWRESIStillSensFactorNumber::control(float still_factor) {
//   this->publish_state(still_factor);
//   this->parent_->gate_still_sensitivity_factor = still_factor;
// }

// WW_MR01MoveThresholdNumbers::WW_MR01MoveThresholdNumbers(uint8_t gate) : gate_(gate) {}

// void WWRESIMoveThresholdNumbers::control(float move_threshold) {
//   this->publish_state(move_threshold);
//   if (!this->parent_->is_gate_select()) {
//     this->parent_->new_config.move_thresh[this->gate_] = move_threshold;
//   } else {
//     this->parent_->new_config.move_thresh[this->parent_->get_gate_select_value()] = move_threshold;
//   }
// }

// WWRESIStillThresholdNumbers::WW_MR01StillThresholdNumbers(uint8_t gate) : gate_(gate) {}

// void WWRESIStillThresholdNumbers::control(float still_threshold) {
//   this->publish_state(still_threshold);
//   if (!this->parent_->is_gate_select()) {
//     this->parent_->new_config.still_thresh[this->gate_] = still_threshold;
//   } else {
//     this->parent_->new_config.still_thresh[this->parent_->get_gate_select_value()] = still_threshold;
//   }
// }

}  // namespace wwresi
}  // namespace esphome
