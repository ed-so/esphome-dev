#include "config_number.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

static const char *const TAG = "wwmr01.number";

namespace esphome {
namespace wwmr01 {

void WWMR01TimeoutNumber::control(float timeout) {
  this->publish_state(timeout);
  this->parent_->new_config.timeout = timeout;
}

// void WWMR01MinDistanceNumber::control(float min_gate) {
//   if ((uint16_t) min_gate > this->parent_->new_config.max_gate) {
//     min_gate = this->parent_->get_min_distance_value();
//   } else {
//     this->parent_->new_config.min_gate = (uint16_t) min_gate;
//   }
//   this->publish_state(min_gate);
// }

// void WW_MR01MaxDistanceNumber::control(float max_gate) {
//   if ((uint16_t) max_gate < this->parent_->new_config.min_gate) {
//     max_gate = this->parent_->get_max_distance_value();
//   } else {
//     this->parent_->new_config.max_gate = (uint16_t) max_gate;
//   }
//   this->publish_state(max_gate);
// }

// void WW_MR01GateSelectNumber::control(float gate_select) {
//   const uint8_t gate = (uint8_t) gate_select;
//   this->publish_state(gate_select);
//   this->parent_->publish_gate_move_threshold(gate);
//   this->parent_->publish_gate_still_threshold(gate);
// }

// void WW_MR01MoveSensFactorNumber::control(float move_factor) {
//   this->publish_state(move_factor);
//   this->parent_->gate_move_sensitivity_factor = move_factor;
// }

// void WW_MR01StillSensFactorNumber::control(float still_factor) {
//   this->publish_state(still_factor);
//   this->parent_->gate_still_sensitivity_factor = still_factor;
// }

// WW_MR01MoveThresholdNumbers::WW_MR01MoveThresholdNumbers(uint8_t gate) : gate_(gate) {}

// void WW_MR01MoveThresholdNumbers::control(float move_threshold) {
//   this->publish_state(move_threshold);
//   if (!this->parent_->is_gate_select()) {
//     this->parent_->new_config.move_thresh[this->gate_] = move_threshold;
//   } else {
//     this->parent_->new_config.move_thresh[this->parent_->get_gate_select_value()] = move_threshold;
//   }
// }

// WW_MR01StillThresholdNumbers::WW_MR01StillThresholdNumbers(uint8_t gate) : gate_(gate) {}

// void WW_MR01StillThresholdNumbers::control(float still_threshold) {
//   this->publish_state(still_threshold);
//   if (!this->parent_->is_gate_select()) {
//     this->parent_->new_config.still_thresh[this->gate_] = still_threshold;
//   } else {
//     this->parent_->new_config.still_thresh[this->parent_->get_gate_select_value()] = still_threshold;
//   }
// }

}  // namespace wwmr01
}  // namespace esphome
