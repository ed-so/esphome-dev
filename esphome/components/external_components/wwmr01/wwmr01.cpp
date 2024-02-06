#include "esphome/core/helpers.h"
#include "wwmr01.h"

namespace esphome {
namespace wwmr01 {

static const char *const TAG = "wwmr01";

float WWMR01Component::get_setup_priority() const { return setup_priority::BUS; }

void WWMR01Component::dump_config() {
  ESP_LOGCONFIG(TAG, "WW_MR01:");
  ESP_LOGCONFIG(TAG, "  Firmware Version : %7s", this->firmware_ver_);
  ESP_LOGCONFIG(TAG, "WW_MR01 Number:");

#ifdef USE_NUMBER
  LOG_NUMBER(TAG, "  Timeout:", this->timeout_number_);
  // LOG_NUMBER(TAG, "  Gate Max Distance:", this->max_gate_distance_number_);
  // LOG_NUMBER(TAG, "  Gate Min Distance:", this->min_gate_distance_number_);
  // LOG_NUMBER(TAG, "  Gate Select:", this->gate_select_number_);
  // for (uint8_t gate = 0; gate < LD2420_TOTAL_GATES; gate++) {
  //   LOG_NUMBER(TAG, "  Gate Move Threshold:", this->gate_move_threshold_numbers_[gate]);
  //   LOG_NUMBER(TAG, "  Gate Still Threshold::", this->gate_still_threshold_numbers_[gate]);
  // }
#endif

#ifdef USE_BUTTON
  LOG_BUTTON(TAG, "  Apply Config:", this->apply_config_button_);
  LOG_BUTTON(TAG, "  Revert Edits:", this->revert_config_button_);
  LOG_BUTTON(TAG, "  Factory Reset:", this->factory_reset_button_);
  LOG_BUTTON(TAG, "  Restart Module:", this->restart_module_button_);
#endif

#ifdef USE_SELECT
  // ESP_LOGCONFIG(TAG, "WW_MR01 Select:");
  // LOG_SELECT(TAG, "  Operating Mode", this->operating_selector_);
#endif
  // if (this->get_firmware_int_(firmware_ver_) < CALIBRATE_VERSION_MIN) {
  //  ESP_LOGW(TAG, "WW_MR01 Firmware Version %s and older are only supported in Simple Mode", firmware_ver_);
  // }
}

void WWMR01Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up WWMR01...");

//   if (this->set_config_mode(true) == LD2420_ERROR_TIMEOUT) {
//     ESP_LOGE(TAG, "WW_MR01 module has failed to respond, check baud rate and serial connections.");
//     this->mark_failed();
//     return;
//   }
//   this->get_min_max_distances_timeout_();

#ifdef USE_NUMBER
  this->init_config_numbers();
#endif

  this->get_firmware_version_();
  const char *pfw = this->firmware_ver_;
  std::string fw_str(pfw);

  for (auto &listener : listeners_) {
    listener->on_fw_version(fw_str);
  }

//   for (uint8_t gate = 0; gate < LD2420_TOTAL_GATES; gate++) {
//     delay_microseconds_safe(125);
//     this->get_gate_threshold_(gate);
//   }

//   memcpy(&this->new_config, &this->current_config, sizeof(this->current_config));
//   if (get_firmware_int_(ld2420_firmware_ver_) < CALIBRATE_VERSION_MIN) {
//     this->set_operating_mode(OP_SIMPLE_MODE_STRING);
//     this->operating_selector_->publish_state(OP_SIMPLE_MODE_STRING);
//     this->set_mode_(CMD_SYSTEM_MODE_SIMPLE);
//     ESP_LOGW(TAG, "LD2420 Frimware Version %s and older are only supported in Simple Mode", ld2420_firmware_ver_);
//   } else {
//     this->set_mode_(CMD_SYSTEM_MODE_ENERGY);
//     this->operating_selector_->publish_state(OP_NORMAL_MODE_STRING);
//   }
#ifdef USE_NUMBER
  this->init_config_numbers();
#endif
//   this->set_system_mode(this->system_mode_);
//   this->set_config_mode(false);


  ESP_LOGCONFIG(TAG, "WWMR01 setup complete.");
}

void WWMR01Component::apply_config_action() {
  // const uint8_t checksum = calc_checksum(&this->new_config, sizeof(this->new_config));
  // if (checksum == calc_checksum(&this->current_config, sizeof(this->current_config))) {
  //   ESP_LOGCONFIG(TAG, "No configuration change detected");
  //   return;
  // }
  ESP_LOGCONFIG(TAG, "Reconfiguring WWMR01...");
  // if (this->set_config_mode(true) == LD2420_ERROR_TIMEOUT) {
  //   ESP_LOGE(TAG, "LD2420 module has failed to respond, check baud rate and serial connections.");
  //   this->mark_failed();
  //   return;
  // }
  // this->set_min_max_distances_timeout(this->new_config.max_gate, this->new_config.min_gate, this->new_config.timeout);
  // for (uint8_t gate = 0; gate < LD2420_TOTAL_GATES; gate++) {
  //   delay_microseconds_safe(125);
  //   this->set_gate_threshold(gate);
  // }
  // memcpy(&current_config, &new_config, sizeof(new_config));
#ifdef USE_NUMBER
  this->init_config_numbers();
#endif
  // this->set_system_mode(this->system_mode_);
  // this->set_config_mode(false);  // Disable config mode to save new values in LD2420 nvm
  // this->set_operating_mode(OP_NORMAL_MODE_STRING);
  ESP_LOGCONFIG(TAG, "WWMR01 reconfig complete.");
}

void WWMR01Component::factory_reset_action() {
  ESP_LOGCONFIG(TAG, "Setiing factory defaults...");
  // if (this->set_config_mode(true) == LD2420_ERROR_TIMEOUT) {
  //   ESP_LOGE(TAG, "LD2420 module has failed to respond, check baud rate and serial connections.");
  //   this->mark_failed();
  //   return;
  // }
  // this->set_min_max_distances_timeout(FACTORY_MAX_GATE, FACTORY_MIN_GATE, FACTORY_TIMEOUT);
#ifdef USE_NUMBER
  this->timeout_number_->state = FACTORY_TIMEOUT;
  // this->min_gate_distance_number_->state = FACTORY_MIN_GATE;
  // this->max_gate_distance_number_->state = FACTORY_MAX_GATE;
#endif
  // for (uint8_t gate = 0; gate < LD2420_TOTAL_GATES; gate++) {
  //   this->new_config.move_thresh[gate] = FACTORY_MOVE_THRESH[gate];
  //   this->new_config.still_thresh[gate] = FACTORY_STILL_THRESH[gate];
  //   delay_microseconds_safe(125);
  //   this->set_gate_threshold(gate);
  // }
  // memcpy(&this->current_config, &this->new_config, sizeof(this->new_config));
  // this->set_system_mode(this->system_mode_);
  // this->set_config_mode(false);
#ifdef USE_NUMBER
  this->init_config_numbers();
  this->refresh_config_numbers();
#endif
  ESP_LOGCONFIG(TAG, "WWMR01 factory reset complete.");
}

void WWMR01Component::restart_module_action() {
  ESP_LOGCONFIG(TAG, "Restarting WWMR01 module...");
  this->send_module_restart();
  // delay_microseconds_safe(45000);
  // this->set_config_mode(true);
  // this->set_system_mode(system_mode_);
  // this->set_config_mode(false);
  ESP_LOGCONFIG(TAG, "WWMR01 Restarted.");
}

void WWMR01Component::revert_config_action() {
  memcpy(&this->new_config, &this->current_config, sizeof(this->current_config));
#ifdef USE_NUMBER
  this->init_config_numbers();
#endif
  ESP_LOGCONFIG(TAG, "WWMR01 Reverted config number edits.");
}


void WWMR01Component::loop() {
  // If there is a active send command do not process it here, the send command call will handle it.
  if (!get_cmd_active_()) {
    if (!available())
      return;
    static uint8_t buffer[2048];
    static uint8_t rx_data;
    while (available()) {
      rx_data = read();
      this->readline_(rx_data, buffer, sizeof(buffer));
    }
  }
}  

void WWMR01Component::readline_(int rx_data, uint8_t *buffer, int len) {
  static int pos = 0;

  if (rx_data >= 0) {
    if (pos < len - 1) {
      buffer[pos++] = rx_data;
      buffer[pos] = 0;
    } else {
      pos = 0;
    }
    if (pos >= 4) {
      // if (memcmp(&buffer[pos - 4], &CMD_FRAME_FOOTER, sizeof(CMD_FRAME_FOOTER)) == 0) {
      //   this->set_cmd_active_(false);  // Set command state to inactive after responce.
      //   this->handle_ack_data_(buffer, pos);
      //   pos = 0;
      // } else if ((buffer[pos - 2] == 0x0D && buffer[pos - 1] == 0x0A) && (get_mode_() == CMD_SYSTEM_MODE_SIMPLE)) {
      //   this->handle_simple_mode_(buffer, pos);
      //   pos = 0;
      // } else if ((memcmp(&buffer[pos - 4], &ENERGY_FRAME_FOOTER, sizeof(ENERGY_FRAME_FOOTER)) == 0) &&
      //            (get_mode_() == CMD_SYSTEM_MODE_ENERGY)) {
      //   this->handle_energy_mode_(buffer, pos);
      //   pos = 0;
      // }
    }
  }
}  


// Sends a restart and set system running mode to normal
void WWMR01Component::send_module_restart() { this->wwmr01_restart(); }

void WWMR01Component::wwmr01_restart() {
  CmdFrameT cmd_frame;
  // cmd_frame.data_length = 0;
  // cmd_frame.header = CMD_FRAME_HEADER;
  // cmd_frame.command = CMD_RESTART;
  // cmd_frame.footer = CMD_FRAME_FOOTER;
  ESP_LOGD(TAG, "Sending restart command: %2X", cmd_frame.command);
  //this->send_cmd_from_array(cmd_frame);
}


int WWMR01Component::get_firmware_int_(const char *version_string) { return 0; }

void WWMR01Component::get_firmware_version_() {
  CmdFrameT cmd_frame;
  // cmd_frame.data_length = 0;
  // cmd_frame.header = CMD_FRAME_HEADER;
  // cmd_frame.command = CMD_READ_VERSION;
  // cmd_frame.footer = CMD_FRAME_FOOTER;

  ESP_LOGD(TAG, "Sending read firmware version command: %2X", cmd_frame.command);
  // this->send_cmd_from_array(cmd_frame);
} 

#ifdef USE_NUMBER
void WWMR01Component::init_config_numbers() {
  if (this->timeout_number_ != nullptr)
    this->timeout_number_->publish_state(static_cast<uint16_t>(this->current_config.timeout));
  // if (this->gate_select_number_ != nullptr)
  //   this->gate_select_number_->publish_state(0);
  // if (this->min_gate_distance_number_ != nullptr)
  //   this->min_gate_distance_number_->publish_state(static_cast<uint16_t>(this->current_config.min_gate));
  // if (this->max_gate_distance_number_ != nullptr)
  //   this->max_gate_distance_number_->publish_state(static_cast<uint16_t>(this->current_config.max_gate));
  // if (this->gate_move_sensitivity_factor_number_ != nullptr)
  //   this->gate_move_sensitivity_factor_number_->publish_state(this->gate_move_sensitivity_factor);
  // if (this->gate_still_sensitivity_factor_number_ != nullptr)
  //   this->gate_still_sensitivity_factor_number_->publish_state(this->gate_still_sensitivity_factor);
  // for (uint8_t gate = 0; gate < LD2420_TOTAL_GATES; gate++) {
  //   if (this->gate_still_threshold_numbers_[gate] != nullptr) {
  //     this->gate_still_threshold_numbers_[gate]->publish_state(
  //         static_cast<uint16_t>(this->current_config.still_thresh[gate]));
  //   }
  //   if (this->gate_move_threshold_numbers_[gate] != nullptr) {
  //     this->gate_move_threshold_numbers_[gate]->publish_state(
  //         static_cast<uint16_t>(this->current_config.move_thresh[gate]));
  //   }
  // }
}

void WWMR01Component::refresh_config_numbers() {
  this->timeout_number_->publish_state(this->new_config.timeout);
  // this->min_gate_distance_number_->publish_state(this->new_config.min_gate);
  // this->max_gate_distance_number_->publish_state(this->new_config.max_gate);
}

#endif

}  // namespace wwmr01
}  // namespace esphome


