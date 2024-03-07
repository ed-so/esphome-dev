
#include "wwresi.h"
#include <unistd.h>
#include <list>
#include <algorithm>

using std::string;

namespace esphome {
namespace wwresi {

static const char *const TAG = "wwresi";

/** global variables */



/* void WriteToAll(const string & str, LineBuffer::flags dest) {
  t_STREAM_LIST::iterator i;
  for(i=streams.begin(); i!=streams.end(); ++i) {
    if((*i)->m_flags & dest) {
      write((*i)->m_fd, str.data(), str.length());
    }
  }
}
 */

  //streams.push_front(new Line_Buffer(1, handleCommand));  // fd=1 socket


//---------------------------------------------------------------------------
/** Initialize this wwresi component
 *
 */
WWRESIComponent::WWRESIComponent() {

}

//---------------------------------------------------------------------------
/** Free all data:
 * socket_
 */
WWRESIComponent::~WWRESIComponent() {
  if (this->socket_) {
    this->socket_->close();
  }
}

//---------------------------------------------------------------------------
/** priority of setup(). higher -> executed earlier
 *
 * Defaults to 0.
 *
 * @return The setup priority of this component
 */
float WWRESIComponent::get_setup_priority() const { return setup_priority::ETHERNET; }

//---------------------------------------------------------------------------
/** prints the user configuration.
 *
 *
 *
 */
void WWRESIComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "WWRESI:");
  ESP_LOGCONFIG(TAG, "  Firmware Version : %7s", this->firmware_ver_);
  ESP_LOGCONFIG(TAG, "WWRESI Number:");

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
  if (this->get_firmware_int_(firmware_ver_) < CALIBRATE_VERSION_MIN) {
    ESP_LOGW(TAG, "WW_MR01 Firmware Version %s and older are only supported in Simple Mode", firmware_ver_);
  }
}

//---------------------------------------------------------------------------
/** Where the component's initialization should happen.
 *
 * Analogous to Arduino's setup(). This method is guaranteed to only be called once.
 * Defaults to doing nothing.
 */
void WWRESIComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up WWRESI...");
  this->socket_ = socket::socket_ip(SOCK_DGRAM, IPPROTO_IP);

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


//streams.push_front(new Line_Buffer(1, handleCommand));  // fd=1 socket
//streams.insert(streams.begin(), new Line_Buffer(1));  // fd=1 socket

  ESP_LOGCONFIG(TAG, "WWRESI setup complete.");
}

//---------------------------------------------------------------------------
void WWRESIComponent::apply_config_action() {
  // const uint8_t checksum = calc_checksum(&this->new_config, sizeof(this->new_config));
  // if (checksum == calc_checksum(&this->current_config, sizeof(this->current_config))) {
  //   ESP_LOGCONFIG(TAG, "No configuration change detected");
  //   return;
  // }
  ESP_LOGCONFIG(TAG, "Reconfiguring WWRESI...");
  // if (this->set_config_mode(true) == LD2420_ERROR_TIMEOUT) {
  //   ESP_LOGE(TAG, "LD2420 module has failed to respond, check baud rate and serial connections.");
  //   this->mark_failed();
  //   return;
  // }
  // this->set_min_max_distances_timeout(this->new_config.max_gate, this->new_config.min_gate,
  // this->new_config.timeout); for (uint8_t gate = 0; gate < LD2420_TOTAL_GATES; gate++) {
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
  ESP_LOGCONFIG(TAG, "WWRESI reconfig complete.");
}

//---------------------------------------------------------------------------
void WWRESIComponent::factory_reset_action() {
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
  ESP_LOGCONFIG(TAG, "WWRESI factory reset complete.");
}

//---------------------------------------------------------------------------
void WWRESIComponent::restart_module_action() {
  ESP_LOGCONFIG(TAG, "Restarting WWRESI module...");
  this->send_module_restart();
  // delay_microseconds_safe(45000);
  // this->set_config_mode(true);
  // this->set_system_mode(system_mode_);
  // this->set_config_mode(false);
  ESP_LOGCONFIG(TAG, "WWRESI Restarted.");
}

//---------------------------------------------------------------------------
void WWRESIComponent::revert_config_action() {
  memcpy(&this->new_config, &this->current_config, sizeof(this->current_config));
#ifdef USE_NUMBER
  this->init_config_numbers();
#endif
  ESP_LOGCONFIG(TAG, "WWRESI Reverted config number edits.");
}

//---------------------------------------------------------------------------
/** This method will be called repeatedly.
 *
 * Analogous to Arduino's loop(). setup() is guaranteed to be called before this.
 * Defaults to doing nothing.
 */
void WWRESIComponent::loop() {
  // If there is a active send command do not process it here, the send command call will handle it.
  if (!get_cmd_active_()) {


    //Line_Buffer mylb(1,handleCommand);

    //streams.insert(streams.begin(), new Line_Buffer(1));  // fd=1 socket
    //streams.push_front(new Line_Buffer(1, handleCommand));  // fd=1 socket

    // uart data input
    if (!available())
      return;
    static uint8_t buffer[2048];
    static uint8_t rx_data;
    while (available()) {
      rx_data = this->read();
      this->readline_(rx_data, buffer, sizeof(buffer));
    }
  }
}

//---------------------------------------------------------------------------
void WWRESIComponent::readline_(int rx_data, uint8_t *buffer, int len) {
  static int pos = 0;

  if (rx_data > 0) {
    ESP_LOGD(TAG, "rx: %02x", rx_data);
    switch (rx_data) {
      case '\n':  // Ignore LF 0x0a
        break;
      case '\r':  // Return on CR 0x0d
        handle_command_(buffer, pos);
        pos = 0;  // Reset position index ready for next time
        buffer[pos] = 0;
        break;
      default:
        if (pos < len - 1) {
          buffer[pos++] = rx_data;
          buffer[pos] = 0;
        }
    }
  }
}

//---------------------------------------------------------------------------
void WWRESIComponent::handle_command_(uint8_t *buffer, int len) {
  int fd;
  int ret;
  //t_Line_Buffer_List::iterator i;  

  ESP_LOGD(TAG, "CMD: %s", buffer);
  this->write_str("OK 0\r\n");
  this->write_str((char *) buffer);
  this->write_str("\r\n");

  std::string str = "";

  for (int x = 0; x < len; x++) {
    str = str + ((char *) buffer)[x];
  }

  std::stringstream ss;

  // for (i = streams.begin(); i != streams.end();) {
  //   (*i)->AddData((char *) buffer);
  // }

  handle_string_command_(str);

  // std::cout << str << " cout \r\n";

  //	fd = open(dev, O_RDWR);
  // ret = ConfigSerial(fd, speed, lflag);
  // if(ret) {
  // 	ESP_LOGW("OpenSerial ConfigSerial %s %d 0x%x 0x%x failed %d", dev, fd, speed, lflag, ret);
  // 	return ret;
  // } else {
  // 	return fd;
  // }
}

void WWRESIComponent::handle_string_command_(std::string str) {
  // std::cout << "handle_string_command_: ";
  ESP_LOGD(TAG, "handle_string_command_: %s ", str.c_str());
  // std::cout << str << " \r\n";

  this->write_str((str + " uart\r\n").c_str());
}

// int WWRESIComponent::handleCommand(class Line_Buffer *stream, string &line) {
//   ostringstream ack;
//   unsigned int channels = 0;
//   int unit = 1;
//   char unitch = 0;
//   char *end_ptr;
//   double val;
//   int ret;

//   transform(line.begin(), line.end(), line.begin(), (int (*)(int)) toupper);
//   istringstream input(line);
//   string cmd;
//   string chan;
//   string value;
//   bool do_rst = false;

//   input >> cmd;
//   ESP_LOGD(TAG, "cmd  : %s ", cmd);
//   input >> chan;
//   ESP_LOGD(TAG, "chan : %s ", chan);
//   input >> value;
//   ESP_LOGD(TAG, "value: %s ", value);




//   return 0;

// }

// void WWRESIComponent::writeToAll(const string &str, Line_Buffer::flags dest) {
//   t_Line_Buffer_List::iterator i;
//   for (i = streams.begin(); i != streams.end(); ++i) {
//     if ((*i)->m_flags & dest) {
//       switch ((*i)->m_fd) {
//         case 0:  // uart
//                  // code
//           write_str((str.data()));
//           break;

//         default:
//           break;
//       }

//       //////  write((*i)->m_fd, str.data(), str.length());
//     }
//   }
// }

//---------------------------------------------------------------------------
// Sends a restart and set system running mode to normal
void WWRESIComponent::send_module_restart() { this->wwresi_restart(); }

//---------------------------------------------------------------------------
void WWRESIComponent::wwresi_restart() {
  CmdFrameT cmd_frame;
  // cmd_frame.data_length = 0;
  // cmd_frame.header = CMD_FRAME_HEADER;
  // cmd_frame.command = CMD_RESTART;
  // cmd_frame.footer = CMD_FRAME_FOOTER;
  ESP_LOGD(TAG, "Sending restart command: %2X", cmd_frame.command);
  // this->send_cmd_from_array(cmd_frame);
}

//---------------------------------------------------------------------------
int WWRESIComponent::get_firmware_int_(const char *version_string) { return 0; }

//---------------------------------------------------------------------------
void WWRESIComponent::get_firmware_version_() {
  CmdFrameT cmd_frame;
  // cmd_frame.data_length = 0;
  // cmd_frame.header = CMD_FRAME_HEADER;
  // cmd_frame.command = CMD_READ_VERSION;
  // cmd_frame.footer = CMD_FRAME_FOOTER;

  ESP_LOGD(TAG, "Sending read firmware version command: %2X", cmd_frame.command);
  // this->send_cmd_from_array(cmd_frame);
}

#ifdef USE_NUMBER
//---------------------------------------------------------------------------
void WWRESIComponent::init_config_numbers() {
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

//---------------------------------------------------------------------------
void WWRESIComponent::refresh_config_numbers() {
  this->timeout_number_->publish_state(this->new_config.timeout);
  // this->min_gate_distance_number_->publish_state(this->new_config.min_gate);
  // this->max_gate_distance_number_->publish_state(this->new_config.max_gate);
}

#endif

}  // namespace wwresi
}  // namespace esphome
