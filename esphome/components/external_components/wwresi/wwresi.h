#pragma once

#include <cinttypes>
#include <map>
#include <memory>
#include <set>
#include <vector>
#include <functional>
#include <string>

#include <iostream>
#include <sstream>
#include <list>

#include "esphome/core/defines.h"
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_NUMBER
#include "esphome/components/number/number.h"
#endif
#ifdef USE_SWITCH
#include "esphome/components/switch/switch.h"
#endif
#ifdef USE_BUTTON
#include "esphome/components/button/button.h"
#endif
#ifdef USE_SELECT
#include "esphome/components/select/select.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif
#include "esphome/components/uart/uart.h"
#include "esphome/components/socket/socket.h"
#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/core/helpers.h"

#include "linebuffer.h"

using std::string;  // std::cin, std::cout, std::endl; // C++17 or later

namespace esphome {
namespace wwresi {

static const char *const TAG = "wwresi";

static const uint8_t WW_MR01_TOTAL_GATES = 16;
static const uint16_t FACTORY_TIMEOUT = 120;
static const int FACTORY_RESISTANCE = -1;

static const int CALIBRATE_VERSION_MIN = 154;

static const int S_UART = 0;
static const int S_NET = 1;
static const int S_CAN = 2;

// int fdUdp;
// int fdError;
// int fdIn;

enum {
  RESI_RELAYS_N = 4,

  // RESI_R_MAX = 266665,
  // RESI_DECADE_SINGLE_RESISTOR = 200000,
  // RESI_DECADE_MULT_FACT = 10000,
  
  RESI_R_MAX_M = 2666665,
  RESI_DECADE_SINGLE_RESISTOR_M = 2000000,
  RESI_DECADE_MULT_FACT_M = 100000,

  RESI_RELAYS_BITS = 24,
};

enum e_MODE { REMOTE = 0, LOCAL = 1 };

enum e_CHANNELS {
  CH_R0 = 0,
  CH_R1 = 1,
  CH_R_N = 2,
  CH_DOUT = 2,
  CH_DIN = 3,
  CH_R0V = 4,
  CH_R1V = 5,

  CH_N,
  CH_INVALID = 0x80000000
};


struct WWRESIFlashData {
  char serial_number[8];
  uint8_t eth_ip_address[4];
  uint8_t eth_ip_mask[4];
  uint8_t eth_ip_gateway[4];
} PACKED;


class WWRESIListener {
 public:
  virtual void on_presence(bool presence){};
  virtual void on_distance(uint16_t distance){};
  virtual void on_energy(uint16_t *sensor_energy, size_t size){};
  virtual void on_fw_version(std::string &fw){};
};

class t_CHANNELS;


//--------------------------------------------------------------------------------------------------------------
/// @brief class storing bitmask of active channels (up to 15 bits) in bits 16..30 of 32 bit uint and "virtual channel"
/// SZ value in lower 16 bits
class t_CHANNELS {
 protected:
  unsigned int chn;  /// bits 0..15SZ, bits 16..30 selected channel, bit 31 = INVALID channel
 public:
  t_CHANNELS() : chn(0){};
  t_CHANNELS(unsigned int x) : chn(x){};
  bool operator&(enum e_CHANNELS ch) {
    // cout<<"operator& (enum e_CHANNELS ch),  ch : "<<ch<<"  m_chn : "<<chn<<"\n";
    // cout<<"operator&(enum e_CHANNELS ch) ,  ((0x10000<<ch) & chn) != 0 :  "<<(((0x10000<<ch) & (int)chn) != 0)<<"\n";
    // return ((0x10000<<(int)ch) & chn) != 0;
    return (*this) & (int) ch;
  }
  bool operator&(int ch) {
    // cout<<"operator& (int ch),  ch : "<<ch<<"  m_chn : "<<chn<<"\n";
    // cout<<"operator&(int ch) ,  ((0x10000<<ch) & chn) != 0 :  "<<(((0x10000<<ch) & (int)chn) != 0)<<"\n";
    return ((0x10000 << ch) & chn) != 0;
  }
  int getv() { return chn & 0xffff; }
  t_CHANNELS &operator|(enum e_CHANNELS ch) { return *this | (int) ch; }
  t_CHANNELS &operator|(int ch) {
    chn |= (0x10000 << ch);
    return *this;
  }
  /** set SZ value (16 bit)
  @param v SZ value to be stored in this class
  @return class reference
  */
  t_CHANNELS &setv(int v) {
    chn &= ~0xffff;
    chn |= v & 0xffff;
    return *this;
  }
  operator int() { return chn; }
};


class WWRESIComponent : public Component, public uart::UARTDevice {
 public:
  WWRESIComponent();
  ~WWRESIComponent();

  int debug = 1;  // 1;

  /** Where the component's initialization should happen.
   *
   * Analogous to Arduino's setup(). This method is guaranteed to only be called once.
   * Defaults to doing nothing.
   */
  void setup() override;

  /** prints the user configuration.
   *
   *
   */
  void dump_config() override;

  /** This method will be called repeatedly.
   *
   * Analogous to Arduino's loop(). setup() is guaranteed to be called before this.
   * Defaults to doing nothing.
   */
  void loop() override;

  // Custom methods

  // #ifdef USE_SELECT
  //   void set_operating_mode_select(select::Select *selector) { this->operating_selector_ = selector; };
  // #endif

#ifdef USE_NUMBER
  void set_timeout_number(number::Number *number) { this->timeout_number_ = number; };
  //   void set_gate_select_number(number::Number *number) { this->gate_select_number_ = number; };
  void set_resistance_number(number::Number *number) { this->resistance_number_ = number; };

  // void set_max_distance_number(number::Number *number) { this->max_distance_number_ = number; };
  // void set_gate_move_sensitivity_factor_number(number::Number *number) {
  //   this->gate_move_sensitivity_factor_number_ = number;
  // };
  // void set_gate_still_sensitivity_factor_number(number::Number *number) {
  //   this->gate_still_sensitivity_factor_number_ = number;
  // };
  // void set_gate_still_threshold_numbers(int gate, number::Number *n) {
  //   this->gate_still_threshold_numbers_[gate] = n;
  //   };
  //    void set_gate_move_threshold_numbers(int gate, number::Number *n) {
  // this->gate_move_threshold_numbers_[gate] = n;
  // };
  // bool is_gate_select() {
  //    return gate_select_number_ != nullptr;
  // };
  // uint8_t get_gate_select_value() { return static_cast<uint8_t>(this->gate_select_number_->state); };

  int get_resistance_value() { return resistance_number_->state; };

  // float get_max_distance_value() { return  max_distance_number_->state; };
  // void publish_gate_move_threshold(uint8_t gate) {
  //   // With gate_select we only use 1 number pointer, thus we hard code [0]
  //   this->gate_move_threshold_numbers_[0]->publish_state(this->new_config.move_thresh[gate]);
  // };
  // void publish_gate_still_threshold(uint8_t gate) {
  //   this->gate_still_threshold_numbers_[0]->publish_state(this->new_config.still_thresh[gate]);
  // };

  void init_config_numbers();
  void refresh_config_numbers();
#endif

#ifdef USE_BUTTON
  void set_apply_config_button(button::Button *button) { this->apply_config_button_ = button; };
  void set_revert_config_button(button::Button *button) { this->revert_config_button_ = button; };
  void set_restart_module_button(button::Button *button) { this->restart_module_button_ = button; };
  void set_factory_reset_button(button::Button *button) { this->factory_reset_button_ = button; };
#endif

  void register_listener(WWRESIListener *listener) { this->listeners_.push_back(listener); }

  //  struct CmdFrameT {
  //    uint32_t header{0};
  //    uint16_t length{0};
  //    uint16_t command{0};
  //    uint8_t data[18];
  //    uint16_t data_length{0};
  //    uint32_t footer{0};
  //  };

  struct RegConfigT {
    int channel{0};
    int resistance{0};
    // uint16_t max_gate{0};
    uint16_t timeout{0};
    //   uint32_t move_thresh[WW_MR01_TOTAL_GATES];
    //   uint32_t still_thresh[WW_MR01_TOTAL_GATES];
  };

  void send_module_restart();

  void restart_module_action();
  void apply_config_action();
  void factory_reset_action();
  void revert_config_action();

  float get_setup_priority() const override;
  //   int send_cmd_from_array(CmdFrameT cmd_frame);
  //   void report_gate_data();
  //   void handle_cmd_error(uint8_t error);
  //   void set_operating_mode(const std::string &state);
  //   void auto_calibrate_sensitivity();
  //   void update_radar_data(uint16_t const *gate_energy, uint8_t sample_number);
  //   uint8_t calc_checksum(void *data, size_t size);

  RegConfigT current_config;
  RegConfigT new_config;
  //   int32_t last_periodic_millis = millis();
  //   int32_t report_periodic_millis = millis();
  //   int32_t monitor_periodic_millis = millis();
  //   int32_t last_normal_periodic_millis = millis();
  //   bool output_energy_state{false};
  //   uint8_t current_operating_mode{OP_NORMAL_MODE};
  //   uint16_t radar_data[WW_MR01_TOTAL_GATES][CALIBRATE_SAMPLES];
  //   uint16_t gate_avg[WW_MR01_TOTAL_GATES];
  //   uint16_t gate_peak[WW_MR01_TOTAL_GATES];
  //   uint8_t sample_number_counter{0};
  //   uint16_t total_sample_number_counter{0};
  //   float gate_move_sensitivity_factor{0.5};
  //   float gate_still_sensitivity_factor{0.5};
  // #ifdef USE_SELECT
  //   select::Select *operating_selector_{nullptr};
  // #endif

#ifdef USE_BUTTON
  button::Button *apply_config_button_{nullptr};
  button::Button *revert_config_button_{nullptr};
  button::Button *restart_module_button_{nullptr};
  button::Button *factory_reset_button_{nullptr};
#endif

  //   void set_min_max_distances_timeout(uint32_t max_gate_distance, uint32_t min_gate_distance, uint32_t timeout);
  //   void set_gate_threshold(uint8_t gate);
  //   void set_reg_value(uint16_t reg, uint16_t value);
  //   uint8_t set_config_mode(bool enable);
  //   void set_system_mode(uint16_t mode);
  void wwresi_restart();

  void set_resistance_value();


 protected:
  std::unique_ptr<socket::Socket> socket_;

  //   struct CmdReplyT {
  //     uint8_t command;
  //     uint8_t status;
  //     uint32_t data[4];
  //     uint8_t length;
  //     uint16_t error;
  //     volatile bool ack;
  //   };

  int get_firmware_int_(const char *version_string);
  void get_firmware_version_();
  //   int get_gate_threshold_(uint8_t gate);
  //   void get_reg_value_(uint16_t reg);
  //   int get_min_max_distances_timeout_();
  //   uint16_t get_mode_() { return this->system_mode_; };
  //   void set_mode_(uint16_t mode) { this->system_mode_ = mode; };
  //   bool get_presence_() { return this->presence_; };
  //   void set_presence_(bool presence) { this->presence_ = presence; };
  //   uint16_t get_distance_() { return this->distance_; };
  //   void set_distance_(uint16_t distance) { this->distance_ = distance; };
  void get_cmd_new();
  bool get_cmd_active_() { return this->cmd_active_; };
  void set_cmd_active_(bool active) { this->cmd_active_ = active; };
  //   void handle_simple_mode_(const uint8_t *inbuf, int len);
  //   void handle_energy_mode_(uint8_t *buffer, int len);
  //   void handle_ack_data_(uint8_t *buffer, int len);

  //   void set_calibration_(bool state) { this->calibration_ = state; };
  //   bool get_calibration_() { return this->calibration_; };

#ifdef USE_NUMBER
  number::Number *timeout_number_{nullptr};
  //   number::Number *gate_select_number_{nullptr};
  number::Number *resistance_number_{nullptr};
  //   number::Number *max_distance_number_{nullptr};
  //   number::Number *gate_move_sensitivity_factor_number_{nullptr};
  //   number::Number *gate_still_sensitivity_factor_number_{nullptr};
  //   std::vector<number::Number *> gate_still_threshold_numbers_ = std::vector<number::Number *>(16);
  //   std::vector<number::Number *> gate_move_threshold_numbers_ = std::vector<number::Number *>(16);
#endif

  //   uint16_t gate_energy_[WW_MR01_TOTAL_GATES];
  //   CmdReplyT cmd_reply_;
  //   uint32_t max_distance_gate_;
  //   uint32_t min_distance_gate_;
  //   uint16_t system_mode_{CMD_SYSTEM_MODE_ENERGY};
  bool cmd_active_{false};
  char firmware_ver_[8]{"v0.0.1"};
  //   bool presence_{false};
  //   bool calibration_{false};
  //   uint16_t distance_{0};
  //   uint8_t config_checksum_{0};
  std::vector<WWRESIListener *> listeners_{};

  t_Linebuffer_List streams;

  void get_read_DIN();
  void readline_(int rx_data, uint8_t *buffer, int len);

  void addCommandToStream_(int streamNr, uint8_t *buffer, int len);
  void handle_string_command_(std::string str);

  t_CHANNELS parse_channel(string &chan) ;
  int handleCommand(int streamNr, class Linebuffer *stream, string &line);

  void writeToAll(const string &str, Linebuffer::flags dest);

  void stateMachine_DIN(void);
  void transition_DIN(int chan, int transition);

  void transition_mode(int chan, e_MODE mode);
  int calculate_relays(int value, unsigned char relays[RESI_RELAYS_N]);
  void set_r_channel(int chan, double r);
  void set_r_channel_number(int chan, double r);
  void set_r_channel_load(int chan, double r);

};

}  // namespace wwresi
}  // namespace esphome
