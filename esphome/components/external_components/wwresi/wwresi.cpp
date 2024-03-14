
#include <unistd.h>
#include <list>
#include <algorithm>
#include "esphome/core/application.h"

#include "linebuffer.h"
#include "wwresi.h"

using std::string;

namespace esphome {
namespace wwresi {

/** global variables */

char version[] = "RESI MR01 V0.1";

//============================================================================================================

#define TRUE true
#define FALSE false
//=================================================CAN
// #define ACK_FIND 0x10
// #define ACK_GET 0x13
// #define ACK_TYPE 0x18
// #define ACK_CAN 0x1f  // for the rest of can-cmd.

// for broadcasting
#define BROAD_CAN_RID 100

// RESI-Serialnumber
#define SN 1234
// RESI-TYP
// #define TYP 0x2
#define DIN_MASK 0xf

#define RESOLUTION 1  // for resolution 1 -> 0x81 for resolution 10 -> 0x91 (Formel in der Spezifikation)

// int debug = 0;
int watchdog = 0;

unsigned long brid = BROAD_CAN_RID;
// var. for RID
unsigned long rid = 100;
// var. for TID
unsigned long tid = 101;
unsigned long serialnr = SN;
unsigned long serial_pcb = 0;

// can mesg. ext
unsigned int ext = 0;
unsigned int can_identifier = (1 << 11) - 1;  // std. 536870911 (29 bits) or 2047(11 bits)
unsigned int can_bitrate = 500;

// todo eso  canmsg_t rx;
// todo eso  canmsg_t tx;

typedef int t_TIME;

enum e_resi_type { RESI_T_K = 0, RESI_T_M = 1, RESI_T_KM = 2 };

int ResiType = RESI_T_KM;

enum { RESI_DOUT_MAX = 15 };

enum {
  RESI_RELAYS_N = 3,
  RESI_R_MAX = 266665,
  RESI_DECADE_SINGLE_RESISTOR = 200000,
  RESI_DECADE_MULT_FACT = 10000,
  RESI_R_MAX_M = 2666665,
  RESI_DECADE_SINGLE_RESISTOR_M = 2000000,
  RESI_DECADE_MULT_FACT_M = 100000,
  RESI_RELAYS_BITS = 20,
};

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

enum e_MODE { REMOTE = 0, LOCAL = 1 };

enum e_CONSTANTS { INDEX_MAX = 256, DEBOUNCE_DIN = 20000 };

e_MODE Mode[CH_R_N];

int LastDIN[CH_R_N];
int ConfirmedDIN[CH_R_N];
t_TIME tLastDINChanged[CH_R_N];

double ValCurrent[CH_N];
double ValSet[CH_R_N];
double ValLoad[CH_N];
double ValVirtual[CH_R_N][INDEX_MAX];
int Index[CH_R_N];

string ValIpAddr = "";
string ValIpNetmask = "";
string ValIpGateway = "";
int ValConfigVersion = -1;

FILE *fLog;
long iBusFlags;
time_t tsStarted;
long tmLastVerified = 0;
long tmNow;  // current time, global, will be updated in many places
long tmLastSentReceived = 0;
struct timeval tvNow;
struct timeval tvSelectTimeout;

/** all t_TIME in units of us */
t_TIME time0;
t_TIME SleepUntil = 0;
t_TIME time_v_changed = 0;

int quiet = 0;

int RecentDIN = -1;  // -1 to force refresh on start
int RecentDOUT = 0;

/** return time in "ticks", not used */
// long tmGet() {
// 	gettimeofday(&tvNow,NULL);
// 	return (tmNow=TM_HZ*(tvNow.tv_sec-tsStarted) + (TM_HZ*tvNow.tv_usec)/1000000);
// #if TM_HZ>2147
// #error when using 32 bit signed int TM_HZ can not be greater than 2147
// #endif
// }

/** microseconds, 32 bit, will wrap around, use only for relative comparison ! */

t_TIME tnow = 0;
/** return time in us from unspecified origin and update tnow */
t_TIME gettime() {
  gettimeofday(&tvNow, NULL);
  return (tnow = tvNow.tv_sec * 1000000 + tvNow.tv_usec);
}

void die(char *str) {
  fprintf(stderr, "exiting: %s, errno %d\n", str, errno);
  exit(10);
}

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

int CH(int ch) { return 0x10000 << ch; }

t_CHANNELS ParseChannel(string &chan) {
  t_CHANNELS channels = 0;
  const char *str = chan.data();
  if (chan.length() == 0) {
    channels | CH_R0 | CH_R1 | CH_DOUT;
  } else if (chan.length() == 1) {
    if (chan[0] == '0') {
      channels | CH_R0;
    } else if (chan == "1") {
      channels | CH_R1;
    } else {
      return CH_INVALID;
    }
  } else if (chan.length() == 2) {
    return CH_INVALID;
  } else {  // min 3 characters
    if (str[1] == 'V') {
      int tmp = atoi(str + 2);
      if (tmp < 0 || tmp >= INDEX_MAX) {
        return CH_INVALID;
      }
      channels.setv(tmp);
      if (str[0] == '0') {
        channels | CH_R0V;
      } else if (str[0] == '1') {
        channels | CH_R1V;
      } else {
        return CH_INVALID;
      }
    } else if (chan == "ALL") {
      channels | CH_R0 | CH_R1;
    } else if (chan == "DOUT") {
      channels | CH_DOUT;
    } else if (chan == "DIN") {
      channels | CH_DIN;
    } else {
      return CH_INVALID;
    }
  }
  return channels;
}

/** function based on PwdOutValue from Pwd-Treiber.txt (Pascal).
  @param[in] value resistance in Ohm, negative means open, maximum ::RESI_R_MAX
  @param[out] relays 3 bytes containing relay settings to get value Ohm
  @return 0 on success, -1 on error
*/
int CalculateRelays(int value, unsigned char relays[RESI_RELAYS_N]) {
  int i;
  int DecadeSingleResistor;
  int DecadeMultFact;
  int BytePos;
  int BitPos;

  if (value < 0) {
    // open
    for (i = 0; i < RESI_RELAYS_N; i++) {
      relays[i] = 0;
    }
    relays[2] = 0x80;
  } else {
    for (i = 0; i < RESI_RELAYS_N; i++) {
      relays[i] = 0;
    }
    if (ResiType & RESI_T_M) {
      // 2.666... MOhm version
      DecadeSingleResistor = RESI_DECADE_SINGLE_RESISTOR_M;
      DecadeMultFact = RESI_DECADE_MULT_FACT_M;
    } else {
      DecadeSingleResistor = RESI_DECADE_SINGLE_RESISTOR;
      DecadeMultFact = RESI_DECADE_MULT_FACT;
    }

    for (i = RESI_RELAYS_BITS; i >= 0; i--) {
      BytePos = i / 8;
      BitPos = i % 8;
      if (i % 4 == 3) {
        DecadeSingleResistor = 8 * DecadeMultFact;
        DecadeMultFact /= 10;
      } else {
        DecadeSingleResistor /= 2;
      }

      if (value >= DecadeSingleResistor) {
        value -= DecadeSingleResistor;
        // each BytePos,BitPos pair occurs only once so XOR works OK
        // original code had OR here
        // but XOR will also work when starting from 0xff
        relays[BytePos] ^= (1 << BitPos);
      }
    }
    if (ResiType & RESI_T_M) {
      unsigned tmp;
      // we have to shift right 4 bits
      tmp = relays[2];
      relays[2] = (relays[1] >> 4) | (relays[2] & 0x10);
      relays[1] = (relays[1] << 4) | (relays[0] >> 4);
      relays[0] = (relays[0] << 4) | (tmp & 0xf);
    }
    // check whether bypass is possible, also for 2.6M
    if (relays[2] == 0 && relays[1] == 0) {
      relays[2] ^= 0x40;
      if ((relays[0] & 0xf0) == 0) {
        relays[2] ^= 0x20;
      }
    }
    // invertion needed for 21 bits
    for (i = 0; i < 2; i++) {
      relays[i] ^= 0xff;
    }
    relays[2] ^= 0x1f;
  }
  return 0;
}

void SetRChannel(int chan, double r) {
  unsigned char relays[RESI_RELAYS_N];
  char val_str[30];
  int ri = (int) round(r);
  CalculateRelays(ri, relays);
  relays[2] ^= 0x80;  // invert bit X24 here instead in FPGA RESI20

  // if(debug>1) {
  // 	cerr << "SetRChannel("<<chan<<","<<ri<<") = " <<hex<< (int)relays[0] <<" "<< (int)relays[1] <<" "<<
  // (int)relays[2]<<"\n";
  // }

  // todo eso
  //  PortWrite(chan?SFMB_15_0_A:SFMA_15_0_A , relays[0] | (relays[1]<<8));
  //  PortWrite(chan?SFMB_23_16_A:SFMA_23_16_A , relays[2]);

  if (ri < 0) {
    // todo eso
    //  Display57C(chan, 5*6, "OPEN        ");
    return;
  }
  if (ri / 1000000) {
    sprintf(val_str, "%1d %03d %03d <", ri / 1000000, (ri / 1000) % 1000, ri % 1000);
  } else if (ri / 1000) {
    sprintf(val_str, "  %3d %03d <", ri / 1000, ri % 1000);
  } else {
    sprintf(val_str, "      %3d <", ri);
  }
  // todo eso
  //  Display57C(chan, 5*6, val_str);
}

void RefreshIndex(void) {
  char buf[20];
  int chan;

  for (chan = 0; chan < CH_R_N; chan++) {
    if (Mode[chan] == LOCAL) {
      sprintf(buf, "%3d", Index[chan]);
      // todo eso			Display57C(chan,17*6, buf);
    } else {
      // todo eso			Display57C(chan,17*6, "   ");
    }
  }
}

void SetDOUT(unsigned long val) {
  // cerr << "SetDOUT("<<val<<")\n";
  // PortWrite(RESIC_DOUT, val);
  // RecentDOUT = val;
  // RefreshLcdDOUT();
}

void TransitionDIN(int chan, int transition) {
  if (Mode[chan] != LOCAL) {
    return;
  }
  // specification rev 0.6 Table 6
  switch (transition) {
    case 0x1:  // increment
      Index[chan] = (Index[chan] + 1) % INDEX_MAX;
      break;
    case 0x2:  // decrement
      Index[chan] = (Index[chan] + INDEX_MAX - 1) % INDEX_MAX;
      break;
    case 0x3:  // reset 00 -> 11
    case 0x7:  // 01 -> 11 (robustness)
    case 0xB:  // 10 -> 11 (robustness)
      Index[chan] = 0;
      break;
    default:
      return;  // no further action
  }

  // we are in LOCAL mode and Index has changed, time to change R as well
  ValCurrent[chan] = ValVirtual[chan][Index[chan]];
  SetRChannel(chan, ValCurrent[chan]);
  RefreshIndex();

  // send ACK message
  std::ostringstream ack;
  ack << "OK\nINDX " << chan << " " << Index[chan] << "\nSET " << chan << " " << ValCurrent[chan] << "\n\r\n";
  // todo eso  writeToAll(ack.str(), Linebuffer::F_ECHO_INDX);
}

void TransitionMODE(int chan, enum e_MODE mode) {
  Index[chan] = 0;
  Mode[chan] = mode;
  if (mode == LOCAL) {
    ValCurrent[chan] = ValVirtual[chan][Index[chan]];
  } else {
    ValCurrent[chan] = ValSet[chan];
  }
  SetRChannel(chan, ValCurrent[chan]);
  RefreshIndex();
}

void StateMachineDIN(void) {
  int i;
  for (i = 0; i < CH_R_N; i++) {
    int din = (RecentDIN >> (2 * i)) & 3;
    if (din != ConfirmedDIN[i]) {
      if (din == LastDIN[i]) {
        if ((tnow - tLastDINChanged[i]) > DEBOUNCE_DIN) {
          // 2 bits FROM + 2 bits TO
          // FROM may be also -1 during program initialization
          TransitionDIN(i, (ConfirmedDIN[i] << 2) | din);
          ConfirmedDIN[i] = din;
        } else {
          // just wait longer
        }
      } else {
        LastDIN[i] = din;
        tLastDINChanged[i] = tnow;
      }
    }
  }
}

void Changed(bool chd) {
  // todo eso
  //  if (chd) {
  //    if (!time_v_changed) {
  //      time_v_changed = tnow | 1;
  //    }
  //  } else {
  //    time_v_changed = 0;
  //  }
}

int Clear(t_CHANNELS ch) {
  int ret = -1;
  // if(debug>2)
  // 	cout<<"clear("<<ch<<")\n";
  if ((ch & CH_R0V) || (ch & CH_R0)) {
    for (int i = 0; i < INDEX_MAX; i++) {
      if (ValVirtual[0][i] != -1)
        Changed(true);
      ValVirtual[0][i] = -1;
    }
    ret = 0;
  }
  if ((ch & CH_R1V) || (ch & CH_R1)) {
    for (int i = 0; i < INDEX_MAX; i++) {
      if (ValVirtual[1][i] != -1)
        Changed(true);
      ValVirtual[1][i] = -1;
    }
    ret = 0;
  }
  // if(ret) {
  // 	WARN("Clear, channel 0x%x is incorrect", ch);
  // 	return ERR_CAN_CHAN;
  // }
  return ret;
}

//=====================================================================================================================

//---------------------------------------------------------------------------------------------------------------------
/** Initialize this wwresi component
 *
 */
WWRESIComponent::WWRESIComponent() {}

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
  LOG_NUMBER(TAG, "  Resistance:", this->resistance_number_);
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

  this->streams.push_back(new Linebuffer(static_cast<int>(0)));  // fd=0 uart
  this->streams.push_back(new Linebuffer(static_cast<int>(1)));  // fd=1 socket
  this->streams.push_back(new Linebuffer(static_cast<int>(2)));  // fd=2 can

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
  this->resistance_number_->state = FACTORY_RESISTANCE;
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
  get_cmd_new();
  get_read_DIN();
}

//---------------------------------------------------------------------------
/// @brief read new command from interfaces uart,eth,can
void WWRESIComponent::get_cmd_new() {
  // If there is a active send command do not process it here, the send command call will handle it.
  if (!get_cmd_active_()) {
    // uart data input
    if (!available())
      return;
    static uint8_t buffer[2048];
    static uint8_t rx_data;
    while (available()) {
      rx_data = this->read();
      this->readline_(rx_data, buffer, sizeof(buffer));
    }

    // eth data input
    // todo eso

    // can data input
    // todo eso
  }
}

//---------------------------------------------------------------------------
/// @brief read new DIN from digital inputs
void WWRESIComponent::get_read_DIN() {
  int ret;
  std::ostringstream ack;
  // todo eso  ret = PortRead(RESIC_INS);
  //  bits [3:0] digital inputs, bit 7 is F_USB_PWR_ON and works, is masked below
  if (ret == RecentDIN) {
    return;
  }
  ack << "OK\nGET DIN " << (ret & DIN_MASK) << "\n\r\n";
  writeToAll(ack.str(), Linebuffer::F_ECHO_DIN);
  ValCurrent[CH_DIN] = RecentDIN = ret;
  // todo eso RefreshLcdDIN();
}

//---------------------------------------------------------------------------
/// @brief put uart char to buffer until CR 0x0a
/// @param rx_data
/// @param buffer
/// @param len
void WWRESIComponent::readline_(int rx_data, uint8_t *buffer, int len) {
  static int pos = 0;

  if (rx_data > 0) {
    // ESP_LOGD(TAG, "rx: %02x", rx_data);
    switch (rx_data) {
      case '\r':  // Ignore LF 0x0a
        break;
      case '\n':  // Return on CR 0x0d
        addCommandToStream_(S_UART, buffer, pos);
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
/// @brief convert uint8_t array to string, add to Stringstream append newline, call handleCommand
/// @param buffer
/// @param len
void WWRESIComponent::addCommandToStream_(int streamNr, uint8_t *buffer, int len) {
  t_Linebuffer_List::iterator it;
  std::string str = "";

  // uint8_t array to str
  for (int x = 0; x < len; x++) {
    str = str + ((char *) buffer)[x];
  }
  // str to stream
  it = streams.begin();
  std::advance(it, streamNr);
  (*it)->AddData(str + "\n");  // newline have to be streamend befor eof()

  str = (*it)->Line;
  if (debug) {
    ESP_LOGD("wwresi", "new cmd  %s from %d", str.c_str(), streamNr);
  }

  if (str.length() == 0){    
    return;
  }

  switch (streamNr) {
    case 0:
      handleCommand(streamNr, (*it), str);
      break;

    case 1:
      handleCommand(streamNr, (*it), str);
      break;

    case 2:
      handleCommand(streamNr, (*it), str);
      break;

    default:
      if (debug) {
        ESP_LOGD("wwresi", "Unknown input stream  %d", streamNr);
      }
      break;
  }
}

//==========================================================================================================

//---------------------------------------------------------------------------
/// @brief run command called from Linebuffer
/// @param stream
/// @param line
/// @return
int WWRESIComponent::handleCommand(int streamNr, class Linebuffer *stream, string &line) {
  char unitch = ' ';
  bool do_modul_restart = false;
  std::ostringstream ack;
  unsigned int channels = 0;
  int unit = 1;
  char *end_ptr;
  double val;
  int ret;

  transform(line.begin(), line.end(), line.begin(), (int (*)(int)) toupper);
  std::istringstream input(line);
  string cmd;
  string chan;
  string value;

  input >> cmd;
  input >> chan;
  input >> value;
  // last char is maybe value unit like Kilo or Mega
  if (value != "") {
    if (!isdigit(value.end()[-1])) {
      unitch = value.end()[-1];
    }
  }
  if (debug) {
    ESP_LOGD("wwresi", "cmd: %s - chan %s - value %s - unit %c", cmd.c_str(), chan.c_str(), value.c_str(), unitch);
  }
  //--- set and load command .........................
  if (cmd == "SET" || cmd == "LOAD") {
    channels = ParseChannel(chan);
    if (channels == CH_INVALID || (channels & CH(CH_DIN))) {
      ack << "ERR " << stream->m_fd << " invalid channel :" << chan << ": " << channels << "\n";
      ack << line << "\n";
      goto error_jump;
    }
    if (value != "") {
      if ((channels & (CH(CH_R0) | CH(CH_R1))) && (channels & CH(CH_DOUT))) {
        // this will never happen in ParseChannel
        ESP_LOGE("wwresi", "HandleCommand ParseChannel problem %s %d", chan.c_str(), channels);
        ack << "ERR " << stream->m_fd << " channel\n";
        ack << line << "\n";
        goto error_jump;
      } else if (channels & (CH(CH_R0) | CH(CH_R1) | CH(CH_R0V) | CH(CH_R1V))) {
        if (unitch == 'K') {
          unit = 1000;
        } else if (unitch == 'M') {
          unit = 1000000;
        } else if (unitch == ' ') {
          unit = 1;
          unitch = 'D';
        } else {
          ack << "ERR fd=" << stream->m_fd << " invalid unit\n";
          ack << line << "\n";
          goto error_jump;
        }
        val = strtod(value.c_str(), &end_ptr) * unit;
        if (debug) {
          ESP_LOGD("wwresi", "val %.0f - value %s - unit %c", val, value.c_str(), unitch);
        }
        if (ResiType & RESI_T_KM) {
          if (val < 0.0) {
            if (val < -1.0001 || val > -0.9999) {
              ack << "ERR " << stream->m_fd << " wrong value\n";
              ack << line << "\n";
              goto error_jump;
            }
          } else if (val < -1.0001 || val > RESI_R_MAX_M + 0.0001 || fabs(round(val) - val) > 0.0001) {
            ack << "ERR " << stream->m_fd << " wrong value\n";
            ack << line << "\n";
            goto error_jump;
          }
        } else if (ResiType & RESI_T_M) {
          if (val < 0.0) {
            if (val < -1.0001 || val > -0.9999) {
              ack << "ERR " << stream->m_fd << " wrong value\n";
              ack << line << "\n";
              goto error_jump;
            }
          } else if (val > RESI_R_MAX_M + 0.0001 || fabs(10 * round(val / 10) - val) > 0.0001) {
            ack << "ERR " << stream->m_fd << " wrong value\n";
            ack << line << "\n";
            goto error_jump;
          }
        } else {
          if (val < -1.0001 || val > RESI_R_MAX + 0.0001 || fabs(round(val) - val) > 0.0001) {
            ack << "ERR " << stream->m_fd << " wrong value\n";
            ack << line << "\n";
            goto error_jump;
          }
        }
      } else if (channels & CH(CH_DOUT)) {
        if (unitch == 'H') {
          val = strtol(value.c_str(), &end_ptr, 16);
        } else if (unitch == 'B') {
          // get binary
          val = strtol(value.c_str(), &end_ptr, 2);
        } else if (unitch == 'D' || isdigit(unitch)) {
          // get decimal
          val = strtol(value.c_str(), &end_ptr, 10);
          unitch = 'D';
        } else {
          ack << "ERR " << stream->m_fd << " invalid unit\n";
          ack << line << "\n";
          goto error_jump;
        }
        if (val < -0.001 || val > 15.001) {
          ack << "ERR " << stream->m_fd << " wrong DOUT value\n";
          ack << line << "\n";
          goto error_jump;
        }
      } else {
        ack << "ERR " << stream->m_fd << " invalid channel " << channels << "\n";
        ack << line << "\n";
        goto error_jump;
      }
      if (end_ptr == value.c_str()) {
        ack << "ERR " << stream->m_fd << " invalid " << unitch << " value " << value << "\n";
        ack << line << "\n";
        goto error_jump;
      }

      for (int i = CH_R0; i < CH_N; i++) {
        if (channels & CH(i)) {
          ValLoad[i] = val;
        }
      }
      // printArr(ValLoad); //temp

    }  // end of if(value)

    ack << "OK " << stream->m_fd << "\n";
    if (cmd == "SET") {
      for (int i = CH_R0; i < CH_N; i++) {
        if (channels & CH(i)) {
          // ack << "cur "<< ValCurrent[i] << " load "<< ValLoad[i]<< " ";
          // ack << "val "<< value << " ";
          switch (i) {
            case CH_R0:
            case CH_R1:
              if (Mode[i] != LOCAL) {
                ValSet[i] = ValLoad[i];
                ValCurrent[i] = ValSet[i];
                SetRChannel(i, ValCurrent[i]);
                ack << "SET " << i << " " << ValCurrent[i] << "\n";
              } else {
                ack << "SET " << i << " " << ValCurrent[i] << "\n";  // CHECKME return ValSet ?
              }
              break;
            case CH_R0V:
            case CH_R1V:
              if (value != "") {
                ValVirtual[i - CH_R0V][channels & 0xff] = ValLoad[i];
                Changed(true);
              }
              ack << "SET " << (i - CH_R0V) << "V" << (channels & 0xff) << " ";
              ack << ValVirtual[i - CH_R0V][channels & 0xff] << "\n";
              break;
            case CH_DOUT:
              SetDOUT((unsigned long) (ValCurrent[i] = ValLoad[i]));
              ack << "SET DOUT " << (unsigned long) (ValCurrent[i]) << "\n";
              break;
            default:
              ack << "ERR " << stream->m_fd << " SET " << i << " impossible\n";
              goto error_jump;
              break;
          }
        }
      }
    } else {
      // --------------------------------------------------LOAD
      for (int i = CH_R0; i < CH_N; i++) {
        if (channels & CH(i)) {
          switch (i) {
            case CH_R0:
            case CH_R1:
              ack << "LOAD " << i << " " << ValLoad[i] << "\n";
              break;
            case CH_DOUT:
              ack << "LOAD DOUT " << (unsigned long) (ValLoad[i]) << "\n";
              break;
            default:
              ack << "ERR LOAD " << i << " impossible\n";
              break;
          }
        }
      }
      // printACK(ack.str()); //temp
    }
  } else if (cmd == "CLEAR") {
    channels = ParseChannel(chan);
    if (channels == CH_INVALID) {
      ack << "ERR " << stream->m_fd << " invalid channel CLEAR\n";
      ack << line << "\n";
      goto error_jump;
    }
    ret = Clear(channels);  // Changed(true) called inside
    if (ret) {
      ack << "ERR " << stream->m_fd << " error CLEAR\n";
      ack << line << "\n";
      goto error_jump;
    }
    ack << "OK " << stream->m_fd << "\nCLEAR " << chan << "\n";
  } else if (cmd == "GET") {
    channels = ParseChannel(chan);
    if (channels == CH_INVALID) {
      ack << "ERR " << stream->m_fd << " invalid channel GET\n";
      ack << line << "\n";
      goto error_jump;
    }
    ack << "OK " << stream->m_fd << "\n";
    for (int i = CH_R0; i < CH_N; i++) {
      if (channels & CH(i)) {
        switch (i) {
          case CH_R0:
          case CH_R1:
            ack << "SET " << i << " " << ValCurrent[i] << "\n";
            // todo eso            cout << "val : " << ValCurrent[i] << "\n";
            break;
          case CH_R0V:
          case CH_R1V:
            ack << "SET " << i - CH_R0V << "V" << (channels & 0xff) << " " << ValVirtual[i - CH_R0V][channels & 0xff]
                << "\n";
            break;
          case CH_DOUT:
            ack << "SET DOUT " << (unsigned long) (ValCurrent[i]) << "\n";
            break;
          case CH_DIN:
            ack << "GET DIN " << ((unsigned long) (ValCurrent[i]) & DIN_MASK) << "\n";
            break;
          default:
            break;
        }
      }
    }
  } else if (cmd == "MODE") {
    if (chan == "LOCAL") {
      Mode[CH_R0] = Mode[CH_R1] = LOCAL;
      TransitionMODE(CH_R0, Mode[CH_R0]);
      TransitionMODE(CH_R1, Mode[CH_R1]);
    } else if (chan == "REMOTE") {
      Mode[CH_R0] = Mode[CH_R1] = REMOTE;
      TransitionMODE(CH_R0, Mode[CH_R0]);
      TransitionMODE(CH_R1, Mode[CH_R1]);
    } else if (chan == "") {
    } else {
      ack << "ERR " << stream->m_fd << " invalid MODE :" << chan << ":\n";
      ack << line << "\n";
      goto error_jump;
    }
    ack << "OK " << stream->m_fd << "\n";
    ack << "MODE ";
    ack << ((Mode[CH_R0] == LOCAL) ? "LOCAL" : "REMOTE");
    ack << "\n";
    // CHANGE removed in V3.3 as not in spec FL01DE0000-00088-19/4
    // ack << "SET 0 "<< ValCurrent[0]<<"\n";
    // ack << "SET 1 "<< ValCurrent[1]<<"\n";
  } else if (cmd == "IP") {
    string gateway;
    input >> gateway;

    if (chan.length()) {
      ValIpAddr = chan;
    } else {
      // no error, use "IP" without parameters to query things
      //			ack << "ERR "<< stream->m_fd<<"\n";
      //			ack << line << "\n";
      //			goto error_jump;
    }
    if (value.length())
      ValIpNetmask = value;
    if (gateway.length())
      ValIpGateway = gateway;
    if (ValIpAddr == "0") {
      ValIpAddr = "0.0.0.0";
    }

    Changed(true);
    ack << "OK " << stream->m_fd << "\n";
    ack << "IP " << ValIpAddr << " " << ValIpNetmask << " " << ValIpGateway << "\n";
  } else if (cmd == "RST" || cmd == "QUIT" || cmd == "EXIT") {
    ack << "OK " << stream->m_fd << "\nRST\n";
    do_modul_restart = true;
  } else if (cmd == "ECHO") {
    val = strtol(chan.c_str(), &end_ptr, 10);
    ack << "OK " << stream->m_fd << "\nECHO ";
    stream->m_flags = val;
    ack << stream->m_flags << "\n";
  } else if (cmd == "TYPE") {
    ack << "OK " << stream->m_fd << "\nTYPE ";
    if (ResiType & RESI_T_M) {
      ack << "RESI 3 2.6M\n";
      ack << "RMIN 0\nRMAX " << RESI_R_MAX_M << "\n";
      ack << "RSTEP 10\nROPEN -1\nINP_N 4\nOUT_N 4\nVIRT_N 256\n";
    } else {
      ack << "RESI 2 266K\n";
      ack << "RMIN 0\nRMAX " << RESI_R_MAX << "\n";
      ack << "RSTEP 1\nROPEN -1\nINP_N 4\nOUT_N 4\nVIRT_N 256\n";
    }
    ack << "SW " << version << "\n";
    ack << "SN " << serialnr << "\n";
  } else if (cmd == "CFRAME") {
    string ident_length = chan;
    if (ident_length == "29") {
      ext = 1;
      can_identifier = 536870911;
      Changed(true);
    } else if (ident_length == "11") {
      ext = 0;
      can_identifier = 2047;
      Changed(true);
    } else if (ident_length != "") {
      ack << "ERR " << stream->m_fd << " wrong parameter " << chan << "\n";
      ack << line << "\n";
      goto error_jump;
    }
    ack << "OK " << stream->m_fd << "\n";
    ack << "CFRAME " << (ext ? "29" : "11") << "\n";
  } else if (cmd == "CBAUDRATE") {
    int ret;
    if (chan != "") {
      int n = atoi(chan.c_str());
      if (n > 0) {
        // todo eso        ret = set_bitrate(can_fd, n);
      } else {
        ret = -1;
      }
      if (ret != 0) {
        ack << "ERR " << stream->m_fd << " wrong parameter " << chan << "\n";
        ack << line << "\n";
        goto error_jump;
      }
      can_bitrate = n;
      Changed(true);
    }
    ack << "OK " << stream->m_fd << "\n";
    ack << "CBAUDRATE " << can_bitrate << "\n";
    // Changed(true);
  } else if (cmd == "CRID") {
    if (chan != "") {
      int getRID = atoi(chan.c_str());
      if (getRID < 0 || getRID > can_identifier) {
        ack << "ERR " << stream->m_fd << " wrong parameter " << chan << "\n";
        ack << line << "\n";
        goto error_jump;
      }
      rid = getRID;
      Changed(true);
    }
    ack << "OK " << stream->m_fd << "\n";
    ack << "CRID " << rid << "\n";
  } else if (cmd == "CBRID") {
    if (chan != "") {
      int getBRID = atoi(chan.c_str());
      if (getBRID < 0 || getBRID > can_identifier) {
        ack << "ERR " << stream->m_fd << " wrong parameter " << chan << "\n";
        ack << line << "\n";
        goto error_jump;
      }
      brid = getBRID;
      Changed(true);
    }
    ack << "OK " << stream->m_fd << "\n";
    ack << "CBRID " << brid << "\n";
  } else if (cmd == "CTID") {
    if (chan != "") {  // chan here is ID
      int getTID = atoi(chan.c_str());
      if (getTID < 0 || getTID > can_identifier) {
        printf("can_identifier: %d\n", can_identifier);
        ack << "ERR " << stream->m_fd << " wrong parameter " << value << "\n";
        ack << line << "\n";
        goto error_jump;
      }
      tid = getTID;
      Changed(true);
    }
    ack << "OK " << stream->m_fd << "\n";
    ack << "CTID " << tid << "\n";
  } else if (cmd == "VERSION") {
    // ignore
  } else if (cmd == "CONFIG") {
    // TODO switch to quiet (no-ack) mode
  } else if (cmd == "END") {
    // TODO switch to normal ACK mode
  } else if (cmd == "SAVE") {
    // todo eso    WriteConfig();
    // todo eso    SaveConfig(0);

    ack << "OK " << stream->m_fd << "\n";
    ack << "SAVE\n";
  } else if (cmd[0] == '#') {
    // comment line, ignore
  } else if (cmd.length() == 0) {
    // empty line, ignore
  } else {
    ack << "ERR " << stream->m_fd << " unknown command\n";
    ack << line << "\n";
  }

error_jump:
  ack << "\r\n";
  //	cerr << ack.str();
  if (debug) {
    ESP_LOGD(TAG, "ack: %s", ack.str().c_str());
  }

  writeToAll(ack.str(), Linebuffer::F_ECHO_OTHER);

  // eso check it ??????

  // // if F_ECHO_OTHER flag is not there, send ACK to this stream anyway
  // if (!(stream->m_flags & Linebuffer::F_ECHO_OTHER)) {
  //   ret = write(stream->m_fd, ack.str().data(), ack.str().length());

  // eso check it ??????

  if (do_modul_restart) {
    this->send_module_restart();
    // this function will not exit, therefore it is done after ACK is sent
  }
  return 0;
}

void WWRESIComponent::writeToAll(const string &str, Linebuffer::flags dest) {
  t_Linebuffer_List::iterator i;
  for (i = streams.begin(); i != streams.end(); ++i) {
    // if ((*i)->m_flags & dest) {
    switch ((*i)->m_fd) {
      case 0:  // uart
        // code
        write_str((str.data()));
        break;

      default:
        break;
    }

    //////  write((*i)->m_fd, str.data(), str.length());
    //}
  }
}

//---------------------------------------------------------------------------
// Sends a restart and set system running mode to normal
void WWRESIComponent::send_module_restart() { this->wwresi_restart(); }

//---------------------------------------------------------------------------
void WWRESIComponent::wwresi_restart() {
  t_Linebuffer_List::iterator i;
  ESP_LOGD(TAG, "Sending restart command");

  if (time_v_changed) {
    // todo eso		WriteConfig();
    // todo eso		SaveConfig(0);
    Changed(false);
  }

  // todo eso	LcdClear();

  for (i = this->streams.begin(); i != this->streams.end(); ++i) {
    close((*i)->m_fd);
  }

  delay(100);  // NOLINT
  App.safe_reboot();
}

//---------------------------------------------------------------------------
int WWRESIComponent::get_firmware_int_(const char *version_string) { return 0; }

//---------------------------------------------------------------------------
void WWRESIComponent::get_firmware_version_() {
  // CmdFrameT cmd_frame;
  //  cmd_frame.data_length = 0;
  //  cmd_frame.header = CMD_FRAME_HEADER;
  //  cmd_frame.command = CMD_READ_VERSION;
  //  cmd_frame.footer = CMD_FRAME_FOOTER;

  // ESP_LOGD(TAG, "Sending read firmware version command: %2X", cmd_frame.command);
  //  this->send_cmd_from_array(cmd_frame);
}

#ifdef USE_NUMBER
//---------------------------------------------------------------------------
void WWRESIComponent::init_config_numbers() {
  if (this->timeout_number_ != nullptr)
    this->timeout_number_->publish_state(static_cast<uint16_t>(this->current_config.timeout));
  // if (this->gate_select_number_ != nullptr)
  //   this->gate_select_number_->publish_state(0);
  if (this->resistance_number_ != nullptr)
     this->resistance_number_->publish_state(static_cast<int>(this->current_config.resistance));
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
  this->resistance_number_->publish_state(this->new_config.resistance);

  // this->max_gate_distance_number_->publish_state(this->new_config.max_gate);
}

#endif

}  // namespace wwresi
}  // namespace esphome
