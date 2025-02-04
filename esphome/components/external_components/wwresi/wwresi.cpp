#include "wwresi.h"
#include <cerrno>
#include "linebuffer.h"
#include "esphome/components/network/util.h"
#include "esphome/core/application.h"
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/core/util.h"
#include "esphome/core/version.h"
#include "esphome/components/md5/md5.h"
#include "esphome/components/ethernet/ethernet_component.h"

#include <algorithm>

using std::string;

namespace esphome {
namespace wwresi {

static const char *const TAG = "wwresi";

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
unsigned int can_bitrate = 500;               // kbit

// todo eso  canmsg_t rx;
// todo eso  canmsg_t tx;

typedef int t_TIME;

enum e_resi_type { RESI_T_K = 0, RESI_T_M = 1, RESI_T_KM = 2 };

int ResiType = RESI_T_KM;

enum { RESI_DOUT_MAX = 15 };
double ValCurrent[CH_N];
double ValSet[CH_R_N];
double ValLoad[CH_N];
double ValVirtual[CH_R_N][INDEX_MAX];

e_MODE Mode[CH_R_N];

int LastDIN[CH_R_N];
int ConfirmedDIN[CH_R_N];
t_TIME tLastDINChanged[CH_R_N];

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

int CH(int ch) { return 0x10000 << ch; }

void RefreshIndex(void) {
  char buf[20];
  int chan;

  for (chan = 0; chan < CH_R_N; chan++) {
    if (Mode[chan] == e_LOCAL) {
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
  // if ((ch & CH_R1V) || (ch & CH_R1)) {
  //   for (int i = 0; i < INDEX_MAX; i++) {
  //     if (ValVirtual[1][i] != -1)
  //       Changed(true);
  //     ValVirtual[1][i] = -1;
  //   }
  //   ret = 0;
  // }
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
WWRESIComponent::WWRESIComponent() {
  this->socket_ = nullptr;
  //  this->client_ = nullptr;
}

//---------------------------------------------------------------------------
/** Free all data:
 * socket_
 */
WWRESIComponent::~WWRESIComponent() {
  if (this->socket_) {
    this->socket_->close();
    this->socket_ = nullptr;
  }
  // if (this->client_) {
  //   this->client_->close();
  //   this->client_ = nullptr;
  // }
}

//---------------------------------------------------------------------------
/** Where the component's initialization should happen.
 *
 * Analog to Arduino's setup(). This method is guaranteed to only be called once.
 * Defaults to doing nothing.
 */
void WWRESIComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Wwresi '%s'...", this->resistance_number_->get_name().c_str());

  config_read_nvs_();

  // ----------------------------------------SOCKET TCP
  ESP_LOGCONFIG(TAG, "  Config net...");

  ESP_LOGCONFIG(TAG, "  Set buffer to %d", this->buf_size_);

  this->buf_ = std::unique_ptr<uint8_t[]>{new uint8_t[this->buf_size_]};

  struct sockaddr_storage bind_addr;
  socklen_t bind_addrlen =
      socket::set_sockaddr_any(reinterpret_cast<struct sockaddr *>(&bind_addr), sizeof(bind_addr), this->port_);
  if (bind_addrlen == 0) {
    ESP_LOGW(TAG, "  Could not set sockaddr.");
    this->mark_failed();
    return;
  }
  this->socket_ = socket::socket_ip(SOCK_STREAM, PF_INET);
  if (this->socket_ == nullptr) {
    ESP_LOGW(TAG, "  Could not create socket.");
    this->mark_failed();
    return;
  }
  int enable = 1;
  int err = this->socket_->setsockopt(SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int));
  if (err != 0) {
    ESP_LOGW(TAG, "  Socket unable to set reuseaddr: errno %d", err);
    // we can still continue
  }
  err = this->socket_->setblocking(false);
  if (err != 0) {
    ESP_LOGW(TAG, "  Socket unable to set nonblocking mode: errno %d", err);
    this->mark_failed();
    return;
  }
  err = this->socket_->bind(reinterpret_cast<struct sockaddr *>(&bind_addr), bind_addrlen);
  if (err != 0) {
    ESP_LOGW(TAG, "  Socket unable to bind: errno %d", errno);
    this->mark_failed();  // Mark this component as failed. Any future timeouts/intervals/setup/loop will no longer be
                          // // called.
    return;
  }
  err = this->socket_->listen(4);
  if (err != 0) {
    ESP_LOGW(TAG, "  Socket unable to listen: errno %d", errno);
    this->mark_failed();
    return;
  }

  this->publish_sensor();

 
  ESP_LOGCONFIG(TAG, "  Done config net...");

  // ----------------------------------------

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

  // memcpy(&this->new_config, &this->current_config, sizeof(this->current_config));
  //   if (get_firmware_int_(ld2420_firmware_ver_) < CALIBRATE_VERSION_MIN) {
  //     this->set_operating_mode(OP_SIMPLE_MODE_STRING);
  //     this->operating_selector_->publish_state(OP_SIMPLE_MODE_STRING);
  //     this->set_mode_(CMD_SYSTEM_MODE_SIMPLE);
  //     ESP_LOGW(TAG, "LD2420 Frimware Version %s and older are only supported in Simple Mode", ld2420_firmware_ver_);
  //   } else {
  //     this->set_mode_(CMD_SYSTEM_MODE_ENERGY);
  //     this->operating_selector_->publish_state(OP_NORMAL_MODE_STRING);
  //   }

  //   this->set_system_mode(this->system_mode_);
  //   this->set_config_mode(false);

  this->streams.push_back(new Linebuffer(static_cast<int>(0)));  // fd=0 uart
  this->streams.push_back(new Linebuffer(static_cast<int>(1)));  // fd=1 socket
  this->streams.push_back(new Linebuffer(static_cast<int>(2)));  // fd=2 can

  ESP_LOGCONFIG(TAG, "  Setup complete.");
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

  config_write_nvs_();
}

//---------------------------------------------------------------------------
/// @brief prints the user configuration.
void WWRESIComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "WWRESI:");
  ESP_LOGCONFIG(TAG, "  Address: %s:%u", network::get_use_address().c_str(), this->port_);
  ESP_LOGCONFIG(TAG, "  IP Address: %s", network::get_ip_addresses()[0].str().c_str());
  ESP_LOGCONFIG(TAG, "  IP Address: %s", network::get_ip_addresses()[1].str().c_str());
  ESP_LOGCONFIG(TAG, "  IP Address: %s", network::get_ip_addresses()[2].str().c_str());
  ESP_LOGCONFIG(TAG, "  IP Address: %s", network::get_ip_addresses()[3].str().c_str());
  ESP_LOGCONFIG(TAG, "  IP Address: %s", network::get_ip_addresses()[4].str().c_str());
  ESP_LOGCONFIG(TAG, "  Hostname: '%s'", App.get_name().c_str());

  ESP_LOGCONFIG(TAG, "  Firmware Version : %7s", this->firmware_ver_);
  ESP_LOGCONFIG(TAG, "  Serial Number: ");

#ifdef USE_NUMBER
  LOG_NUMBER("", "  Timeout:", this->timeout_number_);
  // LOG_NUMBER(TAG, "  Gate Max Distance:", this->max_gate_distance_number_);
  LOG_NUMBER("", "  Resistance:", this->resistance_number_);
  // LOG_NUMBER(TAG, "  Gate Select:", this->gate_select_number_);
  // for (uint8_t gate = 0; gate < LD2420_TOTAL_GATES; gate++) {
  //   LOG_NUMBER(TAG, "  Gate Move Threshold:", this->gate_move_threshold_numbers_[gate]);
  //   LOG_NUMBER(TAG, "  Gate Still Threshold::", this->gate_still_threshold_numbers_[gate]);
  // }
#endif

#ifdef USE_BUTTON
  LOG_BUTTON("", "  Apply Config:", this->apply_config_button_);
  LOG_BUTTON("", "  Revert Edits:", this->revert_config_button_);
  LOG_BUTTON("", "  Factory Reset:", this->factory_reset_button_);
  LOG_BUTTON("", "  Restart Module:", this->restart_module_button_);
#endif

#ifdef USE_SELECT
  // LOG_SELECT(TAG, "  Operating Mode", this->operating_selector_);
#endif
#ifdef USE_BINARY_SENSOR
    LOG_BINARY_SENSOR("  ", "Connected:", this->connected_sensor_);
#endif
#ifdef USE_SENSOR
    LOG_SENSOR("  ", "Connection count:", this->connection_count_sensor_);
#endif


  if (this->get_firmware_int_(firmware_ver_) < CALIBRATE_VERSION_MIN) {
    ESP_LOGW(TAG, "WW_MR01 Firmware Version %s and older are only supported in Simple Mode", firmware_ver_);
  }
}

void WWRESIComponent::on_shutdown() {
  for (Client &client : this->clients_)
    client.socket->shutdown(SHUT_RDWR);
}

void WWRESIComponent::publish_sensor() {
  #ifdef USE_BINARY_SENSOR
    if (this->connected_sensor_)
      this->connected_sensor_->publish_state(this->clients_.size() > 0);
  #endif
  #ifdef USE_SENSOR
    if (this->connection_count_sensor_)
      this->connection_count_sensor_->publish_state(this->clients_.size());
  #endif
}

//---------------------------------------------------------------------------
/// @brief Write TCP port
/// @param port
void WWRESIComponent::set_port(uint16_t port) { this->port_ = port; }

//---------------------------------------------------------------------------
/// @brief Read TCP port
/// @return
uint16_t WWRESIComponent::get_port() const { return this->port_; }

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
#endif
}

//---------------------------------------------------------------------------
void WWRESIComponent::restart_module_action() {
  this->restart();
  // delay_microseconds_safe(45000);
  // this->set_config_mode(true);
  // this->set_system_mode(system_mode_);
  // this->set_config_mode(false);
}

//---------------------------------------------------------------------------
void WWRESIComponent::revert_config_action() {
#ifdef USE_NUMBER
  this->init_config_numbers();
#endif
}

//---------------------------------------------------------------------------
/// @brief read new command from interfaces uart,eth,can
void WWRESIComponent::get_cmd_new() {
  // If there is a active send command do not process it here, the send command call will handle it.

  this->handle_uart_();
  this->handle_net_();

  // can data input
  // todo eso
}

//---------------------------------------------------------------------------
/// @brief
void WWRESIComponent::handle_uart_() {
  if (!get_cmd_active_()) {
    // uart data input
    if (available()) {
      static uint8_t buffer[2048];
      static uint8_t rx_data;
      while (available()) {
        rx_data = this->read();
        this->readline_(S_UART, rx_data, buffer, sizeof(buffer));
      }
    }
  }
}

//---------------------------------------------------------------------------
/// @brief
void WWRESIComponent::handle_net_() {
  this->so_accept();
  this->so_cleanup();
  this->so_flush();
  this->so_write();
  this->so_cleanup();

  // if (client_ == nullptr) {
  //   struct sockaddr_storage source_addr;
  //   socklen_t addr_len = sizeof(source_addr);
  //   client_ = socket_->accept((struct sockaddr *) &source_addr, &addr_len);
  //   if (client_ == nullptr)
  //     return;

  //   ESP_LOGD(TAG, "Socket client accept: %s", client_->getpeername().c_str());

  //   int i = 1;
  //   int err = client_->setsockopt(SOL_SOCKET, SO_KEEPALIVE, (char *) &i, sizeof(int));
  //   if (err != 0) {
  //     ESP_LOGW(TAG, "Socket could not enable keepalive, errno: %d", errno);
  //   }

  //   int enable = 1;
  //   err = client_->setsockopt(IPPROTO_TCP, TCP_NODELAY, &enable, sizeof(int));
  //   if (err != 0) {
  //     ESP_LOGW(TAG, "Socket could not enable tcp nodelay, errno: %d", errno);
  //   }

  //   enable = 1;
  //   err = socket_->setsockopt(SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int));
  //   if (err != 0) {
  //     ESP_LOGW(TAG, "  Socket unable to set reuseaddr: errno %d", err);
  //     // we can still continue
  //   }

  //   ESP_LOGW(TAG, "setsockopt 1 , errno: %d", err);

  //   err = client_->setblocking(FALSE);
  //   if (err != 0) {
  //     ESP_LOGW(TAG, "setblocking could not set , errno: %d", errno);
  //     client_ = nullptr;
  //   }
  // }

  // if (!network::is_connected) {
  //   ESP_LOGD(TAG, "network disconnect");
  //   client_ = nullptr;
  //   return;
  // }

  // if (!ethernet::global_eth_component->is_connected()) {
  //   ESP_LOGD(TAG, "ethernet disconnect");
  //   client_ = nullptr;
  //   return;
  // }

  // if (client_ == nullptr) {
  //   ESP_LOGD(TAG, "Socket client null");
  //   return;
  // }

  // // eth data input
  // static uint8_t eth_buf[1460];
  // // cmd buffer
  // static uint8_t buffer[2048];
  // static uint8_t rx_data;

  // ssize_t len = this->client_->read(eth_buf, sizeof(eth_buf));
  // // ssize_t len = this->client_->read(buf, 1);
  // if (len > 0) {
  //   if (debug) {
  //     (TAG, "Socket read: len %d", len);
  //   }

  //   for (int i = 0; i < len; i++) {
  //     if (debug) {
  //       ESP_LOGD(TAG, "Socket read: %d - %c", eth_buf[i], ((char *) eth_buf)[i]);
  //     }

  //     rx_data = eth_buf[i];
  //     this->readline_(S_NET, rx_data, buffer, sizeof(buffer));
  //   }
  // }
  // if (len < 0) {
  //   if (debug) {
  //     (TAG, "Socket len < 0");
  //   }
  // }
}

void WWRESIComponent::so_accept() {
  struct sockaddr_storage client_addr;
  socklen_t client_addrlen = sizeof(client_addr);
  std::unique_ptr<socket::Socket> socket =
      this->socket_->accept(reinterpret_cast<struct sockaddr *>(&client_addr), &client_addrlen);
  if (!socket)
    return;

  socket->setblocking(false);
  std::string identifier = socket->getpeername();
  this->clients_.emplace_back(std::move(socket), identifier, this->buf_head_);
  ESP_LOGD(TAG, "New client connected from %s", identifier.c_str());
  this->publish_sensor();
}

void WWRESIComponent::so_cleanup() {
  auto discriminator = [](const Client &client) { return !client.disconnected; };
  auto last_client = std::partition(this->clients_.begin(), this->clients_.end(), discriminator);
  if (last_client != this->clients_.end()) {
    this->clients_.erase(last_client, this->clients_.end());
    this->publish_sensor();
  }
}

void WWRESIComponent::so_read() {
  size_t len = 0;
  int available;
  while ((available = this->available()) > 0) { // Read uart
    size_t free = this->buf_size_ - (this->buf_head_ - this->buf_tail_);
    if (free == 0) {
      // Only overwrite if nothing has been added yet, otherwise give flush() a chance to empty the buffer first.
      if (len > 0)
        return;

      ESP_LOGE(TAG, "Incoming bytes available, but outgoing buffer is full: stream will be corrupted!");
      free = std::min<size_t>(available, this->buf_size_);
      this->buf_tail_ += free;
      for (Client &client : this->clients_) {
        if (client.position < this->buf_tail_) {
          ESP_LOGW(TAG, "Dropped %u pending bytes for client %s", this->buf_tail_ - client.position,
                   client.identifier.c_str());
          client.position = this->buf_tail_;
        }
      }
    }

    // Fill all available contiguous space in the ring buffer.
    len = std::min<size_t>(available, std::min<size_t>(this->buf_ahead(this->buf_head_), free));
    this->read_array(&this->buf_[this->buf_index(this->buf_head_)], len);
    this->buf_head_ += len;
  }
}

void WWRESIComponent::so_flush() {
  ssize_t written;
  this->buf_tail_ = this->buf_head_;
  for (Client &client : this->clients_) {
    if (client.disconnected || client.position == this->buf_head_)
      continue;

    // Split the write into two parts: from the current position to the end of the ring buffer, and from the start
    // of the ring buffer until the head. The second part might be zero if no wraparound is necessary.
    struct iovec iov[2];
    iov[0].iov_base = &this->buf_[this->buf_index(client.position)];
    iov[0].iov_len = std::min(this->buf_head_ - client.position, this->buf_ahead(client.position));
    iov[1].iov_base = &this->buf_[0];
    iov[1].iov_len = this->buf_head_ - (client.position + iov[0].iov_len);
    if ((written = client.socket->writev(iov, 2)) > 0) {
      client.position += written;
    } else if (written == 0 || errno == ECONNRESET) {
      ESP_LOGD(TAG, "Client %s disconnected", client.identifier.c_str());
      client.disconnected = true;
      continue;  // don't consider this client when calculating the tail position
    } else if (errno == EWOULDBLOCK || errno == EAGAIN) {
      // Expected if the (TCP) transmit buffer is full, nothing to do.
    } else {
      ESP_LOGE(TAG, "Failed to write to client %s with error %d!", client.identifier.c_str(), errno);

      // todo eso check if eth present ???
      if (errno == ENOTCONN ||errno == ECONNABORTED) {
        client.disconnected = true;
      }
      if (!network::is_connected) {
        ESP_LOGE(TAG, "network disconnect");
        client.disconnected = true;
      }

      if (!ethernet::global_eth_component->is_connected()) {
        ESP_LOGE(TAG, "ethernet disconnect");
          client.disconnected = true;
      }
    }

    this->buf_tail_ = std::min(this->buf_tail_, client.position);
  }
}

void WWRESIComponent::so_write() {
  uint8_t buf[128];
  ssize_t read;
  for (Client &client : this->clients_) {
    if (client.disconnected)
      continue;

    while ((read = client.socket->read(&buf, sizeof(buf))) > 0)
      this->write_array(buf, read);

    if (read == 0 || errno == ECONNRESET) {
      ESP_LOGD(TAG, "Client %s disconnected", client.identifier.c_str());
      client.disconnected = true;
    } else if (errno == EWOULDBLOCK || errno == EAGAIN) {
      // Expected if the (TCP) receive buffer is empty, nothing to do.
    } else {
      ESP_LOGW(TAG, "Failed to read from client %s with error %d!", client.identifier.c_str(), errno);

      // todo eso check if eth present ???
      if (errno == ENOTCONN ||errno == ECONNABORTED) {
        client.disconnected = true;
      }
      if (!network::is_connected) {
        ESP_LOGE(TAG, "network disconnect");
          client.disconnected = true;
      }

      if (!ethernet::global_eth_component->is_connected()) {
        ESP_LOGE(TAG, "ethernet disconnect");
          client.disconnected = true;
      }
    }
  }
}

WWRESIComponent::Client::Client(std::unique_ptr<esphome::socket::Socket> socket, std::string identifier,
                                size_t position)
    : socket(std::move(socket)), identifier{identifier}, position{position} {}

// bool WWRESIComponent::readall_(uint8_t *buf, size_t len) {
//   uint32_t start = millis();
//   uint32_t at = 0;
//   while (len - at > 0) {
//     uint32_t now = millis();
//     if (now - start > 1000) {
//       ESP_LOGW(TAG, "Timed out reading %d bytes of data", len);
//       return false;
//     }

//     ssize_t read = this->client_->read(buf + at, len - at);
//     if (read == -1) {
//       if (errno == EAGAIN || errno == EWOULDBLOCK) {
//         App.feed_wdt();
//         delay(1);
//         continue;
//       }
//       ESP_LOGW(TAG, "Failed to read %d bytes of data, errno: %d", len, errno);
//       return false;
//     } else if (read == 0) {
//       ESP_LOGW(TAG, "Remote closed connection");
//       return false;
//     } else {
//       at += read;
//     }
//     App.feed_wdt();
//     delay(1);
//   }

//   return true;
// }

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
  // eso  ack << "OK\nGET DIN " << (ret & DIN_MASK) << "\n\r\n";
  // eso  writeToAll(ack.str());
  ValCurrent[CH_DIN] = RecentDIN = ret;
  // todo eso RefreshLcdDIN();
}

//---------------------------------------------------------------------------
/// @brief
/// @param chan
/// @param
void WWRESIComponent::transition_mode(int chan, enum e_MODE mode) {
  Index[chan] = 0;
  Mode[chan] = mode;
  if (mode == e_LOCAL) {
    ValCurrent[chan] = ValVirtual[chan][Index[chan]];
  } else {
    ValCurrent[chan] = ValSet[chan];
  }
  set_r_channel(chan, ValCurrent[chan]);
  RefreshIndex();
}

//---------------------------------------------------------------------------
/// @brief
/// @param
void WWRESIComponent::stateMachine_DIN(void) {
  int i;
  for (i = 0; i < CH_R_N; i++) {
    int din = (RecentDIN >> (2 * i)) & 3;
    if (din != ConfirmedDIN[i]) {
      if (din == LastDIN[i]) {
        if ((tnow - tLastDINChanged[i]) > DEBOUNCE_DIN) {
          // 2 bits FROM + 2 bits TO
          // FROM may be also -1 during program initialization
          transition_DIN(i, (ConfirmedDIN[i] << 2) | din);
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

//---------------------------------------------------------------------------
/// @brief
/// @param chan
/// @param transition
void WWRESIComponent::transition_DIN(int chan, int transition) {
  if (Mode[chan] != e_LOCAL) {
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
  set_r_channel(chan, ValCurrent[chan]);
  RefreshIndex();

  // send ACK message
  std::ostringstream ack;
  ack << "OK\nINDX " << chan << " " << Index[chan] << "\nSET " << chan << " " << ValCurrent[chan] << "\n\r\n";
  // todo eso  writeToAll(ack.str(), Linebuffer::F_ECHO_INDX);
}

//---------------------------------------------------------------------------
/// @brief function based on PwdOutValue from Pwd-Treiber.txt (Pascal).
/// @param value value resistance in Ohm, negative means open, maximum ::RESI_R_MAX
/// @param relays relays 4 bytes containing relay settings to get value Ohm
/// @return 0 on success, -1 on error
int WWRESIComponent::calculate_relays(int value, unsigned char relays[RESI_RELAYS_N]) {
  int i;
  int DecadeSingleResistor;
  int DecadeMultFact;
  int BytePos;
  int BitPos;

  // open all
  for (i = 0; i < RESI_RELAYS_N; i++) {
    relays[i] = 0;
  }

  if (value >= 0) {
    DecadeSingleResistor = RESI_DECADE_SINGLE_RESISTOR_M;
    DecadeMultFact = RESI_DECADE_MULT_FACT_M;

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
        relays[BytePos] ^= (1 << BitPos);
      }
    }

    // check whether bypass is possible,
    if (relays[3] == 0 && relays[2] == 0 && relays[1] == 0) {
      relays[3] ^= 0x04;  // mr01 K27
      if ((relays[0] & 0xf0) == 0) {
        relays[3] ^= 0x02;  // mr01 K26
      }
    }
  } else {
    relays[3] ^= 0x08;  // mr01 K28
  }

  if (debug) {
    std::ostringstream dbgstr;
    dbgstr << "relays = " << std::hex << (int) relays[3] << " " << (int) relays[2] << " " << (int) relays[1] << " "
           << (int) relays[0];
    ESP_LOGD(TAG, "Rel: %02x %02x %02x %02x", relays[3], relays[2], relays[1], relays[0]);
  }

  return 0;
}

//---------------------------------------------------------------------------
/// @brief
/// @param chan
/// @param r
void WWRESIComponent::set_r_channel_number(int chan, double r) {
  set_r_channel_load(chan, r);

  if (this->resistance_number_ != nullptr)
    this->resistance_number_->publish_state(static_cast<int>(round(r)));
}

/// @brief
/// @param chan
/// @param r
void WWRESIComponent::set_r_channel_load(int chan, double r) {
  ValLoad[chan] = r;
  ValSet[chan] = ValLoad[chan];
  ValCurrent[chan] = ValSet[chan];
  this->store.resistor = round(ValCurrent[chan]);
  set_r_channel(chan, ValCurrent[chan]);
}

/// @brief
void WWRESIComponent::set_resistor_value() { set_r_channel_load(0, this->store.resistor); }

//---------------------------------------------------------------------------
/// @brief
/// @param chan
/// @param r
void WWRESIComponent::set_r_channel(int chan, double r) {
  unsigned char relays[RESI_RELAYS_N];
  char val_str[30];

  int ri = (int) round(r);
  calculate_relays(ri, relays);

  // relays[2] ^= 0x80;  // invert bit X24 here instead in FPGA RESI20

  if (debug) {
    std::ostringstream dbgstr;
    dbgstr << "set_r_channel(" << chan << "," << ri << ") = " << std::hex << (int) relays[3] << " " << (int) relays[2]
           << " " << (int) relays[1] << " " << (int) relays[0];
    ESP_LOGD(TAG, "%s", dbgstr.str().c_str());
  }

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

//---------------------------------------------------------------------------
/// @brief put uart char to buffer until CR 0x0a
/// @param rx_data
/// @param buffer
/// @param len
void WWRESIComponent::readline_(int streamNr, int rx_data, uint8_t *buffer, int len) {
  static int pos = 0;
  if (rx_data > 0) {
    if (debug) {
      ESP_LOGD(TAG, "rx: %02x", rx_data);
    }
    switch (rx_data) {
      case '\r':  // Return  CR 0x0d 13
        break;
      case '\n':  // Ignore LF 0x0a 10
        addCommandToStream_(streamNr, buffer, pos);
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
/// @param streamNr
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
    ESP_LOGD(TAG, "new cmd '%s' from %d", str.c_str(), (*it)->m_fd);
  }

  if (str.length() == 0) {
    return;
  }

  handleCommand((*it)->m_fd, (*it), str);
}

//==========================================================================================================
// remove CH_R1x
/// @brief check given channels correct
/// @param chan
/// @return class t_CHANNELS
t_CHANNELS WWRESIComponent::parse_channel(string &chan) {
  t_CHANNELS channels = 0;
  const char *str = chan.data();
  if (chan.length() == 0) {
    // channels | CH_R0 | CH_R1 | CH_DOUT;
    channels | CH_R0 | CH_DOUT;
  } else if (chan.length() == 1) {
    if (chan[0] == '0') {
      channels | CH_R0;
      //    } else if (chan == "1") {
      //      channels | CH_R1;
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
        //      } else if (str[0] == '1') {
        //        channels | CH_R1V;
      } else {
        return CH_INVALID;
      }
    } else if (chan == "ALL") {
      // channels | CH_R0 | CH_R1;
      channels | CH_R0;
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

//---------------------------------------------------------------------------
/// @brief run command called from Linebuffer
/// @param stream
/// @param line
/// @return
int WWRESIComponent::handleCommand(int sNr, class Linebuffer *stream, string &line) {
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
    ESP_LOGD(TAG, "cmd: %s - chan %s - value %s - unit %c", cmd.c_str(), chan.c_str(), value.c_str(), unitch);
  }
  //--- set and load command .........................
  if (cmd == "SET" || cmd == "LOAD") {
    channels = parse_channel(chan);
    if (channels == CH_INVALID || (channels & CH(CH_DIN))) {
      ack << "ERR " << stream->m_fd << " invalid channel :" << chan << ": " << channels << "\n";
      ack << line << "\n";
      goto error_jump;
    }
    if (value != "") {
      if ((channels & (CH(CH_R0) /*| CH(CH_R1)*/)) && (channels & CH(CH_DOUT))) {
        // this will never happen in ParseChannel
        ESP_LOGE(TAG, "HandleCommand ParseChannel problem %s %d", chan.c_str(), channels);
        ack << "ERR " << stream->m_fd << " channel\n";
        ack << line << "\n";
        goto error_jump;
      } else if (channels & (CH(CH_R0) | /* CH(CH_R1) | CH(CH_R1V) |*/ CH(CH_R0V))) {
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
          ESP_LOGD(TAG, "val %.0f - value %s - unit %c", val, value.c_str(), unitch);
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
        }
        // else if (ResiType & RESI_T_M) {
        //   if (val < 0.0) {
        //     if (val < -1.0001 || val > -0.9999) {
        //       ack << "ERR " << stream->m_fd << " wrong value\n";
        //       ack << line << "\n";
        //       goto error_jump;
        //     }
        //   } else if (val > RESI_R_MAX_M + 0.0001 || fabs(10 * round(val / 10) - val) > 0.0001) {
        //     ack << "ERR " << stream->m_fd << " wrong value\n";
        //     ack << line << "\n";
        //     goto error_jump;
        //   }
        // } else {
        //   if (val < -1.0001 || val > RESI_R_MAX + 0.0001 || fabs(round(val) - val) > 0.0001) {
        //     ack << "ERR " << stream->m_fd << " wrong value\n";
        //     ack << line << "\n";
        //     goto error_jump;
        //   }
        // }
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
              // case CH_R1:
              if (Mode[i] != e_LOCAL) {
                ValSet[i] = ValLoad[i];
                ValCurrent[i] = ValSet[i];
                set_r_channel_number(i, ValCurrent[i]);
                ack << "SET " << i << " " << (int) ValCurrent[i] << "\n";
              } else {
                ack << "SET " << i << " " << (int) ValCurrent[i] << "\n";
              }
              break;
            case CH_R0V:
              // case CH_R1V:
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
              // case CH_R1:
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
    // --------------------------------------------------CLEAR
    channels = parse_channel(chan);
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
    // --------------------------------------------------GET
    channels = parse_channel(chan);
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
            // case CH_R1:
            ack << "SET " << i << " " << (long) ValCurrent[i] << "\n";
            break;
          case CH_R0V:
            // case CH_R1V:
            ack << "SET " << i - CH_R0V << "V" << (channels & 0xff) << " " << ValVirtual[i - CH_R0V][channels & 0xff]
                << "\n";
            break;
          case CH_DOUT:
            ack << "SET DOUT " << (long) (ValCurrent[i]) << "\n";
            break;
          case CH_DIN:
            ack << "GET DIN " << ((long) (ValCurrent[i]) & DIN_MASK) << "\n";
            break;
          default:
            break;
        }
      }
    }

  } else if (cmd == "MODE") {
    // --------------------------------------------------MODE
    if (chan == "LOCAL") {
      Mode[CH_R0] = /*Mode[CH_R1] =*/e_LOCAL;
      transition_mode(CH_R0, Mode[CH_R0]);
      // transition_mode(CH_R1, Mode[CH_R1]);
    } else if (chan == "REMOTE") {
      Mode[CH_R0] = /*Mode[CH_R1] =*/e_REMOTE;
      transition_mode(CH_R0, Mode[CH_R0]);
      // transition_mode(CH_R1, Mode[CH_R1]);
    } else if (chan == "") {
    } else {
      ack << "ERR " << stream->m_fd << " invalid MODE :" << chan << ":\n";
      ack << line << "\n";
      goto error_jump;
    }
    ack << "OK " << stream->m_fd << "\n";
    ack << "MODE ";
    ack << ((Mode[CH_R0] == e_LOCAL) ? "LOCAL" : "REMOTE");
    ack << "\n";
    // CHANGE removed in V3.3 as not in spec FL01DE0000-00088-19/4
    // ack << "SET 0 "<< ValCurrent[0]<<"\n";
    // ack << "SET 1 "<< ValCurrent[1]<<"\n";

  } else if (cmd == "IP") {
    // --------------------------------------------------IP
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
    // --------------------------------------------------RST
    ack << "OK " << stream->m_fd << "\nRST\n";
    do_modul_restart = true;

    // } else if (cmd == "ECHO") {
    //   // --------------------------------------------------ECHO
    //   val = strtol(chan.c_str(), &end_ptr, 10);
    //   ack << "OK " << stream->m_fd << "\nECHO ";
    //   stream->m_flags = val;
    //   ack << stream->m_flags << "\n";

  } else if (cmd == "TYPE") {
    // --------------------------------------------------TYPE
    ack << "OK " << stream->m_fd << "\nTYPE ";
    if (ResiType & RESI_T_KM) {
      ack << "RESI 4 2.6M\n";
      ack << "RMIN 0\nRMAX " << RESI_R_MAX_M << "\n";
      ack << "RSTEP 1\nROPEN -1\nINP_N 4\nOUT_N 4\nVIRT_N 256\n";
    }
    //  else {
    //   ack << "RESI 2 266K\n";
    //   ack << "RMIN 0\nRMAX " << RESI_R_MAX << "\n";
    //   ack << "RSTEP 1\nROPEN -1\nINP_N 4\nOUT_N 4\nVIRT_N 256\n";
    // }
    ack << "SW " << version << "\n";
    ack << "SN " << serialnr << "\n";

  } else if (cmd == "CFRAME") {
    // --------------------------------------------------CFRAME
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
    // --------------------------------------------------CBAUDRATE
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
    // --------------------------------------------------CRID
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
    // --------------------------------------------------CRID
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
    // --------------------------------------------------CRID
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
    // --------------------------------------------------VERSION
    // ignore

  } else if (cmd == "CONFIG") {
    // --------------------------------------------------CONFIG
    // TODO switch to quiet (no-ack) mode

  } else if (cmd == "END") {
    // --------------------------------------------------END
    // TODO switch to normal ACK mode

  } else if (cmd == "SAVE") {
    // --------------------------------------------------SAVE
    this->config_write_nvs_();
    ack << "OK " << stream->m_fd << "\n";
    ack << "SAVE\n";

  } else if (cmd[0] == '#') {
    // --------------------------------------------------#
    // comment line, ignore

  } else if (cmd.length() == 0) {
    // --------------------------------------------------
    // empty line, ignore

  } else {
    // --------------------------------------------------
    ack << "ERR " << stream->m_fd << " unknown command\n";
    ack << line << "\n";
  }

error_jump:
  ack << "\r\n";

  if (debug) {
    ESP_LOGD(TAG, "ack: %s", ack.str().c_str());
  }

  writeTo(stream->m_fd, ack.str());

  // eso check it ??????

  // // if F_ECHO_OTHER flag is not there, send ACK to this stream anyway
  // if (!(stream->m_flags & Linebuffer::F_ECHO_OTHER)) {
  //   ret = write(stream->m_fd, ack.str().data(), ack.str().length());

  // eso check it ??????

  if (do_modul_restart) {
    this->restart();
    // this function will not exit, therefore it is done after ACK is sent
  }
  return 0;
}

//---------------------------------------------------------------------------
// todo can
/// @brief write ack back to source uart,net can
/// @param fd
/// @param str
void WWRESIComponent::writeTo(int fd, const string &str) {
  switch (fd) {
    case 0:  // uart
      write_str((str.data()));
      break;
    case 1:  // net
      // if (!client_) {
      //   ESP_LOGW(TAG, "Eth client not connected");
      //   return;
      // }
      // client_->write(str.data(), str.length());
      break;
    case 2:  // can
      // eso todo
      break;
    default:
      break;
  }
}

//---------------------------------------------------------------------------
/// @brief
void WWRESIComponent::restart() {
  t_Linebuffer_List::iterator i;
  ESP_LOGD(TAG, "Sending restart command");

  if (time_v_changed) {
    config_write_nvs_();
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
/// @brief read store_ from NVS
void WWRESIComponent::config_read_nvs_() {
  ESP_LOGCONFIG(TAG, "  Read saved config...");
  // todo eso set all store_ to default
  switch (this->restore_mode_) {
    case WWRESI_RESTORE_DEFAULT_OPEN:
      this->nvs_ = global_preferences->make_preference<int32_t>(this->get_object_id_hash());
      if (!this->nvs_.load(&this->store)) {
        ESP_LOGW(TAG, "  Load error, set to factory ");
        this->store.resistor = FACTORY_RESISTANCE;
      } else {
        this->store.resistor = clamp((int) this->store.resistor, FACTORY_RESISTANCE, RESI_R_MAX_M);

        ESP_LOGCONFIG(TAG, "  Set resistor to %d ", this->store.resistor);
        this->set_r_channel_number(0, this->store.resistor);
        this->store.first_read = false;
      }
      break;
    case WWRESI_ALWAYS_OPEN:
      ESP_LOGCONFIG(TAG, "  Set resistor to WWRESI_ALWAYS_OPEN ");
      this->store.resistor = FACTORY_RESISTANCE;
      break;
  }
}

//---------------------------------------------------------------------------
/// @brief write store_ do NVS
void WWRESIComponent::config_write_nvs_() {
  int resistor = this->store.resistor;

  if (this->store.last_resistor != resistor || this->publish_initial_value_) {
    if (this->restore_mode_ == WWRESI_RESTORE_DEFAULT_OPEN) {
      this->nvs_.save(&this->store);
      ESP_LOGD(TAG, "Save resistor %d ", resistor);
    }
    this->store.last_resistor = resistor;
    // todo eso    this->publish_state(resistor);
    // this->listeners_.call(resistor);
    this->publish_initial_value_ = false;
  }
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
    this->timeout_number_->publish_state(static_cast<uint16_t>(this->store.timeout));
  // if (this->gate_select_number_ != nullptr)
  //   this->gate_select_number_->publish_state(0);
  if (this->resistance_number_ != nullptr)
    this->resistance_number_->publish_state(static_cast<int>(this->store.resistor));
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
#endif

}  // namespace wwresi
}  // namespace esphome
