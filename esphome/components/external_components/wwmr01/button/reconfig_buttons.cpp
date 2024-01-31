#include "reconfig_buttons.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

static const char *const TAG = "WWMR01.button";

namespace esphome {
namespace wwmr01 {

void WWMR01ApplyConfigButton::press_action() { this->parent_->apply_config_action(); }
void WWMR01RevertConfigButton::press_action() { this->parent_->revert_config_action(); }
void WWMR01RestartModuleButton::press_action() { this->parent_->restart_module_action(); }
void WWMR01FactoryResetButton::press_action() { this->parent_->factory_reset_action(); }

}  // namespace wwmr01
}  // namespace esphome
