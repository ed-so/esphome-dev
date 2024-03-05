#include "reconfig_buttons.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

static const char *const TAG = "wwresi.button";

namespace esphome {
namespace wwresi {

void WWRESIApplyConfigButton::press_action() { this->parent_->apply_config_action(); }
void WWRESIRevertConfigButton::press_action() { this->parent_->revert_config_action(); }
void WWRESIRestartModuleButton::press_action() { this->parent_->restart_module_action(); }
void WWRESIFactoryResetButton::press_action() { this->parent_->factory_reset_action(); }

}  // namespace wwresi
}  // namespace esphome
