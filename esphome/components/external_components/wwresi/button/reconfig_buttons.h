#pragma once

#include "esphome/components/button/button.h"
#include "../wwresi.h"

namespace esphome {
namespace wwresi {

class WWRESIApplyConfigButton : public button::Button, public Parented<WWRESIComponent> {
 public:
  WWRESIApplyConfigButton() = default;

 protected:
  void press_action() override;
};

class WWRESIRevertConfigButton : public button::Button, public Parented<WWRESIComponent> {
 public:
  WWRESIRevertConfigButton() = default;

 protected:
  void press_action() override;
};

class WWRESIRestartModuleButton : public button::Button, public Parented<WWRESIComponent> {
 public:
  WWRESIRestartModuleButton() = default;

 protected:
  void press_action() override;
};

class WWRESIFactoryResetButton : public button::Button, public Parented<WWRESIComponent> {
 public:
  WWRESIFactoryResetButton() = default;

 protected:
  void press_action() override;
};

}  // namespace wwresi
}  // namespace esphome
