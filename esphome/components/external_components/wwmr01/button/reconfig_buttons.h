#pragma once

#include "esphome/components/button/button.h"
#include "../wwmr01.h"

namespace esphome {
namespace wwmr01 {

class WWMR01ApplyConfigButton : public button::Button, public Parented<WWMR01Component> {
 public:
  WWMR01ApplyConfigButton() = default;

 protected:
  void press_action() override;
};

class WWMR01RevertConfigButton : public button::Button, public Parented<WWMR01Component> {
 public:
  WWMR01RevertConfigButton() = default;

 protected:
  void press_action() override;
};

class WWMR01RestartModuleButton : public button::Button, public Parented<WWMR01Component> {
 public:
  WWMR01RestartModuleButton() = default;

 protected:
  void press_action() override;
};

class WWMR01FactoryResetButton : public button::Button, public Parented<WWMR01Component> {
 public:
  WWMR01FactoryResetButton() = default;

 protected:
  void press_action() override;
};

}  // namespace wwmr01
}  // namespace esphome
