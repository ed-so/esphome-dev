#pragma once

#include "esphome/components/number/number.h"
#include "../wwmr01.h"

namespace esphome {
namespace wwmr01 {

class WWMR01TimeoutNumber : public number::Number, public Parented<WWMR01Component> {
 public:
  WWMR01TimeoutNumber() = default;

 protected:
  void control(float timeout) override;
};

class WWMR01MinDistanceNumber : public number::Number, public Parented<WWMR01Component> {
 public:
  WWMR01MinDistanceNumber() = default;

 protected:
  void control(float min_gate) override;
};

class WW_MR01MaxDistanceNumber : public number::Number, public Parented<WWMR01Component> {
 public:
  WW_MR01MaxDistanceNumber() = default;

 protected:
  void control(float max_gate) override;
};

// class WW_MR01GateSelectNumber : public number::Number, public Parented<WWMR01Component> {
//  public:
//   WW_MR01GateSelectNumber() = default;

//  protected:
//   void control(float gate_select) override;
// };

// class WW_MR01MoveSensFactorNumber : public number::Number, public Parented<WWMR01Component> {
//  public:
//   WW_MR01MoveSensFactorNumber() = default;

//  protected:
//   void control(float move_factor) override;
// };

// class WW_MR01StillSensFactorNumber : public number::Number, public Parented<WWMR01Component> {
//  public:
//   WW_MR01StillSensFactorNumber() = default;

//  protected:
//   void control(float still_factor) override;
// };

// class WW_MR01StillThresholdNumbers : public number::Number, public Parented<WWMR01Component> {
//  public:
//   WW_MR01StillThresholdNumbers() = default;
//   WW_MR01StillThresholdNumbers(uint8_t gate);

//  protected:
//   uint8_t gate_;
//   void control(float still_threshold) override;
// };

// class WW_MR01MoveThresholdNumbers : public number::Number, public Parented<WWMR01Component> {
//  public:
//   WW_MR01MoveThresholdNumbers() = default;
//   WW_MR01MoveThresholdNumbers(uint8_t gate);

//  protected:
//   uint8_t gate_;
//   void control(float move_threshold) override;
// };

}  // namespace wwmr01
}  // namespace esphome
