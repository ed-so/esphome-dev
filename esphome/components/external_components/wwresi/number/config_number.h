#pragma once

#include "esphome/components/number/number.h"
#include "../wwresi.h"

namespace esphome {
namespace wwresi {

class WWRESITimeoutNumber : public number::Number, public Parented<WWRESIComponent> {
 public:
  WWRESITimeoutNumber() = default;

 protected:
  void control(float timeout) override;
};

class WWRESIMinDistanceNumber : public number::Number, public Parented<WWRESIComponent> {
 public:
  WWRESIMinDistanceNumber() = default;

 protected:
  void control(float min_gate) override;
};

// class WW_MR01GateSelectNumber : public number::Number, public Parented<WWRESIComponent> {
//  public:
//   WW_MR01GateSelectNumber() = default;

//  protected:
//   void control(float gate_select) override;
// };

// class WW_MR01MoveSensFactorNumber : public number::Number, public Parented<WWRESIComponent> {
//  public:
//   WW_MR01MoveSensFactorNumber() = default;

//  protected:
//   void control(float move_factor) override;
// };

// class WW_MR01StillSensFactorNumber : public number::Number, public Parented<WWRESIComponent> {
//  public:
//   WW_MR01StillSensFactorNumber() = default;

//  protected:
//   void control(float still_factor) override;
// };

// class WW_MR01StillThresholdNumbers : public number::Number, public Parented<WWRESIComponent> {
//  public:
//   WW_MR01StillThresholdNumbers() = default;
//   WW_MR01StillThresholdNumbers(uint8_t gate);

//  protected:
//   uint8_t gate_;
//   void control(float still_threshold) override;
// };

// class WW_MR01MoveThresholdNumbers : public number::Number, public Parented<WWRESIComponent> {
//  public:
//   WW_MR01MoveThresholdNumbers() = default;
//   WW_MR01MoveThresholdNumbers(uint8_t gate);

//  protected:
//   uint8_t gate_;
//   void control(float move_threshold) override;
// };

}  // namespace wwresi
}  // namespace esphome
