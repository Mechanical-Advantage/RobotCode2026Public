// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <frc/AddressableLED.h>
#include <hal/AddressableLED.h>
#include <hal/AddressableLEDTypes.h>
#include <hal/FRCUsageReporting.h>
#include <hal/HALBase.h>
#include <hal/PWM.h>
#include <hal/Types.h>

#include "LedConstants.h"
#include "LedsIO.h"

class LedsIOHAL : public LedsIO {
 public:
  LedsIOHAL();

 private:
  void updateInputs(LedsIOInputs& inputs) override;
  void applyOutputs(const LedsIOOutputs& outputs) override;
  void stop() override;
  void sigterm();

  hal::Handle<HAL_AddressableLEDHandle, HAL_FreeAddressableLED> handle;
  std::array<frc::AddressableLED::LEDData, LedConstants::length> localBuffer;

  static LedsIOHAL* instance;
};
