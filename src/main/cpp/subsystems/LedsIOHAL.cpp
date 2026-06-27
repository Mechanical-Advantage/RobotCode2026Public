// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/LedsIOHAL.h"

#include <frc/Timer.h>
#include <math.h>

#include <csignal>

#include "LedConstants.h"
#include "Robot.h"

LedsIOHAL* LedsIOHAL::instance = nullptr;

LedsIOHAL::LedsIOHAL() {
  // Initialize PWM and AddressableLEDs
  int32_t status = 0;
  hal::Handle<HAL_DigitalHandle, HAL_FreePWMPort> pwmHandle =
      HAL_InitializePWMPort(HAL_GetPort(LedConstants::port), "", &status);
  handle = HAL_InitializeAddressableLED(pwmHandle, &status);
  HAL_Report(HALUsageReporting::kResourceType_AddressableLEDs,
             LedConstants::port + 1);

  // Configure and start LEDs
  HAL_SetAddressableLEDLength(handle, LedConstants::length, &status);
  HAL_StartAddressableLEDOutput(handle, &status);

  // Add signal handler for exit
  if (instance == nullptr) {
    instance = this;
    std::signal(SIGTERM, [](int signum) {
      if (LedsIOHAL::instance) {
        LedsIOHAL::instance->sigterm();
      }
      std::signal(signum, SIG_DFL);
      std::raise(signum);
    });
  }
}

void LedsIOHAL::updateInputs(LedsIOInputs& inputs) {
  inputs.fpgaTime = frc::Timer::GetFPGATimestamp().value();
}

void LedsIOHAL::applyOutputs(const LedsIOOutputs& outputs) {
  // Wait until "ready", meaning that one full cycle has been completed between
  // the RIO and Mac mini. This ensures that the pattern is fully updated before
  // writing to the LEDs, preventing discontinuities when Idun connects.
  if (outputs.ready) {
    int32_t status = 0;
    HAL_WriteAddressableLEDData(
        handle,
        reinterpret_cast<const HAL_AddressableLEDData*>(outputs.buffer.c_str()),
        outputs.buffer.size() / 4, &status);
  }
}

void LedsIOHAL::stop() {
  // Update local buffer
  double x = (std::fmod(frc::Timer::GetFPGATimestamp().value(),
                        LedConstants::startupBreathDuration) /
              LedConstants::startupBreathDuration) *
             2.0 * M_PI;
  double brightness = (sin(x) + 1.0) / 2.0;
  const auto color =
      Robot::isLocalDrive()
          ? frc::Color(brightness * 0.65, 0.0, brightness)   // Purple
          : frc::Color(brightness, brightness, brightness);  // White
  for (int i = 0; i < LedConstants::length; i++) {
    localBuffer[i].SetLED(color);
  }

  // Write local buffer
  int32_t status = 0;
  HAL_WriteAddressableLEDData(handle, localBuffer.data(), localBuffer.size(),
                              &status);
}

void LedsIOHAL::sigterm() {
  // Update local buffer
  for (int i = 0; i < LedConstants::length; i++) {
    localBuffer[i].SetRGB(0, 0, 0);
  }

  // Write local buffer
  int32_t status = 0;
  HAL_WriteAddressableLEDData(handle, localBuffer.data(), localBuffer.size(),
                              &status);
}
