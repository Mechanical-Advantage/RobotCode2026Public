// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/SlamIOTalonFX.h"

#include "CurrentLimits.h"
#include "util/PhoenixUtil.h"

SlamIOTalonFX::SlamIOTalonFX()
    : SlamIO(),
      position(talon.GetPosition()),
      velocity(talon.GetVelocity()),
      appliedVoltage(talon.GetMotorVoltage()),
      supplyCurrentAmps(talon.GetSupplyCurrent()),
      torqueCurrentAmps(talon.GetTorqueCurrent()),
      tempCelsius(talon.GetDeviceTemp()) {
  // Configure motor
  config.MotorOutput.Inverted =
      ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
  config.CurrentLimits = PhoenixUtil::currentLimitsDefault;
  config.CurrentLimits.StatorCurrentLimit = units::current::ampere_t{70_A};
  config.CurrentLimits.StatorCurrentLimitEnable = true;
  config.CurrentLimits.SupplyCurrentLimit =
      units::current::ampere_t{CurrentLimits::slamLimitAmps};
  config.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  config.Feedback.SensorToMechanismRatio = 52.5;
  config.Audio = PhoenixUtil::audioConfigs;

  PhoenixUtil::tryUntilOk(5, [&]() {
    return talon.GetConfigurator().Apply(config, PhoenixUtil::configTimeout);
  });
  ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
      200_Hz, position, velocity, appliedVoltage, supplyCurrentAmps,
      torqueCurrentAmps, tempCelsius);
  PhoenixUtil::tryUntilOk(5, [&]() {
    return talon.OptimizeBusUtilization(0_Hz, PhoenixUtil::configTimeout);
  });
  PhoenixUtil::registerSignals(false, position, velocity, appliedVoltage,
                               supplyCurrentAmps, torqueCurrentAmps,
                               tempCelsius);
};

void SlamIOTalonFX::updateInputs(SlamIOInputs& inputs) {
  inputs.connected = ctre::phoenix6::BaseStatusSignal::IsAllGood(
      position, velocity, appliedVoltage, supplyCurrentAmps, torqueCurrentAmps,
      tempCelsius);
  inputs.positionRads = position.GetValue().convert<units::radian>().value();
  inputs.velocityRadsPerSec =
      velocity.GetValue().convert<units::rad_per_s>().value();
  inputs.appliedVoltage = appliedVoltage.GetValue().value();
  inputs.supplyCurrentAmps = supplyCurrentAmps.GetValue().value();
  inputs.torqueCurrentAmps = torqueCurrentAmps.GetValue().value();
  inputs.tempCelsius = tempCelsius.GetValue().value();
};

void SlamIOTalonFX::applyOutputs(const SlamIOOutputs& outputs) {
  if (outputs.kP != prevkP || outputs.kD != prevkD) {
    prevkP = outputs.kP;
    prevkD = outputs.kD;

    config.Slot0 = ctre::phoenix6::configs::Slot0Configs()
                       .WithKP(units::dimensionless::scalar_t{outputs.kP})
                       .WithKD(units::dimensionless::scalar_t{outputs.kD});
    PhoenixUtil::tryUntilOk(5, [&]() {
      return talon.GetConfigurator().Apply(config.Slot0,
                                           PhoenixUtil::configTimeout);
    });
  }

  switch (outputs.mode) {
    case SlamIOOutputMode::BRAKE:
      talon.SetControl(brakeRequest);
      break;
    case SlamIOOutputMode::COAST:
      talon.SetControl(coastRequest);
      break;
    case SlamIOOutputMode::RUN_OPEN_LOOP:
      talon.SetControl(voltageRequest.WithOutput(
          units::voltage::volt_t{outputs.appliedVolts}));
      break;
    case SlamIOOutputMode::RUN_CLOSED_LOOP:
      talon.SetControl(
          positionRequest.WithPosition(units::radian_t{outputs.position}));
      break;
  }
}

void SlamIOTalonFX::stop() { talon.StopMotor(); };
