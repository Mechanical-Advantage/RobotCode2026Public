// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/RollerSystemIOTalonFX.h"

#include "util/PhoenixUtil.h"

RollerSystemIOTalonFX::RollerSystemIOTalonFX(
    const std::string_view name, const int id, const std::string_view bus,
    const units::current::ampere_t currentLimitAmps, const bool invert,
    const double reduction)
    : RollerSystemIO(name),
      talon(id, bus),
      position(talon.GetPosition()),
      velocity(talon.GetVelocity()),
      appliedVoltage(talon.GetMotorVoltage()),
      supplyCurrentAmps(talon.GetSupplyCurrent()),
      torqueCurrentAmps(talon.GetTorqueCurrent()),
      tempCelsius(talon.GetDeviceTemp()),
      reduction(reduction) {
  // Configure drive motor
  ctre::phoenix6::configs::TalonFXConfiguration config{};
  config.MotorOutput.Inverted =
      invert ? ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive
             : ctre::phoenix6::signals::SensorDirectionValue::
                   CounterClockwise_Positive;
  config.CurrentLimits.SupplyCurrentLimit =
      units::current::ampere_t{currentLimitAmps};
  config.CurrentLimits.SupplyCurrentLimitEnable = true;
  config.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;

  PhoenixUtil::tryUntilOk(5, [&]() {
    return talon.GetConfigurator().Apply(config, PhoenixUtil::configTimeout);
  });
  ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
      50_Hz, position, velocity, appliedVoltage, supplyCurrentAmps,
      torqueCurrentAmps, tempCelsius);
  PhoenixUtil::tryUntilOk(5, [&]() {
    return talon.OptimizeBusUtilization(0_Hz, PhoenixUtil::configTimeout);
  });
  PhoenixUtil::registerSignals(
      ctre::phoenix6::CANBus(bus).IsNetworkFD(), position, velocity,
      appliedVoltage, supplyCurrentAmps, torqueCurrentAmps, tempCelsius);
};

void RollerSystemIOTalonFX::updateInputs(RollerSystemIOInputs& inputs) {
  inputs.connected = ctre::phoenix6::BaseStatusSignal::IsAllGood(
      position, velocity, appliedVoltage, supplyCurrentAmps, torqueCurrentAmps,
      tempCelsius);
  inputs.positionRads =
      position.GetValue().convert<units::radian>().value() / reduction;
  inputs.velocityRadsPerSec =
      velocity.GetValue().convert<units::rad_per_s>().value() / reduction;
  inputs.appliedVoltage = appliedVoltage.GetValue().value();
  inputs.supplyCurrentAmps = supplyCurrentAmps.GetValue().value();
  inputs.torqueCurrentAmps = torqueCurrentAmps.GetValue().value();
  inputs.tempCelsius = tempCelsius.GetValue().value();
};

void RollerSystemIOTalonFX::applyOutputs(const RollerSystemIOOutputs& outputs) {
  if (outputs.appliedVoltage == 0.0) {
    if (outputs.brakeModeEnabled) {
      talon.SetControl(brakeRequest);
    } else {
      talon.SetControl(coastRequest);
    }
    return;
  }
  talon.SetControl(voltageRequest.WithOutput(
      units::voltage::volt_t{outputs.appliedVoltage}));
}

void RollerSystemIOTalonFX::stop() { talon.StopMotor(); };
