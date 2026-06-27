// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/TurretIOTalonFX.h"

#include "util/PhoenixUtil.h"

TurretIOTalonFX::TurretIOTalonFX() : TurretIO() {
  // Configure
  config.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  config.Slot0 = ctre::phoenix6::configs::Slot0Configs()
                     .WithKP(units::dimensionless::scalar_t{0.0})
                     .WithKI(units::dimensionless::scalar_t{0.0})
                     .WithKD(units::dimensionless::scalar_t{0.0});
  config.Feedback.SensorToMechanismRatio =
      units::dimensionless::scalar_t{(50.0 / 12.0) * (100.0 / 10.0)};
  config.CurrentLimits.SupplyCurrentLimit = 40.0_A;
  config.CurrentLimits.SupplyCurrentLimitEnable = true;
  config.MotorOutput.Inverted =
      ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

  PhoenixUtil::tryUntilOk(5, [&]() {
    return talon.GetConfigurator().Apply(config, PhoenixUtil::configTimeout);
  });
  ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
      50_Hz, internalVelocity, supplyCurrentAmps, torqueCurrentAmps);
  internalPosition.SetUpdateFrequency(200_Hz);
  PhoenixUtil::tryUntilOk(5, [&]() {
    return talon.OptimizeBusUtilization(0_Hz, PhoenixUtil::configTimeout);
  });
  PhoenixUtil::registerSignals(false, internalPosition, internalVelocity,
                               appliedVoltage, supplyCurrentAmps,
                               torqueCurrentAmps);
};

void TurretIOTalonFX ::updateInputs(TurretIOInputs& inputs) {
  inputs.motorConnected = ctre::phoenix6::BaseStatusSignal::IsAllGood(
      internalPosition, internalVelocity, appliedVoltage, supplyCurrentAmps,
      torqueCurrentAmps);
  inputs.positionRads =
      internalPosition.GetValue().convert<units::radian>().value();
  inputs.velocityRadsPerSec =
      internalVelocity.GetValue().convert<units::rad_per_s>().value();
  inputs.appliedVolts = appliedVoltage.GetValue().value();
  inputs.supplyCurrentAmps = supplyCurrentAmps.GetValue().value();
  inputs.torqueCurrentAmps = torqueCurrentAmps.GetValue().value();
};

void TurretIOTalonFX ::applyOutputs(const TurretIOOutputs& outputs) {
  if (outputs.kP != prevkP || outputs.kD != prevkD) {
    config.Slot0 = ctre::phoenix6::configs::Slot0Configs()
                       .WithKP(units::dimensionless::scalar_t{outputs.kP})
                       .WithKD(units::dimensionless::scalar_t{outputs.kD});
    PhoenixUtil::tryUntilOk(5, [&]() {
      return talon.GetConfigurator().Apply(config.Slot0, 0.25_s);
    });

    prevkP = outputs.kP;
    prevkD = outputs.kD;
  }

  switch (outputs.mode) {
    case TurretIOOutputMode::BRAKE:
      talon.SetControl(brakeRequest);
      break;
    case TurretIOOutputMode::COAST:
      talon.SetControl(coastRequest);
      break;
    case TurretIOOutputMode::CLOSED_LOOP:
      talon.SetControl(
          positionTorqueCurrentFOC
              .WithPosition(units::angle::radian_t{outputs.position})
              .WithVelocity(units::angular_velocity::radians_per_second_t{
                  outputs.velocity}));
      break;
  }
};

void TurretIOTalonFX ::stop() { talon.StopMotor(); };
