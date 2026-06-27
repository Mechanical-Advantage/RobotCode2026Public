// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/FlywheelIOTalonFX.h"

#include "util/PhoenixUtil.h"

FlywheelIOTalonFX::FlywheelIOTalonFX() : FlywheelIO() {
  // Configure motor
  ctre::phoenix6::configs::TalonFXConfiguration config{};

  config.Slot0 = ctre::phoenix6::configs::Slot0Configs()
                     .WithKP(units::dimensionless::scalar_t{0.0})
                     .WithKI(units::dimensionless::scalar_t{0.0})
                     .WithKD(units::dimensionless::scalar_t{0.0});
  config.Feedback.SensorToMechanismRatio = units::dimensionless::scalar_t{1.0};
  config.Slot0.kP = 999999.0;

  config.TorqueCurrent.PeakForwardTorqueCurrent = 40.0_A;
  config.TorqueCurrent.PeakReverseTorqueCurrent = 0.0_A;
  config.MotorOutput.PeakForwardDutyCycle = 1.0;
  config.MotorOutput.PeakReverseDutyCycle = 0.0;
  config.CurrentLimits.StatorCurrentLimit = 160.0_A;
  config.CurrentLimits.StatorCurrentLimitEnable = true;

  config.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Coast;
  config.MotorOutput.Inverted =
      ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive;

  PhoenixUtil::tryUntilOk(5, [&]() {
    return talon.GetConfigurator().Apply(config, PhoenixUtil::configTimeout);
  });
  PhoenixUtil::tryUntilOk(5, [&]() {
    return talonFollower.GetConfigurator().Apply(config,
                                                 PhoenixUtil::configTimeout);
  });
  ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
      50_Hz, position, appliedVoltage, supplyCurrentAmps, tempCelsius,
      followerSupplyCurrentAmps, followerTempCelsius);
  velocity.SetUpdateFrequency(200_Hz);
  torqueCurrentAmps.SetUpdateFrequency(1000_Hz);
  talon.GetBridgeOutput().SetUpdateFrequency(1000_Hz);
  PhoenixUtil::tryUntilOk(5, [&]() {
    return ctre::phoenix6::hardware::ParentDevice::OptimizeBusUtilizationForAll(
        0_Hz, talon, talonFollower);
  });
  PhoenixUtil::registerSignals(false, position, velocity, appliedVoltage,
                               supplyCurrentAmps, torqueCurrentAmps,
                               tempCelsius, followerSupplyCurrentAmps,
                               followerTempCelsius);
  talonFollower.SetControl(follower);
};

void FlywheelIOTalonFX::updateInputs(FlywheelIOInputs& inputs) {
  inputs.connected = ctre::phoenix6::BaseStatusSignal::IsAllGood(
      position, velocity, appliedVoltage, supplyCurrentAmps, torqueCurrentAmps,
      tempCelsius);
  inputs.followerConnected = ctre::phoenix6::BaseStatusSignal::IsAllGood(
      followerSupplyCurrentAmps, followerTempCelsius);
  inputs.positionRads = position.GetValue().convert<units::radian>().value();
  inputs.velocityRadsPerSec =
      velocity.GetValue().convert<units::rad_per_s>().value();
  inputs.appliedVoltage = appliedVoltage.GetValue().value();
  inputs.supplyCurrentAmps = supplyCurrentAmps.GetValue().value();
  inputs.followerSupplyCurrentAmps =
      followerSupplyCurrentAmps.GetValue().value();
  inputs.torqueCurrentAmps = torqueCurrentAmps.GetValue().value();
  inputs.tempCelsius = tempCelsius.GetValue().value();
  inputs.followerTempCelsius = followerTempCelsius.GetValue().value();
};

void FlywheelIOTalonFX::applyOutputs(const FlywheelIOOutputs& outputs) {
  switch (outputs.mode) {
    case FlywheelIOOutputMode::COAST:
      talon.SetControl(coastRequest);
      break;
    case FlywheelIOOutputMode::DUTY_CYCLE_BANG_BANG:
      talon.SetControl(velocityDutyCycleRequest.WithVelocity(
          units::angular_velocity::radians_per_second_t{
              outputs.velocityRadsPerSec}));
      break;
    case FlywheelIOOutputMode::TORQUE_CURRENT_BANG_BANG:
      talon.SetControl(velocityTorqueCurrentRequest.WithVelocity(
          units::angular_velocity::radians_per_second_t{
              outputs.velocityRadsPerSec}));
      break;
  }
};

void FlywheelIOTalonFX::stop() { talon.StopMotor(); };
