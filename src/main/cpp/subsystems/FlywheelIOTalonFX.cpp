// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/FlywheelIOTalonFX.h"

#include "CurrentLimits.h"
#include "util/PhoenixUtil.h"

FlywheelIOTalonFX::FlywheelIOTalonFX(const std::string_view name, const int id,
                                     const std::string_view bus,
                                     const int idFollower1,
                                     const int idFollower2,
                                     const int idFollower3, const bool invert,
                                     const double reduction)
    : FlywheelIO(name),
      talon(id, bus),
      followerTalons{ctre::phoenix6::hardware::TalonFX(idFollower1, bus),
                     ctre::phoenix6::hardware::TalonFX(idFollower2, bus),
                     ctre::phoenix6::hardware::TalonFX(idFollower3, bus)},
      position(talon.GetPosition()),
      velocity(talon.GetVelocity()),
      appliedVoltage(talon.GetMotorVoltage()),
      supplyVoltage(talon.GetSupplyVoltage()),
      supplyCurrentAmps(talon.GetSupplyCurrent()),
      torqueCurrentAmps(talon.GetTorqueCurrent()),
      tempCelsius(talon.GetDeviceTemp()),
      follower1SupplyCurrentAmps(followerTalons[0].GetSupplyCurrent()),
      follower1TempCelsius(followerTalons[0].GetDeviceTemp()),
      follower2SupplyCurrentAmps(followerTalons[1].GetSupplyCurrent()),
      follower2TempCelsius(followerTalons[1].GetDeviceTemp()),
      follower3SupplyCurrentAmps(followerTalons[2].GetSupplyCurrent()),
      follower3TempCelsius(followerTalons[2].GetDeviceTemp()),
      bridgeBrownout(talon.GetFault_BridgeBrownout()),
      follower1BridgeBrownout(followerTalons[0].GetFault_BridgeBrownout()),
      follower2BridgeBrownout(followerTalons[1].GetFault_BridgeBrownout()),
      follower3BridgeBrownout(followerTalons[2].GetFault_BridgeBrownout()) {
  // Configure motor
  config.Slot0 = ctre::phoenix6::configs::Slot0Configs()
                     .WithKP(units::dimensionless::scalar_t{0.0})
                     .WithKI(units::dimensionless::scalar_t{0.0})
                     .WithKD(units::dimensionless::scalar_t{0.0});
  config.Feedback.SensorToMechanismRatio =
      units::dimensionless::scalar_t{reduction};
  config.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Coast;
  config.MotorOutput.Inverted =
      invert
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  config.CurrentLimits = PhoenixUtil::currentLimitsDefault;
  config.CurrentLimits.SupplyCurrentLimit =
      units::current::ampere_t{CurrentLimits::flywheelLimitAmps};
  config.Audio = PhoenixUtil::audioConfigs;

  PhoenixUtil::tryUntilOk(5, [&]() {
    return talon.GetConfigurator().Apply(config, PhoenixUtil::configTimeout);
  });
  for (ctre::phoenix6::hardware::TalonFX& followerTalon : followerTalons) {
    PhoenixUtil::tryUntilOk(5, [&]() {
      return followerTalon.GetConfigurator().Apply(config,
                                                   PhoenixUtil::configTimeout);
    });
  }

  ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
      200_Hz, position, appliedVoltage, supplyVoltage, supplyCurrentAmps,
      torqueCurrentAmps, tempCelsius, follower1SupplyCurrentAmps,
      follower1TempCelsius, follower2SupplyCurrentAmps, follower2TempCelsius,
      follower3SupplyCurrentAmps, follower3TempCelsius, bridgeBrownout,
      follower1BridgeBrownout, follower2BridgeBrownout,
      follower3BridgeBrownout);
  velocity.SetUpdateFrequency(200_Hz);
  appliedVoltage.SetUpdateFrequency(1000_Hz);
  talon.GetBridgeOutput().SetUpdateFrequency(1000_Hz);
  PhoenixUtil::tryUntilOk(5, [&]() {
    return ctre::phoenix6::hardware::ParentDevice::OptimizeBusUtilizationForAll(
        0_Hz, talon, followerTalons[0], followerTalons[1], followerTalons[2]);
  });
  PhoenixUtil::registerSignals(
      true, position, velocity, appliedVoltage, supplyVoltage,
      supplyCurrentAmps, torqueCurrentAmps, tempCelsius,
      follower1SupplyCurrentAmps, follower1TempCelsius,
      follower2SupplyCurrentAmps, follower2TempCelsius,
      follower3SupplyCurrentAmps, follower3TempCelsius, bridgeBrownout,
      follower1BridgeBrownout, follower2BridgeBrownout,
      follower3BridgeBrownout);

  followerTalons[0].SetControl(follower);
  followerTalons[1].SetControl(oppFollower);
  followerTalons[2].SetControl(oppFollower);
};

void FlywheelIOTalonFX::updateInputs(FlywheelIOInputs& inputs) {
  inputs.connected = ctre::phoenix6::BaseStatusSignal::IsAllGood(
      position, velocity, appliedVoltage, supplyCurrentAmps, torqueCurrentAmps,
      tempCelsius, bridgeBrownout);
  inputs.follower1Connected = ctre::phoenix6::BaseStatusSignal::IsAllGood(
      follower1SupplyCurrentAmps, follower1TempCelsius,
      follower1BridgeBrownout);
  inputs.follower2Connected = ctre::phoenix6::BaseStatusSignal::IsAllGood(
      follower2SupplyCurrentAmps, follower2TempCelsius,
      follower2BridgeBrownout);
  inputs.follower3Connected = ctre::phoenix6::BaseStatusSignal::IsAllGood(
      follower3SupplyCurrentAmps, follower3TempCelsius,
      follower3BridgeBrownout);
  inputs.positionRads = position.GetValue().convert<units::radian>().value();
  inputs.velocityRadsPerSec =
      velocity.GetValue().convert<units::rad_per_s>().value();
  inputs.appliedVoltage = appliedVoltage.GetValue().value();
  inputs.supplyVoltage = supplyVoltage.GetValue().value();
  inputs.supplyCurrentAmps = supplyCurrentAmps.GetValue().value();
  inputs.torqueCurrentAmps = torqueCurrentAmps.GetValue().value();
  inputs.tempCelsius = tempCelsius.GetValue().value();
  inputs.follower1SupplyCurrentAmps =
      follower1SupplyCurrentAmps.GetValue().value();
  inputs.follower1TempCelsius = follower1TempCelsius.GetValue().value();
  inputs.follower2SupplyCurrentAmps =
      follower2SupplyCurrentAmps.GetValue().value();
  inputs.follower2TempCelsius = follower2TempCelsius.GetValue().value();
  inputs.follower3SupplyCurrentAmps =
      follower3SupplyCurrentAmps.GetValue().value();
  inputs.follower3TempCelsius = follower3TempCelsius.GetValue().value();
  inputs.bridgeBrownout = bridgeBrownout.GetValue();
  inputs.follower1BridgeBrownout = follower1BridgeBrownout.GetValue();
  inputs.follower2BridgeBrownout = follower2BridgeBrownout.GetValue();
  inputs.follower3BridgeBrownout = follower3BridgeBrownout.GetValue();
};

void FlywheelIOTalonFX::applyOutputs(const FlywheelIOOutputs& outputs) {
  if (outputs.kP != prevkP || outputs.kD != prevkD) {
    config.Slot0 = ctre::phoenix6::configs::Slot0Configs()
                       .WithKP(units::dimensionless::scalar_t{outputs.kP})
                       .WithKD(units::dimensionless::scalar_t{outputs.kD});
    PhoenixUtil::tryUntilOk(5, [&]() {
      return talon.GetConfigurator().Apply(config.Slot0,
                                           PhoenixUtil::configTimeout);
    });

    prevkP = outputs.kP;
    prevkD = outputs.kD;
  }

  switch (outputs.mode) {
    case FlywheelIOOutputMode::COAST:
      talon.SetControl(coastRequest);
      break;
    case FlywheelIOOutputMode::VELOCITY:
      talon.SetControl(
          velocityRequest
              .WithVelocity(units::angular_velocity::radians_per_second_t{
                  outputs.velocityRadsPerSec})
              .WithFeedForward(units::voltage::volt_t{outputs.voltage}));
      break;
    case FlywheelIOOutputMode::VOLTAGE:
      talon.SetControl(
          voltageRequest.WithOutput(units::voltage::volt_t{outputs.voltage}));
      break;
  }
};

void FlywheelIOTalonFX::stop() { talon.StopMotor(); };
