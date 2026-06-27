// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/RollerSystemIOTalonFX.h"

#include "CurrentLimits.h"
#include "util/PhoenixUtil.h"

RollerSystemIOTalonFX::RollerSystemIOTalonFX(
    const std::string_view name, const int id, const std::string_view bus,
    const bool invert, const double reduction, const double currentLimit)
    : RollerSystemIOTalonFX(
          name, std::make_unique<ctre::phoenix6::hardware::TalonFX>(id, bus),
          nullptr, bus, invert, false, reduction, currentLimit) {}

RollerSystemIOTalonFX::RollerSystemIOTalonFX(
    const std::string_view name, const int id, const int followerId,
    const std::string_view bus, const bool invert, const bool followerAligned,
    const double reduction, const double currentLimit)
    : RollerSystemIOTalonFX(
          name, std::make_unique<ctre::phoenix6::hardware::TalonFX>(id, bus),
          std::make_unique<ctre::phoenix6::hardware::TalonFX>(followerId, bus),
          bus, invert, followerAligned, reduction, currentLimit) {}

RollerSystemIOTalonFX::RollerSystemIOTalonFX(
    const std::string_view name,
    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> talonPtr,
    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> followerTalonPtr,
    const std::string_view bus, const bool invert, const bool followerAligned,
    const double reduction, const double currentLimit)
    : RollerSystemIO(name),
      talon(std::move(talonPtr)),
      followerTalon(std::move(followerTalonPtr)),
      position(talon->GetPosition()),
      velocity(talon->GetVelocity()),
      appliedVoltage(talon->GetMotorVoltage()),
      supplyCurrentAmps(talon->GetSupplyCurrent()),
      torqueCurrentAmps(talon->GetTorqueCurrent()),
      tempCelsius(talon->GetDeviceTemp()),
      followerSupplyCurrentAmps(
          followerTalon ? &followerTalon->GetSupplyCurrent() : nullptr),
      followerTempCelsius(followerTalon ? &followerTalon->GetDeviceTemp()
                                        : nullptr) {
  // Configure drive motor
  config.Slot0 = ctre::phoenix6::configs::Slot0Configs()
                     .WithKP(units::dimensionless::scalar_t{0.0})
                     .WithKI(units::dimensionless::scalar_t{0.0})
                     .WithKD(units::dimensionless::scalar_t{0.0});
  config.MotorOutput.Inverted =
      invert
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  config.Feedback.SensorToMechanismRatio =
      units::dimensionless::scalar_t{reduction};
  config.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  config.CurrentLimits = PhoenixUtil::currentLimitsDefault;
  config.CurrentLimits.SupplyCurrentLimit =
      units::current::ampere_t{currentLimit};
  config.Audio = PhoenixUtil::audioConfigs;

  PhoenixUtil::tryUntilOk(5, [&]() {
    return talon->GetConfigurator().Apply(config, PhoenixUtil::configTimeout);
  });
  ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
      200_Hz, position, velocity, appliedVoltage, supplyCurrentAmps,
      torqueCurrentAmps, tempCelsius);
  PhoenixUtil::tryUntilOk(5, [&]() {
    return talon->OptimizeBusUtilization(0_Hz, PhoenixUtil::configTimeout);
  });
  PhoenixUtil::registerSignals(
      ctre::phoenix6::CANBus(bus).IsNetworkFD(), position, velocity,
      appliedVoltage, supplyCurrentAmps, torqueCurrentAmps, tempCelsius);

  if (followerTalon) {
    PhoenixUtil::tryUntilOk(5, [&]() {
      return followerTalon->GetConfigurator().Apply(config,
                                                    PhoenixUtil::configTimeout);
    });
    ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
        200_Hz, *followerSupplyCurrentAmps, *followerTempCelsius);
    PhoenixUtil::tryUntilOk(5, [&]() {
      return followerTalon->OptimizeBusUtilization(0_Hz,
                                                   PhoenixUtil::configTimeout);
    });
    PhoenixUtil::registerSignals(ctre::phoenix6::CANBus(bus).IsNetworkFD(),
                                 *followerSupplyCurrentAmps,
                                 *followerTempCelsius);
    followerTalon->SetControl(
        ctre::phoenix6::controls::Follower(
            talon->GetDeviceID(),
            followerAligned
                ? ctre::phoenix6::signals::MotorAlignmentValue::Aligned
                : ctre::phoenix6::signals::MotorAlignmentValue::Opposed)
            .WithUpdateFreqHz(20_Hz));
  }
};

void RollerSystemIOTalonFX::updateInputs(RollerSystemIOInputs& inputs) {
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

  inputs.hasFollower = followerTalon != nullptr;
  if (followerTalon) {
    inputs.followerConnected = ctre::phoenix6::BaseStatusSignal::IsAllGood(
        *followerSupplyCurrentAmps, *followerTempCelsius);
    inputs.followerSupplyCurrentAmps =
        followerSupplyCurrentAmps->GetValue().value();
    inputs.followerTempCelsius = followerTempCelsius->GetValue().value();
  }
};

void RollerSystemIOTalonFX::applyOutputs(const RollerSystemIOOutputs& outputs) {
  if (outputs.kP != prevkP || outputs.kD != prevkD) {
    config.Slot0 = ctre::phoenix6::configs::Slot0Configs()
                       .WithKP(units::dimensionless::scalar_t{outputs.kP})
                       .WithKD(units::dimensionless::scalar_t{outputs.kD});
    PhoenixUtil::tryUntilOk(5, [&]() {
      return talon->GetConfigurator().Apply(config.Slot0,
                                            PhoenixUtil::configTimeout);
    });

    prevkP = outputs.kP;
    prevkD = outputs.kD;
  }

  switch (outputs.mode) {
    case RollerSystemIOMode::BRAKE:
      talon->SetControl(brakeRequest);
      break;
    case RollerSystemIOMode::COAST:
      talon->SetControl(coastRequest);
      break;
    case RollerSystemIOMode::VOLTAGE_CONTROL:
      talon->SetControl(voltageRequest.WithOutput(
          units::voltage::volt_t{outputs.appliedVoltage}));
      break;
    case RollerSystemIOMode::CLOSED_LOOP:
      talon->SetControl(
          velocityRequest
              .WithVelocity(units::angular_velocity::radians_per_second_t{
                  outputs.velocity})
              .WithFeedForward(units::voltage::volt_t{outputs.feedforward}));
      break;
  }
};

void RollerSystemIOTalonFX::stop() { talon->StopMotor(); };
