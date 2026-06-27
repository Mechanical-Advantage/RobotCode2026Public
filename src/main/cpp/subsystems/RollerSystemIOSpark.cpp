// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/RollerSystemIOSpark.h"

#include <rev/SparkBase.h>
#include <rev/SparkFlex.h>
#include <rev/SparkLowLevel.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkBaseConfig.h>
#include <rev/config/SparkFlexConfig.h>
#include <rev/config/SparkMaxConfig.h>

#include <numbers>
#include <vector>

#include "DriveConstants.h"
#include "util/SparkUtil.h"

using rev::spark::SparkBase;
using rev::spark::SparkFlex;
using rev::spark::SparkLowLevel;
using rev::spark::SparkMax;

RollerSystemIOSpark::RollerSystemIOSpark(
    const std::string_view name, const int id,
    const units::current::ampere_t currentLimitAmps, const double reduction,
    const bool invert, const bool isFlex)
    : RollerSystemIO(name), config(rev::spark::SparkBaseConfig()) {
  if (isFlex) {
    spark.reset(new SparkFlex(id, SparkLowLevel::MotorType::kBrushless));
  } else {
    spark.reset(new SparkMax(id, SparkLowLevel::MotorType::kBrushless));
  }

  config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
      .Inverted(invert)
      .SmartCurrentLimit(currentLimitAmps.value(), 50)
      .VoltageCompensation(12.0);

  config.encoder.UvwMeasurementPeriod(8).UvwAverageDepth(1);
  config.signals.PrimaryEncoderPositionAlwaysOn(true)
      .PrimaryEncoderPositionPeriodMs(20)
      .PrimaryEncoderVelocityAlwaysOn(true)
      .PrimaryEncoderVelocityPeriodMs(5)
      .AppliedOutputPeriodMs(20)
      .BusVoltagePeriodMs(20)
      .OutputCurrentPeriodMs(20);

  SparkUtil::tryUntilOk(5, [&]() -> rev::REVLibError {
    return spark->Configure(
        config, rev::spark::SparkBase::ResetMode::kResetSafeParameters,
        rev::spark::SparkBase::PersistMode::kPersistParameters);
  });

  SparkUtil::tryUntilOk(5, [&]() -> rev::REVLibError {
    return spark->GetEncoder().SetPosition(0.0);
  });
};

void RollerSystemIOSpark::updateInputs(RollerSystemIOInputs& inputs) {
  SparkUtil::sparkStickyFault = false;
  inputs.positionRads = SparkUtil::ifOkOrDefault(
      spark,
      [&]() -> double {
        return SparkUtil::rotationsToRads(spark->GetEncoder().GetPosition());
      },
      inputs.positionRads);
  inputs.velocityRadsPerSec = SparkUtil::ifOkOrDefault(
      spark,
      [&]() -> double {
        return SparkUtil::rpmToRadsPerSec(spark->GetEncoder().GetVelocity());
      },
      inputs.velocityRadsPerSec);
  std::vector<std::function<double()>> appliedVoltageParams = {
      [&]() -> double { return spark->GetBusVoltage(); },
      [&]() -> double { return spark->GetAppliedOutput(); }};
  inputs.appliedVoltage = SparkUtil::ifOkOrDefault(
      spark, appliedVoltageParams,
      [](std::vector<double> v) { return v[0] * v[1]; }, inputs.appliedVoltage);
  inputs.supplyCurrentAmps = 0.0;
  inputs.torqueCurrentAmps = SparkUtil::ifOkOrDefault(
      spark, [&]() -> double { return spark->GetOutputCurrent(); },
      inputs.torqueCurrentAmps);
  inputs.tempCelsius = SparkUtil::ifOkOrDefault(
      spark, [&]() -> double { return spark->GetMotorTemperature(); },
      inputs.tempCelsius);
  inputs.connected = !SparkUtil::sparkStickyFault;
};

void RollerSystemIOSpark::applyOutputs(const RollerSystemIOOutputs& outputs) {
  if (lastBrakeMode != outputs.brakeModeEnabled) {
    spark->ConfigureAsync(
        config.SetIdleMode((outputs.brakeModeEnabled)
                               ? rev::spark::SparkBaseConfig::IdleMode::kBrake
                               : rev::spark::SparkBaseConfig::IdleMode::kCoast),
        rev::spark::SparkBase::ResetMode::kNoResetSafeParameters,
        rev::spark::SparkBase::PersistMode::kNoPersistParameters);
    lastBrakeMode = outputs.brakeModeEnabled;
  }

  if (outputs.appliedVoltage == 0.0) {
    spark->StopMotor();
    return;
  }
  spark->SetVoltage(units::volt_t{outputs.appliedVoltage});
};

void RollerSystemIOSpark::stop() { spark->StopMotor(); };
