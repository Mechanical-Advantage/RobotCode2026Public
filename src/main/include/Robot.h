// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include "FlywheelIO.h"
#include "FuelSensorIO.h"
#include "GyroIO.h"
#include "HoodIO.h"
#include "LedsIO.h"
#include "LocalDrive.h"
#include "ModuleIO.h"
#include "RollerSystemIO.h"
#include "SlamIO.h"
#include "util/CanivoreReader.h"
#include "util/TimedRobot.h"

struct TracerTimes {
  units::time::millisecond_t inputCycle;
  units::time::millisecond_t dsAlert;
  units::time::millisecond_t disconnectedStop;
  units::time::millisecond_t phoenixRefresh;
  units::time::millisecond_t subsystemInputs;
  units::time::millisecond_t localDrive;
  units::time::millisecond_t builtInInputs;
  units::time::millisecond_t serialize;
  units::time::millisecond_t transmit;
  units::time::millisecond_t outputCycle;
};

class Robot : public frc::TimedRobot {
 private:
  // Subsystems
  GyroIO* gyro;
  GyroIO* backupGyro;
  ModuleIO* module0;
  ModuleIO* module1;
  ModuleIO* module2;
  ModuleIO* module3;
  SlamIO* slam;
  RollerSystemIO* intakeRoller;
  RollerSystemIO* hopperRoller;
  FuelSensorIO* hopperSensorLeft;
  FuelSensorIO* hopperSensorRight;
  RollerSystemIO* kickerRollerFront;
  RollerSystemIO* kickerRollerBack;
  HoodIO* hood;
  FlywheelIO* flywheel;
  LedsIO* leds;

  // Local drive
  LocalDrive* localDrive;

  bool didObserveUserProgramStarting;
  CanivoreReader canivoreReader{"*"};
  TracerTimes tracerTimes;
  TracerTimes lastTracerTimes;
  static std::vector<idun::SubsystemIO*> subsystems;

 public:
  Robot();
  void RobotPeriodic() override;
  static bool isLocalDrive();
  static void registerSubsystem(idun::SubsystemIO* subsystem);

  void SimulationPeriodic() override;
  void DisabledPeriodic() override;
  void AutonomousPeriodic() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
};
