// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "Robot.h"

#include <IdunNetworkConstants.h>
#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <frc/Threads.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <hal/HAL.h>
#include <idun/IdunClient.h>
#include <idun/IdunLogManager.h>
#include <idun/IdunLogger.h>
#include <networktables/NetworkTableInstance.h>
#include <rev/util/StatusLogger.h>

#include <ctre/phoenix6/SignalLogger.hpp>

#include "Constants.h"
#include "DriveConstants.h"
#include "subsystems/FlywheelIOTalonFX.h"
#include "subsystems/GyroIOPigeon2.h"
#include "subsystems/HoodIOTalonFX.h"
#include "subsystems/LedsIOHAL.h"
#include "subsystems/ModuleIOTalonFX.h"
#include "subsystems/ModuleIOTalonFXAlpha.h"
#include "subsystems/RollerSystemIOTalonFX.h"
#include "subsystems/TurretIOTalonFX.h"
#include "util/PhoenixUtil.h"

Robot::Robot()
    : TimedRobot(units::time::second_t{Constants::loopPeriodSecs},
                 units::time::second_t{Constants::loopPeriodWatchdogSecs}),
      didObserveUserProgramStarting(false) {
  // Basic configuration
  nt::NetworkTableInstance::GetDefault().StopServer();
  frc::DriverStation::SilenceJoystickConnectionWarning(true);
  frc::RobotController::SetBrownoutVoltage(6.0_V);
  ctre::phoenix6::SignalLogger::EnableAutoLogging(false);
  StatusLogger::DisableAutoLogging();

  // Report AdvantageKit usage
  HAL_Report(HALUsageReporting::kResourceType_Framework,
             HALUsageReporting::kFramework_AdvantageKit);
  HAL_Report(HALUsageReporting::kResourceType_LoggingFramework,
             HALUsageReporting::kLoggingFramework_AdvantageKit);

  // Start data log
  idun::IdunLogManager::start();
  frc::DriverStation::StartDataLog(idun::IdunLogManager::getLog());
  idun::IdunLogger::start(idun::IdunLogManager::getLog());

  // Create subsystems
  switch (Constants::robot) {
    case RobotType::COMPBOT:
      gyro = new GyroIOPigeon2(DriveConstants::compbotCanBus,
                               DriveConstants::compbotGyroId);
      module0 = new ModuleIOTalonFX("Module0", DriveConstants::compbotCanBus,
                                    DriveConstants::compbotDriveMotorIdFL,
                                    DriveConstants::compbotTurnMotorIdFL,
                                    DriveConstants::compbotEncoderIdFL,
                                    DriveConstants::compbotEncoderOffsetFL);
      module1 = new ModuleIOTalonFX("Module1", DriveConstants::compbotCanBus,
                                    DriveConstants::compbotDriveMotorIdFR,
                                    DriveConstants::compbotTurnMotorIdFR,
                                    DriveConstants::compbotEncoderIdFR,
                                    DriveConstants::compbotEncoderOffsetFR);
      module2 = new ModuleIOTalonFX("Module2", DriveConstants::compbotCanBus,
                                    DriveConstants::compbotDriveMotorIdBL,
                                    DriveConstants::compbotTurnMotorIdBL,
                                    DriveConstants::compbotEncoderIdBL,
                                    DriveConstants::compbotEncoderOffsetBL);
      module3 = new ModuleIOTalonFX("Module3", DriveConstants::compbotCanBus,
                                    DriveConstants::compbotDriveMotorIdBR,
                                    DriveConstants::compbotTurnMotorIdBR,
                                    DriveConstants::compbotEncoderIdBR,
                                    DriveConstants::compbotEncoderOffsetBR);
      leds = new LedsIOHAL();
      break;
    case RobotType::ALPHABOT:
      gyro = new GyroIOPigeon2(DriveConstants::alphabotCanBus,
                               DriveConstants::alphabotGyroId);
      module0 =
          new ModuleIOTalonFXAlpha("Module0", DriveConstants::alphabotCanBus,
                                   DriveConstants::alphabotDriveMotorIdFL,
                                   DriveConstants::alphabotTurnMotorIdFL,
                                   DriveConstants::alphabotEncoderIdFL,
                                   DriveConstants::alphabotEncoderOffsetFL);
      module1 =
          new ModuleIOTalonFXAlpha("Module1", DriveConstants::alphabotCanBus,
                                   DriveConstants::alphabotDriveMotorIdFR,
                                   DriveConstants::alphabotTurnMotorIdFR,
                                   DriveConstants::alphabotEncoderIdFR,
                                   DriveConstants::alphabotEncoderOffsetFR);
      module2 =
          new ModuleIOTalonFXAlpha("Module2", DriveConstants::alphabotCanBus,
                                   DriveConstants::alphabotDriveMotorIdBL,
                                   DriveConstants::alphabotTurnMotorIdBL,
                                   DriveConstants::alphabotEncoderIdBL,
                                   DriveConstants::alphabotEncoderOffsetBL);
      module3 =
          new ModuleIOTalonFXAlpha("Module3", DriveConstants::alphabotCanBus,
                                   DriveConstants::alphabotDriveMotorIdBR,
                                   DriveConstants::alphabotTurnMotorIdBR,
                                   DriveConstants::alphabotEncoderIdBR,
                                   DriveConstants::alphabotEncoderOffsetBR);
      intakeRoller =
          new RollerSystemIOTalonFX("IntakeRoller", 13, "*", 60_A, false, 1.0);
      hopper =
          new RollerSystemIOTalonFX("HopperRoller", 19, "", 60_A, true, 1.0);
      leftIndexer = new RollerSystemIOTalonFX("LeftIndexerRoller", 6, "", 40_A,
                                              false, 1.0);
      rightIndexer = new RollerSystemIOTalonFX("RightIndexerRoller", 5, "",
                                               40_A, true, 1.0);
      kickerFront = new RollerSystemIOTalonFX("KickerRollerFront", 11, "", 60_A,
                                              true, 1.0);
      kickerBack = new RollerSystemIOTalonFX("KickerRollerBack", 10, "", 60_A,
                                             true, 1.0);
      hood = new HoodIOTalonFX();
      flywheel = new FlywheelIOTalonFX();
      turret = new TurretIOTalonFX();
      leds = new LedsIOHAL();
      break;
  }

  // Create local drive
  localDrive = new LocalDrive(gyro, module0, module1, module2, module3);

  // Start Idun client
  idun::IdunClient::start([&] {
    if (!isLocalDrive()) {
      for (const auto& subsystem : subsystems) {
        subsystem->outputPeriodic();
      }
    }
  });

  // Set RT priority
  frc::SetCurrentThreadPriority(true, 1);
}

void Robot::RobotPeriodic() {
  // Alert DS when robot program is ready
  const auto dsAlertStart = frc::Timer::GetTimestamp();
  if (!didObserveUserProgramStarting &&
      (idun::IdunClient::isConnected() || isLocalDrive())) {
    std::puts("\n********** Robot program ready to enable **********");
    HAL_ObserveUserProgramStarting();
    didObserveUserProgramStarting = true;
  }

  // Stop subsystems when disconnected
  const auto disconnectedStart = frc::Timer::GetTimestamp();
  if (isLocalDrive() || !idun::IdunClient::isConnected()) {
    for (const auto& subsystem : subsystems) {
      subsystem->stop();
    }
  }

  // Refresh Phoenix signals
  const auto phoenixRefreshStart = frc::Timer::GetTimestamp();
  PhoenixUtil::refreshAll();

  // Run subsystem input periodic
  const auto subsystemPeriodicStart = frc::Timer::GetTimestamp();
  for (const auto& subsystem : subsystems) {
    subsystem->inputPeriodic();
  }

  // Run local drive periodic
  const auto localDriveStart = frc::Timer::GetTimestamp();
  localDrive->periodic();
  const auto localDriveEnd = frc::Timer::GetTimestamp();

  // Publish to Idun
  idun::IdunClient::processOutgoing(tracerTimes, lastTracerTimes,
                                    isLocalDrive());

  // Update tracer values
  tracerTimes.inputCycle = frc::Timer::GetTimestamp() - dsAlertStart;
  tracerTimes.dsAlert = disconnectedStart - dsAlertStart;
  tracerTimes.disconnectedStop = phoenixRefreshStart - disconnectedStart;
  tracerTimes.phoenixRefresh = subsystemPeriodicStart - phoenixRefreshStart;
  tracerTimes.subsystemInputs = localDriveStart - subsystemPeriodicStart;
  tracerTimes.localDrive = localDriveEnd - localDriveStart;
  lastTracerTimes = tracerTimes;
}

bool Robot::isLocalDrive() { return frc::DriverStation::GetStickButton(5, 1); };

std::vector<idun::SubsystemIO*> Robot::subsystems = {};

void Robot::registerSubsystem(idun::SubsystemIO* subsystem) {
  subsystems.push_back(subsystem);
}

void Robot::SimulationPeriodic() {};
void Robot::DisabledPeriodic() {};
void Robot::AutonomousPeriodic() {};
void Robot::TeleopPeriodic() {};
void Robot::TestPeriodic() {};

int main() { return frc::StartRobot<Robot>(); }
