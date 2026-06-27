// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

// Copyright (c) 2009-2026 FIRST and other WPILib contributors
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// - Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
// - Neither the name of FIRST, WPILib, nor the names of other WPILib
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY FIRST AND OTHER WPILIB CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY NONINFRINGEMENT AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL FIRST OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <units/time.h>

#include "frc/RobotBase.h"
#include "frc/Watchdog.h"

namespace frc {

/**
 * IterativeRobotBase implements a specific type of robot program framework,
 * extending the RobotBase class.
 *
 * The IterativeRobotBase class does not implement StartCompetition(), so it
 * should not be used by teams directly.
 *
 * This class provides the following functions which are called by the main
 * loop, StartCompetition(), at the appropriate times:
 *
 * RobotInit() -- provide for initialization at robot power-on
 *
 * DriverStationConnected() -- provide for initialization the first time the DS
 * is connected
 *
 * Init() functions -- each of the following functions is called once when the
 * appropriate mode is entered:
 *
 * \li DisabledInit() -- called each and every time disabled is entered from
 *   another mode
 * \li AutonomousInit() -- called each and every time autonomous is entered from
 *   another mode
 * \li TeleopInit() -- called each and every time teleop is entered from another
 *   mode
 * \li TestInit() -- called each and every time test is entered from another
 *   mode
 *
 * Periodic() functions -- each of these functions is called on an interval:
 *
 * \li RobotPeriodic()
 * \li DisabledPeriodic()
 * \li AutonomousPeriodic()
 * \li TeleopPeriodic()
 * \li TestPeriodic()
 *
 * Exit() functions -- each of the following functions is called once when the
 * appropriate mode is exited:
 *
 * \li DisabledExit() -- called each and every time disabled is exited
 * \li AutonomousExit() -- called each and every time autonomous is exited
 * \li TeleopExit() -- called each and every time teleop is exited
 * \li TestExit() -- called each and every time test is exited
 */
class IterativeRobotBase : public RobotBase {
 public:
  /**
   * Robot-wide initialization code should go here.
   *
   * Users should override this method for default Robot-wide initialization
   * which will be called when the robot is first powered on. It will be called
   * exactly one time.
   *
   * Note: This method is functionally identical to the class constructor so
   * that should be used instead.
   */
  virtual void RobotInit();

  /**
   * Code that needs to know the DS state should go here.
   *
   * Users should override this method for initialization that needs to occur
   * after the DS is connected, such as needing the alliance information.
   */
  virtual void DriverStationConnected();

  /**
   * Robot-wide simulation initialization code should go here.
   *
   * Users should override this method for default Robot-wide simulation
   * related initialization which will be called when the robot is first
   * started. It will be called exactly one time after RobotInit is called
   * only when the robot is in simulation.
   */
  virtual void SimulationInit();

  /**
   * Initialization code for disabled mode should go here.
   *
   * Users should override this method for initialization code which will be
   * called each time
   * the robot enters disabled mode.
   */
  virtual void DisabledInit();

  /**
   * Initialization code for autonomous mode should go here.
   *
   * Users should override this method for initialization code which will be
   * called each time the robot enters autonomous mode.
   */
  virtual void AutonomousInit();

  /**
   * Initialization code for teleop mode should go here.
   *
   * Users should override this method for initialization code which will be
   * called each time the robot enters teleop mode.
   */
  virtual void TeleopInit();

  /**
   * Initialization code for test mode should go here.
   *
   * Users should override this method for initialization code which will be
   * called each time the robot enters test mode.
   */
  virtual void TestInit();

  /**
   * Periodic code for all modes should go here.
   *
   * This function is called each time a new packet is received from the driver
   * station.
   */
  virtual void RobotPeriodic();

  /**
   * Periodic simulation code should go here.
   *
   * This function is called in a simulated robot after user code executes.
   */
  virtual void SimulationPeriodic();

  /**
   * Periodic code for disabled mode should go here.
   *
   * Users should override this method for code which will be called each time a
   * new packet is received from the driver station and the robot is in disabled
   * mode.
   */
  virtual void DisabledPeriodic();

  /**
   * Periodic code for autonomous mode should go here.
   *
   * Users should override this method for code which will be called each time a
   * new packet is received from the driver station and the robot is in
   * autonomous mode.
   */
  virtual void AutonomousPeriodic();

  /**
   * Periodic code for teleop mode should go here.
   *
   * Users should override this method for code which will be called each time a
   * new packet is received from the driver station and the robot is in teleop
   * mode.
   */
  virtual void TeleopPeriodic();

  /**
   * Periodic code for test mode should go here.
   *
   * Users should override this method for code which will be called each time a
   * new packet is received from the driver station and the robot is in test
   * mode.
   */
  virtual void TestPeriodic();

  /**
   * Exit code for disabled mode should go here.
   *
   * Users should override this method for code which will be called each time
   * the robot exits disabled mode.
   */
  virtual void DisabledExit();

  /**
   * Exit code for autonomous mode should go here.
   *
   * Users should override this method for code which will be called each time
   * the robot exits autonomous mode.
   */
  virtual void AutonomousExit();

  /**
   * Exit code for teleop mode should go here.
   *
   * Users should override this method for code which will be called each time
   * the robot exits teleop mode.
   */
  virtual void TeleopExit();

  /**
   * Exit code for test mode should go here.
   *
   * Users should override this method for code which will be called each time
   * the robot exits test mode.
   */
  virtual void TestExit();

  /**
   * Enables or disables flushing NetworkTables every loop iteration.
   * By default, this is enabled.
   *
   * @param enabled True to enable, false to disable
   * @deprecated Deprecated without replacement.
   */
  [[deprecated("Deprecated without replacement.")]]
  void SetNetworkTablesFlushEnabled(bool enabled);

  /**
   * Sets whether LiveWindow operation is enabled during test mode.
   *
   * @param testLW True to enable, false to disable. Defaults to false.
   * @throws if called in test mode.
   */
  void EnableLiveWindowInTest(bool testLW);

  /**
   * Whether LiveWindow operation is enabled during test mode.
   */
  bool IsLiveWindowEnabledInTest();

  /**
   * Gets time period between calls to Periodic() functions.
   */
  units::second_t GetPeriod() const;

  /**
   * Prints list of epochs added so far and their times.
   */
  void PrintWatchdogEpochs();

  /**
   * Constructor for IterativeRobotBase.
   *
   * @param period Period.
   * @param watchdogPeriod Watchdog period.
   */
  explicit IterativeRobotBase(units::second_t period,
                              units::second_t watchdogPeriod);

  ~IterativeRobotBase() override = default;

 protected:
  IterativeRobotBase(IterativeRobotBase&&) = default;
  IterativeRobotBase& operator=(IterativeRobotBase&&) = default;

  /**
   * Loop function.
   */
  void LoopFunc();

 private:
  enum class Mode { kNone, kDisabled, kAutonomous, kTeleop, kTest };

  Mode m_lastMode = Mode::kNone;
  units::second_t m_period;
  Watchdog m_watchdog;
  bool m_ntFlushEnabled = true;
  bool m_lwEnabledInTest = false;
  bool m_calledDsConnected = false;

  void PrintLoopOverrunMessage();
};

}  // namespace frc
