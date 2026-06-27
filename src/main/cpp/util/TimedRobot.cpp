// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "util/TimedRobot.h"

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

#include <hal/DriverStation.h>
#include <hal/FRCUsageReporting.h>
#include <hal/Notifier.h>
#include <stdint.h>

#include <cstdio>
#include <utility>

#include "frc/Errors.h"

using namespace frc;

void TimedRobot::StartCompetition() {
  RobotInit();

  if constexpr (IsSimulation()) {
    SimulationInit();
  }

  // The RIO robot program has started, but don't tell the DS immediately. The
  // DS will be alerted as part of the main robot loop once Idun is connected or
  // overriden by local drive.
  std::puts("\n********** Robot program startup complete **********");

  // Loop forever, calling the appropriate mode-dependent function
  while (true) {
    // We don't have to check there's an element in the queue first because
    // there's always at least one (the constructor adds one). It's reenqueued
    // at the end of the loop.
    auto callback = m_callbacks.pop();

    int32_t status = 0;
    HAL_UpdateNotifierAlarm(m_notifier, callback.expirationTime.count(),
                            &status);
    FRC_CheckErrorStatus(status, "UpdateNotifierAlarm");

    std::chrono::microseconds currentTime{
        HAL_WaitForNotifierAlarm(m_notifier, &status)};
    if (currentTime.count() == 0 || status != 0) {
      break;
    }

    m_loopStartTimeUs = RobotController::GetFPGATime();

    callback.func();

    // Increment the expiration time by the number of full periods it's behind
    // plus one to avoid rapid repeat fires from a large loop overrun. We assume
    // currentTime ≥ expirationTime rather than checking for it since the
    // callback wouldn't be running otherwise.
    callback.expirationTime +=
        callback.period + (currentTime - callback.expirationTime) /
                              callback.period * callback.period;
    m_callbacks.push(std::move(callback));

    // Process all other callbacks that are ready to run
    while (m_callbacks.top().expirationTime <= currentTime) {
      callback = m_callbacks.pop();

      callback.func();

      callback.expirationTime +=
          callback.period + (currentTime - callback.expirationTime) /
                                callback.period * callback.period;
      m_callbacks.push(std::move(callback));
    }
  }
}

void TimedRobot::EndCompetition() {
  int32_t status = 0;
  HAL_StopNotifier(m_notifier, &status);
}

TimedRobot::TimedRobot(units::second_t period, units::second_t watchdogPeriod)
    : IterativeRobotBase(period, watchdogPeriod) {
  m_startTime = std::chrono::microseconds{RobotController::GetFPGATime()};
  AddPeriodic([=, this] { LoopFunc(); }, period);

  int32_t status = 0;
  m_notifier = HAL_InitializeNotifier(&status);
  FRC_CheckErrorStatus(status, "InitializeNotifier");
  HAL_SetNotifierName(m_notifier, "TimedRobot", &status);
}

TimedRobot::~TimedRobot() {
  if (m_notifier != HAL_kInvalidHandle) {
    int32_t status = 0;
    HAL_StopNotifier(m_notifier, &status);
    FRC_ReportError(status, "StopNotifier");
  }
}

uint64_t TimedRobot::GetLoopStartTime() { return m_loopStartTimeUs; }

void TimedRobot::AddPeriodic(std::function<void()> callback,
                             units::second_t period, units::second_t offset) {
  m_callbacks.emplace(
      callback, m_startTime,
      std::chrono::microseconds{static_cast<int64_t>(period.value() * 1e6)},
      std::chrono::microseconds{static_cast<int64_t>(offset.value() * 1e6)});
}
