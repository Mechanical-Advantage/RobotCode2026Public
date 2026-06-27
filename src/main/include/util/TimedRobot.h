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

#include <hal/Notifier.h>
#include <hal/Types.h>
#include <units/math.h>
#include <units/time.h>
#include <wpi/priority_queue.h>

#include <chrono>
#include <functional>
#include <utility>
#include <vector>

#include "frc/RobotController.h"
#include "util/IterativeRobotBase.h"

namespace frc {

/**
 * TimedRobot implements the IterativeRobotBase robot program framework.
 *
 * The TimedRobot class is intended to be subclassed by a user creating a
 * robot program.
 *
 * Periodic() functions from the base class are called on an interval by a
 * Notifier instance.
 */
class TimedRobot : public IterativeRobotBase {
 public:
  /// Default loop period.
  static constexpr auto kDefaultPeriod = 20_ms;

  /**
   * Provide an alternate "main loop" via StartCompetition().
   */
  void StartCompetition() override;

  /**
   * Ends the main loop in StartCompetition().
   */
  void EndCompetition() override;

  /**
   * Constructor for TimedRobot.
   *
   * @param period Period.
   * @param watchdogPeriod Watchdog period.
   */
  explicit TimedRobot(units::second_t period = kDefaultPeriod,
                      units::second_t watchdogPeriod = kDefaultPeriod);

  TimedRobot(TimedRobot&&) = default;
  TimedRobot& operator=(TimedRobot&&) = default;

  ~TimedRobot() override;

  /**
   * Return the system clock time in micrseconds for the start of the current
   * periodic loop. This is in the same time base as Timer.GetFPGATimestamp(),
   * but is stable through a loop. It is updated at the beginning of every
   * periodic callback (including the normal periodic loop).
   *
   * @return Robot running time in microseconds, as of the start of the current
   * periodic function.
   */
  uint64_t GetLoopStartTime();

  /**
   * Add a callback to run at a specific period with a starting time offset.
   *
   * This is scheduled on TimedRobot's Notifier, so TimedRobot and the callback
   * run synchronously. Interactions between them are thread-safe.
   *
   * @param callback The callback to run.
   * @param period   The period at which to run the callback.
   * @param offset   The offset from the common starting time. This is useful
   *                 for scheduling a callback in a different timeslot relative
   *                 to TimedRobot.
   */
  void AddPeriodic(std::function<void()> callback, units::second_t period,
                   units::second_t offset = 0_s);

 private:
  class Callback {
   public:
    std::function<void()> func;
    std::chrono::microseconds period;
    std::chrono::microseconds expirationTime;

    /**
     * Construct a callback container.
     *
     * @param func      The callback to run.
     * @param startTime The common starting point for all callback scheduling.
     * @param period    The period at which to run the callback.
     * @param offset    The offset from the common starting time.
     */
    Callback(std::function<void()> func, std::chrono::microseconds startTime,
             std::chrono::microseconds period, std::chrono::microseconds offset)
        : func{std::move(func)},
          period{period},
          expirationTime(
              startTime + offset + period +
              (std::chrono::microseconds{frc::RobotController::GetFPGATime()} -
               startTime) /
                  period * period) {}

    bool operator>(const Callback& rhs) const {
      return expirationTime > rhs.expirationTime;
    }
  };

  hal::Handle<HAL_NotifierHandle, HAL_CleanNotifier> m_notifier;
  std::chrono::microseconds m_startTime;
  uint64_t m_loopStartTimeUs = 0;

  wpi::priority_queue<Callback, std::vector<Callback>, std::greater<Callback>>
      m_callbacks;
};

}  // namespace frc
