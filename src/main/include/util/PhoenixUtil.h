// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <ctre/phoenix/StatusCodes.h>

#include <ctre/phoenix6/StatusSignal.hpp>
#include <vector>

class PhoenixUtil {
 public:
  PhoenixUtil() = delete;

  static constexpr units::time::second_t configTimeout = 0.25_s;

  static void tryUntilOk(
      int maxAttempts,
      const std::function<ctre::phoenix::StatusCode()>& command);

  /** Registers a set of signals for synchronized refresh. */
  template <typename... Signals>
  static void registerSignals(bool canivore, Signals&... signals) {
    std::initializer_list<ctre::phoenix6::BaseStatusSignal*> signalList = {
        &signals...};
    if (canivore) {
      canivoreSignals.insert(canivoreSignals.end(), signalList.begin(),
                             signalList.end());
    } else {
      rioSignals.insert(rioSignals.end(), signalList.begin(), signalList.end());
    }
  }

  /** Refresh all registered signals. */
  static void refreshAll();

 private:
  static std::vector<ctre::phoenix6::BaseStatusSignal*> canivoreSignals;
  static std::vector<ctre::phoenix6::BaseStatusSignal*> rioSignals;
};
