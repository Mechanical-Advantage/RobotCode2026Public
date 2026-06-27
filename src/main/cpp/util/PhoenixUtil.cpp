// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "util/PhoenixUtil.h"

std::vector<ctre::phoenix6::BaseStatusSignal*> PhoenixUtil::canivoreSignals;
std::vector<ctre::phoenix6::BaseStatusSignal*> PhoenixUtil::rioSignals;

void PhoenixUtil::tryUntilOk(
    int maxAttempts,
    const std::function<ctre::phoenix::StatusCode()>& command) {
  for (int i = 0; i < maxAttempts; i++) {
    ctre::phoenix::StatusCode error = command();
    if (error.IsOK()) {
      break;
    }
  }
}

void PhoenixUtil::refreshAll() {
  if (!canivoreSignals.empty()) {
    ctre::phoenix6::BaseStatusSignal::RefreshAll(canivoreSignals);
  }
  if (!rioSignals.empty()) {
    ctre::phoenix6::BaseStatusSignal::RefreshAll(rioSignals);
  }
}
