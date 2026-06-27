// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "util/TalonFXCurrentConfigurator.h"

#include <frc/Threads.h>

TalonFXCurrentConfigurator::TalonFXCurrentConfigurator(
    ctre::phoenix6::configs::TalonFXConfigurator& configurator)
    : configurator(configurator) {
  // Launch background thread
  worker = std::thread(&TalonFXCurrentConfigurator::workerLoop, this);
}

TalonFXCurrentConfigurator::~TalonFXCurrentConfigurator() {
  // Signal the background thread to shut down
  running = false;
  cv.notify_all();

  if (worker.joinable()) {
    worker.join();
  }
}

void TalonFXCurrentConfigurator::setConfig(
    const ctre::phoenix6::configs::CurrentLimitsConfigs& config) {
  std::lock_guard<std::mutex> lock(mutex);

  // Skip if identical
  if (hasInitial &&
      targetConfig.StatorCurrentLimit == config.StatorCurrentLimit &&
      targetConfig.StatorCurrentLimitEnable ==
          config.StatorCurrentLimitEnable &&
      targetConfig.SupplyCurrentLimit == config.SupplyCurrentLimit &&
      targetConfig.SupplyCurrentLimitEnable ==
          config.SupplyCurrentLimitEnable &&
      targetConfig.SupplyCurrentLowerLimit == config.SupplyCurrentLowerLimit &&
      targetConfig.SupplyCurrentLowerTime == config.SupplyCurrentLowerTime) {
    return;
  }
  hasInitial = true;

  // Overwrite the target config directly (avoids queueing)
  targetConfig = config;

  // Update states
  needsApply = true;
  isRetry = false;  // Brand new config, drop the retry delay

  // Wake the worker thread
  cv.notify_one();
}

void TalonFXCurrentConfigurator::workerLoop() {
  std::unique_lock<std::mutex> lock(mutex);
  frc::SetCurrentThreadPriority(false, 0);

  while (running) {
    // Wait until there is a configuration request or a shutdown signal
    cv.wait(lock, [this] { return !running || needsApply; });

    if (!running) {
      break;
    }

    // If retrying a previously failed configuration, sleep briefly to avoid
    // spamming the CAN bus. Wake up immediately if a new config is provided.
    if (isRetry) {
      cv.wait_for(lock, std::chrono::milliseconds(20),
                  [this] { return !running || !isRetry; });

      if (!running) {
        break;
      }
    }

    // Consume the config state
    ctre::phoenix6::configs::CurrentLimitsConfigs configToApply = targetConfig;
    needsApply = false;
    isRetry = false;

    // Unlock the mutex so the user's calling thread isn't blocked during CAN
    // requests
    lock.unlock();

    // Apply the configuration
    ctre::phoenix::StatusCode status = configurator.Apply(configToApply);

    // Re-lock to check state and schedule retries if necessary
    lock.lock();

    // If the configuration failed, evaluate if a retry is needed
    if (status != ctre::phoenix::StatusCode::OK) {
      // If needsApply is still false, no new configs arrived while we were
      // blocking on Apply(). Schedule this one to be retried.
      if (!needsApply) {
        needsApply = true;
        isRetry = true;
      }
    }
  }
}
