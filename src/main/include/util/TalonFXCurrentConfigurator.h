// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <ctre/phoenix6/TalonFX.hpp>
#include <mutex>
#include <thread>

class TalonFXCurrentConfigurator {
 public:
  // Delete copy and move constructors for thread safety
  TalonFXCurrentConfigurator(const TalonFXCurrentConfigurator&) = delete;
  TalonFXCurrentConfigurator& operator=(const TalonFXCurrentConfigurator&) =
      delete;

  /**
   * @brief Constructs a new background current configurator.
   * @param configurator Reference to the TalonFX configurator to wrap. Ensure
   * the configurator outlives this class instance.
   */
  explicit TalonFXCurrentConfigurator(
      ctre::phoenix6::configs::TalonFXConfigurator& configurator);

  /**
   * @brief Destructor. Safely halts the background thread and joins it.
   */
  ~TalonFXCurrentConfigurator();

  /**
   * @brief Sets the current limit configuration to be applied.
   * This overwrites any pending config and cancels retries of older configs.
   * * @param config The CurrentLimitsConfigs object to apply.
   */
  void setConfig(const ctre::phoenix6::configs::CurrentLimitsConfigs& config);

 private:
  void workerLoop();

  ctre::phoenix6::configs::TalonFXConfigurator& configurator;

  std::thread worker;
  std::mutex mutex;
  std::condition_variable cv;
  std::atomic<bool> running{true};

  ctre::phoenix6::configs::CurrentLimitsConfigs targetConfig;

  bool hasInitial{false};
  bool needsApply{false};
  bool isRetry{false};
};
