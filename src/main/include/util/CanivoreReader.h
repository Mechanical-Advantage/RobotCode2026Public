// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <IdunComms.pb.h>

#include <atomic>
#include <ctre/phoenix6/CANBus.hpp>
#include <string_view>
#include <thread>

class CanivoreReader {
 public:
  explicit CanivoreReader(std::string_view canBusName);

  ~CanivoreReader();

  CanivoreReader(const CanivoreReader&) = delete;
  CanivoreReader& operator=(const CanivoreReader&) = delete;
  CanivoreReader(CanivoreReader&&) = delete;
  CanivoreReader& operator=(CanivoreReader&&) = delete;

 private:
  void run();

  ctre::phoenix6::CANBus canBus;
  idun::proto::CANStatus canProto;

  std::atomic<bool> stopFlag{false};
  std::thread thread;
};
