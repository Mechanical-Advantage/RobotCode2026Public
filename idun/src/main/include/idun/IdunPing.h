// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <atomic>
#include <string>
#include <thread>

namespace idun {

class IdunPing {
 public:
  IdunPing();

  ~IdunPing();

  IdunPing(const IdunPing&) = delete;
  IdunPing& operator=(const IdunPing&) = delete;
  IdunPing(IdunPing&&) = delete;
  IdunPing& operator=(IdunPing&&) = delete;

 private:
  void run();

  std::atomic<bool> stopFlag{false};
  std::thread thread;
};

};  // namespace idun
