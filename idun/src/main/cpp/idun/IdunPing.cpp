// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "idun/IdunPing.h"

#include <IdunNetworkConstants.h>
#include <frc/RobotBase.h>
#include <frc/Threads.h>
#include <idun/IdunClient.h>
#include <idun/IdunLogger.h>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>

namespace idun {

IdunPing::IdunPing() : thread(&IdunPing::run, this) {}

IdunPing::~IdunPing() {
  stopFlag.store(true);
  if (thread.joinable()) {
    thread.join();
  }
}

void IdunPing::run() {
  frc::SetCurrentThreadPriority(false, 0);
  std::string serverAddress = std::string(
      frc::RobotBase::IsReal() ? IdunNetworkConstants::serverAddressReal
                               : IdunNetworkConstants::serverAddressSim);
  std::string command = "ping -c 1 -W 1 " + serverAddress + " > /dev/null 2>&1";

  int seqnum = 0;
  while (!stopFlag.load()) {
    seqnum++;

    // Execute ping command and get result
    int result = std::system(command.c_str());
    bool success = (result == 0);

    // Print & log result
    if (!success || !idun::IdunClient::isConnected()) {
      std::cout << "[IdunClient] " << serverAddress
                << " ping status: " << (success ? "GOOD" : "BAD") << std::endl;
    }
    idun::IdunLogger::logPing(seqnum, success);

    // Wait for 500 milliseconds
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

};  // namespace idun
