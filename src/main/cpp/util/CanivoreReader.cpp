// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "util/CanivoreReader.h"

#include <frc/Threads.h>
#include <idun/IdunClient.h>

#include <chrono>
#include <iostream>

CanivoreReader::CanivoreReader(std::string_view canBusName)
    : canBus(canBusName), thread(&CanivoreReader::run, this) {}

CanivoreReader::~CanivoreReader() {
  stopFlag.store(true);
  if (thread.joinable()) {
    thread.join();
  }
}

void CanivoreReader::run() {
  frc::SetCurrentThreadPriority(false, 0);
  while (!stopFlag.load()) {
    // Read can status
    const auto canStatus = canBus.GetStatus();
    canProto.set_utilization(canStatus.BusUtilization);
    canProto.set_offcount(canStatus.BusOffCount);
    canProto.set_txfullcount(canStatus.TxFullCount);
    canProto.set_receiveerrorcount(canStatus.REC);
    canProto.set_transmiterrorcount(canStatus.TEC);
    idun::IdunClient::setCANStatus("CANivore", canProto);

    // Wait for 500 milliseconds
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}
