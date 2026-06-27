// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <IdunComms.pb.h>
#include <wpi/DataLog.h>

namespace idun {

class IdunLogger {
 public:
  IdunLogger() = delete;

  static void start(wpi::log::DataLog& log);

  static void logConnected(const bool& connected);

  static void logRoboRioUIDs(const long long& buildUID,
                             const long long& launchUID);

  static void logMacMiniUIDs(const long long& buildUID,
                             const long long& launchUID);

  static void logPing(const int seqnum, const bool& connected);

  static void logPacketLossRatio(const double& ratio);

  static void logInputs(const int seqnum, const std::string& inputs);

  static void logOutputs(const int seqnum, const std::string& outputs);

 private:
  static bool running;
  static uint64_t lastInputSample;
  static uint64_t lastOutputSample;
  static wpi::log::RawLogEntry schemaEntry;
  static wpi::log::BooleanLogEntry connectedEntry;
  static wpi::log::IntegerLogEntry buildUIDMacMiniEntry;
  static wpi::log::IntegerLogEntry buildUIDRoboRioEntry;
  static wpi::log::IntegerLogEntry launchUIDMacMiniEntry;
  static wpi::log::IntegerLogEntry launchUIDRoboRioEntry;
  static wpi::log::IntegerLogEntry pingSeqnumEntry;
  static wpi::log::BooleanLogEntry pingSuccessEntry;
  static wpi::log::DoubleLogEntry packetLossRatio;
  static wpi::log::IntegerLogEntry inputsSeqnumEntry;
  static wpi::log::IntegerLogEntry inputsSizeEntry;
  static wpi::log::RawLogEntry inputsSampleEntry;
  static wpi::log::IntegerLogEntry outputsSeqnumEntry;
  static wpi::log::IntegerLogEntry outputsSizeEntry;
  static wpi::log::RawLogEntry outputsSampleEntry;
};

};  // namespace idun
