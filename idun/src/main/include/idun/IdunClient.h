// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <IdunComms.pb.h>
#include <arpa/inet.h>
#include <frc/Timer.h>
#include <frc/geometry/Rotation2d.h>
#include <idun/IdunPing.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <thread>

#include "Robot.h"

namespace idun {

class IdunClient {
 public:
  IdunClient() = delete;

  /** Start the connection to the server. */
  static void start(std::function<void()> applyOutputs);

  /** Returns whether the client is currently connected to the server. */
  static bool isConnected();

  static void setSubsystemInputs(std::string_view name,
                                 idun::proto::IOData& inputs);

  static void setCANStatus(std::string_view name,
                           idun::proto::CANStatus& status);

  static const idun::proto::IOData getSubsystemOutputs(std::string_view name,
                                                       bool& success);

  static void updateGyroOffset(frc::Rotation2d& offset);

  /** Send outgoing messages to the server. */
  static void processOutgoing(TracerTimes& activeTracerTimes,
                              const TracerTimes& loggedTracerTimes,
                              const bool isLocalDrive = false);

 private:
  static void runClient(std::function<void()> applyOutputs);
  static void applyJoystickOutputs();
  static idun::proto::JoystickOutputs lastJoystickOutputs[HAL_kMaxJoysticks];

  static bool started;
  static long long seqnum;
  static long long launchUID;
  static std::thread clientThread;
  static std::atomic<double> lastPacketTimestamp;
  static long long lastPacketSeqnum;
  static long long packetLossCount;
  static long long packetLossFirstSeqnum;
  static int clientSocketFd;
  static struct sockaddr_in serverAddress;
  static bool lastConnected;
  static double lastBuildUIDWarning;
  static IdunPing* ping;

  static std::mutex socketAccessMutex;
  static std::mutex outputsAccessMutex;
  static std::mutex inputsAccessMutex;

  static frc::Timer canReadTimer;
  static idun::proto::RobotInputs inputs;
  static idun::proto::RobotOutputs outputs;
  static idun::proto::RobotOutputs newOutputs;
  static units::time::millisecond_t outputCycleTracer;
  static std::mutex outputCycleTracerMutex;
};

};  // namespace idun
