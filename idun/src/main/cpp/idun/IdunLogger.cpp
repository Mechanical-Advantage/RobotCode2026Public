// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include <google/protobuf/descriptor.h>
#include <google/protobuf/descriptor.pb.h>
#include <idun/IdunLogger.h>

namespace {
constexpr uint64_t samplePeriodUs = 200e3;
}

namespace idun {

bool IdunLogger::running = false;
uint64_t IdunLogger::lastInputSample = 0;
uint64_t IdunLogger::lastOutputSample = 0;
wpi::log::RawLogEntry IdunLogger::schemaEntry;
wpi::log::BooleanLogEntry IdunLogger::connectedEntry;
wpi::log::IntegerLogEntry IdunLogger::buildUIDMacMiniEntry;
wpi::log::IntegerLogEntry IdunLogger::buildUIDRoboRioEntry;
wpi::log::IntegerLogEntry IdunLogger::launchUIDMacMiniEntry;
wpi::log::IntegerLogEntry IdunLogger::launchUIDRoboRioEntry;
wpi::log::IntegerLogEntry IdunLogger::pingSeqnumEntry;
wpi::log::BooleanLogEntry IdunLogger::pingSuccessEntry;
wpi::log::DoubleLogEntry IdunLogger::packetLossRatio;
wpi::log::IntegerLogEntry IdunLogger::inputsSeqnumEntry;
wpi::log::IntegerLogEntry IdunLogger::inputsSizeEntry;
wpi::log::RawLogEntry IdunLogger::inputsSampleEntry;
wpi::log::IntegerLogEntry IdunLogger::outputsSeqnumEntry;
wpi::log::IntegerLogEntry IdunLogger::outputsSizeEntry;
wpi::log::RawLogEntry IdunLogger::outputsSampleEntry;

void IdunLogger::start(wpi::log::DataLog& log) {
  running = true;

  // Create entries
  schemaEntry = wpi::log::RawLogEntry(log, "/.schema/proto:IdunComms.proto", "",
                                      "proto:FileDescriptorProto");
  connectedEntry = wpi::log::BooleanLogEntry(log, "Idun:Connected");
  buildUIDMacMiniEntry =
      wpi::log::IntegerLogEntry(log, "Idun:BuildUIDs/MacMini");
  buildUIDRoboRioEntry =
      wpi::log::IntegerLogEntry(log, "Idun:BuildUIDs/roboRIO");
  launchUIDMacMiniEntry =
      wpi::log::IntegerLogEntry(log, "Idun:LaunchUIDs/MacMini");
  launchUIDRoboRioEntry =
      wpi::log::IntegerLogEntry(log, "Idun:LaunchUIDs/roboRIO");
  pingSeqnumEntry = wpi::log::IntegerLogEntry(log, "Idun:Ping/Seqnum");
  pingSuccessEntry = wpi::log::BooleanLogEntry(log, "Idun:Ping/Success");
  packetLossRatio = wpi::log::DoubleLogEntry(log, "Idun:PacketLossRatio");
  inputsSeqnumEntry = wpi::log::IntegerLogEntry(log, "Idun:Inputs/Seqnum");
  inputsSizeEntry = wpi::log::IntegerLogEntry(log, "Idun:Inputs/Size");
  inputsSampleEntry = wpi::log::RawLogEntry(log, "Idun:Inputs/Sample", "",
                                            "proto:idun.proto.RobotInputs");
  outputsSeqnumEntry = wpi::log::IntegerLogEntry(log, "Idun:Outputs/Seqnum");
  outputsSizeEntry = wpi::log::IntegerLogEntry(log, "Idun:Outputs/Size");
  outputsSampleEntry = wpi::log::RawLogEntry(log, "Idun:Outputs/Sample", "",
                                             "proto:idun.proto.RobotOutputs");

  // Add schema value
  const auto fileDescriptor =
      google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
          "IdunComms.proto");
  if (fileDescriptor) {
    google::protobuf::FileDescriptorProto fileDescriptorProto;
    fileDescriptor->CopyTo(&fileDescriptorProto);
    std::string serializedDescriptor;
    if (fileDescriptorProto.SerializeToString(&serializedDescriptor)) {
      schemaEntry.Append(
          {reinterpret_cast<const uint8_t*>(serializedDescriptor.data()),
           serializedDescriptor.size()});
    }
  }
};

void IdunLogger::logConnected(const bool& connected) {
  if (running) {
    connectedEntry.Update(connected);
  }
};

void IdunLogger::logRoboRioUIDs(const long long& buildUID,
                                const long long& launchUID) {
  if (running) {
    buildUIDRoboRioEntry.Update(buildUID);
    launchUIDRoboRioEntry.Update(launchUID);
  }
}

void IdunLogger::logMacMiniUIDs(const long long& buildUID,
                                const long long& launchUID) {
  if (running) {
    buildUIDMacMiniEntry.Update(buildUID);
    launchUIDMacMiniEntry.Update(launchUID);
  }
}

void IdunLogger::logPing(const int seqnum, const bool& success) {
  if (running) {
    pingSeqnumEntry.Append(seqnum);
    pingSuccessEntry.Update(success);
  }
};

void IdunLogger::logPacketLossRatio(const double& ratio) {
  if (running) {
    packetLossRatio.Update(ratio);
  }
};

void IdunLogger::logInputs(const int seqnum, const std::string& inputs) {
  if (running) {
    const auto now = wpi::Now();
    inputsSeqnumEntry.Update(seqnum, now);
    inputsSizeEntry.Update(inputs.size(), now);
    if (now - lastInputSample > samplePeriodUs) {
      inputsSampleEntry.Append(
          {reinterpret_cast<const uint8_t*>(inputs.data()), inputs.size()},
          now);
      lastInputSample = now;
    }
  }
};

void IdunLogger::logOutputs(const int seqnum, const std::string& outputs) {
  if (running) {
    const auto now = wpi::Now();
    outputsSeqnumEntry.Update(seqnum, now);
    outputsSizeEntry.Update(outputs.size(), now);
    if (now - lastOutputSample > samplePeriodUs) {
      outputsSampleEntry.Append(
          {reinterpret_cast<const uint8_t*>(outputs.data()), outputs.size()},
          now);
      lastOutputSample = now;
    }
  }
};

};  // namespace idun
