// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "idun/IdunClient.h"

#include <IdunComms.pb.h>
#include <arpa/inet.h>
#include <frc/DriverStation.h>
#include <frc/RobotController.h>
#include <frc/Threads.h>
#include <google/protobuf/util/message_differencer.h>
#include <hal/DriverStation.h>
#include <idun/IdunBuildConstants.h>
#include <idun/IdunLogger.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <wpi/timestamp.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "IdunNetworkConstants.h"
#include "Robot.h"

#if defined(__linux__)
#include <endian.h>
#elif defined(__APPLE__)
#include <libkern/OSByteOrder.h>
#define htobe64(x) OSSwapHostToBigInt64(x)
#endif
#include <frc/Timer.h>

namespace idun {

bool IdunClient::started = false;
long long IdunClient::seqnum = 0;
long long IdunClient::launchUID = 0;
std::thread IdunClient::clientThread;
std::atomic<double> IdunClient::lastPacketTimestamp{-1.0};
long long IdunClient::lastPacketSeqnum = 0;
long long IdunClient::packetLossCount = 0;
long long IdunClient::packetLossFirstSeqnum = 0;
int IdunClient::clientSocketFd = -1;
struct sockaddr_in IdunClient::serverAddress;
bool IdunClient::lastConnected = false;
double IdunClient::lastBuildUIDWarning = -1;
idun::proto::JoystickOutputs IdunClient::lastJoystickOutputs[HAL_kMaxJoysticks];
IdunPing* IdunClient::ping = nullptr;

std::mutex IdunClient::socketAccessMutex;
std::mutex IdunClient::outputsAccessMutex;
std::mutex IdunClient::inputsAccessMutex;

frc::Timer IdunClient::canReadTimer{};
idun::proto::RobotInputs IdunClient::inputs = idun::proto::RobotInputs();
idun::proto::RobotOutputs IdunClient::outputs = idun::proto::RobotOutputs();
idun::proto::RobotOutputs IdunClient::newOutputs = idun::proto::RobotOutputs();
units::time::millisecond_t IdunClient::outputCycleTracer = 0_ms;
std::mutex IdunClient::outputCycleTracerMutex;

// Main function for the client thread
void IdunClient::runClient(std::function<void()> applyOutputs) {
  std::vector<uint8_t> receiveBuffer(IdunNetworkConstants::maxPacketSizeBytes);
  frc::SetCurrentThreadPriority(true, 1);

  // Set receive timeout on the socket
  struct timeval timeoutSetting;
  timeoutSetting.tv_sec = 1;
  timeoutSetting.tv_usec = 0;
  if (setsockopt(clientSocketFd, SOL_SOCKET, SO_RCVTIMEO, &timeoutSetting,
                 sizeof(timeoutSetting)) < 0) {
    perror("[IdunClient] setsockopt SO_RCVTIMEO failed");
    return;  // Exit thread
  }

  std::cout << "[IdunClient] Socket ready, waiting for server..." << std::endl;

  while (true) {
    ssize_t bytesReceived = recvfrom(clientSocketFd, receiveBuffer.data(),
                                     receiveBuffer.size(), 0, nullptr, nullptr);

    if (bytesReceived > 0) {
      // Parse outputs
      if (!newOutputs.ParseFromArray(receiveBuffer.data(), bytesReceived)) {
        std::cerr << "[IdunClient] Failed to parse outputs message"
                  << std::endl;
        continue;
      }

      // Check build UID
      if (frc::RobotBase::IsReal() && newOutputs.builduid() != buildUID) {
        if (frc::Timer::GetFPGATimestamp().value() - lastBuildUIDWarning >
            1.0) {
          std::cerr << "[IdunClient] Server and client UIDs do not match, "
                       "ignoring message (!!!)"
                    << std::endl;
          lastBuildUIDWarning = frc::Timer::GetFPGATimestamp().value();
        }
        continue;
      }

      // Measure packet loss
      if (!isConnected()) {
        // Receiving first message
        packetLossCount = 0;
        packetLossFirstSeqnum = newOutputs.seqnum();
      } else if (newOutputs.seqnum() > lastPacketSeqnum) {
        packetLossCount += newOutputs.seqnum() - lastPacketSeqnum - 1;
      }
      lastPacketSeqnum = newOutputs.seqnum();
      int packetCountTotal = newOutputs.seqnum() - packetLossFirstSeqnum + 1;
      if (packetCountTotal > 100) {
        IdunLogger::logPacketLossRatio((double)packetLossCount /
                                       packetCountTotal);
      }

      // Log outputs
      std::string outputsStr{
          reinterpret_cast<const char*>(receiveBuffer.data()),
          (size_t)bytesReceived};
      idun::IdunLogger::logOutputs(newOutputs.seqnum(), outputsStr);
      idun::IdunLogger::logMacMiniUIDs(newOutputs.builduid(),
                                       newOutputs.launchuid());

      // Update timestamp on successful receive
      lastPacketTimestamp.store(frc::Timer::GetFPGATimestamp().value());

      // Apply temporary outputs
      {
        std::lock_guard<std::mutex> lock(outputsAccessMutex);
        outputs = newOutputs;
      }

      // Run outputs callback
      const auto applyOutputsStart = frc::Timer::GetTimestamp();
      if (outputs.restartflag()) {
        std::cout << "********** Robot program restarting **********"
                  << std::endl;
        std::system("killall frcUserProgram > /dev/null 2>&1 &");
        return;
      }
      applyJoystickOutputs();
      applyOutputs();
      const auto applyOutputsEnd = frc::Timer::GetTimestamp();
      {
        std::lock_guard<std::mutex> lock(outputCycleTracerMutex);
        outputCycleTracer = applyOutputsEnd - applyOutputsStart;
      }
    } else if (bytesReceived < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        // A real error occurred, not just a timeout
        perror("[IdunClient] Error receiving packet");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }
}

void IdunClient::start(std::function<void()> applyOutputs) {
  if (started) return;
  started = true;

  // Choose launch UID
  std::random_device randomDevice;
  std::mt19937_64 randomGenerator(randomDevice());
  std::uniform_int_distribution<long long> randomDistribution(
      std::numeric_limits<long long>::min(),
      std::numeric_limits<long long>::max());
  launchUID = randomDistribution(randomGenerator);

  // Start ping
  ping = new IdunPing();

  // Create UDP socket
  clientSocketFd = socket(AF_INET, SOCK_DGRAM, 0);
  if (clientSocketFd < 0) {
    perror("[IdunClient] FATAL: Socket creation failed");
    return;
  }

  // Prepare server address
  serverAddress.sin_family = AF_INET;
  serverAddress.sin_port = htons(IdunNetworkConstants::serverPort);
  if (inet_pton(AF_INET,
                std::string(frc::RobotBase::IsReal()
                                ? IdunNetworkConstants::serverAddressReal
                                : IdunNetworkConstants::serverAddressSim)
                    .c_str(),
                &serverAddress.sin_addr) <= 0) {
    perror("[IdunClient] FATAL: Invalid server address");
    close(clientSocketFd);
    clientSocketFd = -1;
    return;
  }

  // Start the client thread
  clientThread = std::thread(runClient, applyOutputs);
  if (clientThread.joinable()) {
    clientThread.detach();
  }
  canReadTimer.Restart();
}

bool IdunClient::isConnected() {
  if (!started || lastPacketTimestamp.load() < 0) {
    return false;
  }
  return (frc::Timer::GetFPGATimestamp().value() - lastPacketTimestamp.load()) <
         (static_cast<double>(IdunNetworkConstants::timeoutMs) / 1000.0);
}

void IdunClient::setSubsystemInputs(std::string_view name,
                                    idun::proto::IOData& sub_inputs) {
  std::lock_guard<std::mutex> lock(inputsAccessMutex);
  (*inputs.mutable_subsystem())[name] = sub_inputs;
}

void IdunClient::setCANStatus(std::string_view name,
                              idun::proto::CANStatus& status) {
  std::lock_guard<std::mutex> lock(inputsAccessMutex);
  (*inputs.mutable_rio()->mutable_canstatus())[name] = status;
}

void IdunClient::applyJoystickOutputs() {
  const auto& defaultJoystickOutputs = idun::proto::JoystickOutputs();
  google::protobuf::util::MessageDifferencer differencer;

  std::lock_guard<std::mutex> lock(outputsAccessMutex);
  for (int id = 0; id < HAL_kMaxJoysticks; id++) {
    const auto& joystickOutputs = isConnected() && id < outputs.joystick_size()
                                      ? outputs.joystick(id)
                                      : defaultJoystickOutputs;
    if (!differencer.Equals(joystickOutputs, lastJoystickOutputs[id])) {
      HAL_SetJoystickOutputs(id, joystickOutputs.outputs(),
                             joystickOutputs.leftrumble(),
                             joystickOutputs.rightrumble());
      lastJoystickOutputs[id] = joystickOutputs;
    }
  }
}

const idun::proto::IOData IdunClient::getSubsystemOutputs(std::string_view name,
                                                          bool& success) {
  std::lock_guard<std::mutex> lock(outputsAccessMutex);
  success = outputs.subsystem().contains(name);
  if (success) {
    return outputs.subsystem().at(name);
  } else {
    return idun::proto::IOData();
  }
}

void IdunClient::updateGyroOffset(frc::Rotation2d& offset) {
  if (isConnected()) {
    std::lock_guard<std::mutex> lock(outputsAccessMutex);
    offset = frc::Rotation2d(units::radian_t{outputs.gyrooffset()});
  }
}

void IdunClient::processOutgoing(TracerTimes& activeTracerTimes,
                                 const TracerTimes& loggedTracerTimes,
                                 const bool isLocalDrive) {
  std::lock_guard<std::mutex> lock(inputsAccessMutex);

  // Print connection status
  const bool connected = isConnected();
  if (connected && !lastConnected) {
    std::cout << "[IdunClient] Connected to server "
              << (frc::RobotBase::IsReal()
                      ? IdunNetworkConstants::serverAddressReal
                      : IdunNetworkConstants::serverAddressSim)
              << ":" << IdunNetworkConstants::serverPort << std::endl;
  } else if (!connected && lastConnected) {
    std::cout << "[IdunClient] Server timed out, waiting for reconnection..."
              << std::endl;
  }
  lastConnected = connected;

  // Update built-in inputs
  const auto builtInInputsStart = frc::Timer::GetTimestamp();
  auto rioInputs = inputs.mutable_rio();
  auto dsInputs = inputs.mutable_ds();
  auto tracerInputs = inputs.mutable_tracer();

  inputs.set_builduid(buildUID);
  inputs.set_launchuid(launchUID);
  inputs.set_seqnum(seqnum++);
  inputs.set_islocaldrive(isLocalDrive);

  rioInputs->set_systemactive(frc::RobotController::IsSysActive());
  rioInputs->set_brownedout(frc::RobotController::IsBrownedOut());
  rioInputs->set_commsdisablecount(
      frc::RobotController::GetCommsDisableCount());
  rioInputs->set_rslstate(frc::RobotController::GetRSLState());
  rioInputs->set_batteryvoltage(frc::RobotController::GetInputVoltage());
  rioInputs->set_batterycurrent(frc::RobotController::GetInputCurrent());

  idun::proto::RailStatus* rail3v3 = rioInputs->mutable_rail3v3();
  rail3v3->set_voltage(frc::RobotController::GetVoltage3V3());
  rail3v3->set_current(frc::RobotController::GetCurrent3V3());
  rail3v3->set_active(frc::RobotController::GetEnabled3V3());
  rail3v3->set_currentfaults(frc::RobotController::GetFaultCount3V3());

  idun::proto::RailStatus* rail5v = rioInputs->mutable_rail5v();
  rail5v->set_voltage(frc::RobotController::GetVoltage5V());
  rail5v->set_current(frc::RobotController::GetCurrent5V());
  rail5v->set_active(frc::RobotController::GetEnabled5V());
  rail5v->set_currentfaults(frc::RobotController::GetFaultCount5V());

  idun::proto::RailStatus* rail6v = rioInputs->mutable_rail6v();
  rail6v->set_voltage(frc::RobotController::GetVoltage6V());
  rail6v->set_current(frc::RobotController::GetCurrent6V());
  rail6v->set_active(frc::RobotController::GetEnabled6V());
  rail6v->set_currentfaults(frc::RobotController::GetFaultCount6V());

  if (canReadTimer.AdvanceIfElapsed(0.5_s)) {
    const auto canStatus = frc::RobotController::GetCANStatus();
    idun::proto::CANStatus canProto;
    canProto.set_utilization(canStatus.percentBusUtilization);
    canProto.set_offcount(canStatus.busOffCount);
    canProto.set_txfullcount(canStatus.txFullCount);
    canProto.set_receiveerrorcount(canStatus.receiveErrorCount);
    canProto.set_transmiterrorcount(canStatus.transmitErrorCount);
    (*rioInputs->mutable_canstatus())["RIO"] = canProto;
  }
  rioInputs->set_epochtimemicros(wpi::GetSystemTime());

  int32_t status = 0;
  dsInputs->set_alliancestation(HAL_GetAllianceStation(&status));
  dsInputs->set_eventname(frc::DriverStation::GetEventName());
  dsInputs->set_gamespecificmessage(
      frc::DriverStation::GetGameSpecificMessage());
  dsInputs->set_matchnumber(frc::DriverStation::GetMatchNumber());
  dsInputs->set_replaynumber(frc::DriverStation::GetReplayNumber());
  dsInputs->set_matchtype(frc::DriverStation::GetMatchType());
  dsInputs->set_matchtime(frc::DriverStation::GetMatchTime().value());

  HAL_ControlWord controlWord;
  HAL_GetControlWord(&controlWord);
  auto controlWordProto = dsInputs->mutable_controlword();
  controlWordProto->set_enabled(controlWord.enabled);
  controlWordProto->set_autonomous(controlWord.autonomous);
  controlWordProto->set_test(controlWord.test);
  controlWordProto->set_estop(controlWord.eStop);
  controlWordProto->set_fmsattached(controlWord.fmsAttached);
  controlWordProto->set_dsattached(controlWord.dsAttached);

  while (dsInputs->joystick_size() < HAL_kMaxJoysticks) {
    dsInputs->add_joystick();
  }
  for (int id = 0; id < HAL_kMaxJoysticks; id++) {
    auto joystick = dsInputs->mutable_joystick(id);
    joystick->set_name(frc::DriverStation::GetJoystickName(id));
    joystick->set_type(frc::DriverStation::GetJoystickType(id));
    joystick->set_isxbox(frc::DriverStation::GetJoystickIsXbox(id));
    joystick->set_buttoncount(frc::DriverStation::GetStickButtonCount(id));
    joystick->set_buttons(frc::DriverStation::GetStickButtons(id));
    joystick->set_povcount(frc::DriverStation::GetStickPOVCount(id));
    joystick->clear_povvalue();
    for (int i = 0; i < frc::DriverStation::GetStickPOVCount(id); i++) {
      joystick->add_povvalue(frc::DriverStation::GetStickPOV(id, i));
    }
    joystick->set_axiscount(frc::DriverStation::GetStickAxisCount(id));
    joystick->clear_axistype();
    joystick->clear_axisvalue();
    for (int i = 0; i < frc::DriverStation::GetStickAxisCount(id); i++) {
      joystick->add_axistype(frc::DriverStation::GetJoystickAxisType(id, i));
      joystick->add_axisvalue(frc::DriverStation::GetStickAxis(id, i));
    }
  }

  tracerInputs->set_inputcyclems(loggedTracerTimes.inputCycle.value());
  tracerInputs->set_dsalertms(loggedTracerTimes.dsAlert.value());
  tracerInputs->set_disconnectedstopms(
      loggedTracerTimes.disconnectedStop.value());
  tracerInputs->set_phoenixrefreshms(loggedTracerTimes.phoenixRefresh.value());
  tracerInputs->set_subsysteminputsms(
      loggedTracerTimes.subsystemInputs.value());
  tracerInputs->set_localdrivems(loggedTracerTimes.localDrive.value());
  tracerInputs->set_builtininputsms(loggedTracerTimes.builtInInputs.value());
  tracerInputs->set_serializems(loggedTracerTimes.serialize.value());
  tracerInputs->set_transmitms(loggedTracerTimes.transmit.value());
  tracerInputs->set_outputcyclems(loggedTracerTimes.outputCycle.value());

  {
    std::lock_guard<std::mutex> lock(outputCycleTracerMutex);
    activeTracerTimes.outputCycle = outputCycleTracer;
  }

  // Serialize inputs
  const auto serializeStart = frc::Timer::GetTimestamp();
  std::string serializedInputData;
  if (!inputs.SerializeToString(&serializedInputData)) {
    std::cerr << "[IdunClient] Failed to serialize inputs message" << std::endl;
    activeTracerTimes.builtInInputs = serializeStart - builtInInputsStart;
    activeTracerTimes.serialize = frc::Timer::GetTimestamp() - serializeStart;
    activeTracerTimes.transmit = 0_ms;
    return;
  }
  idun::IdunLogger::logInputs(inputs.seqnum(), serializedInputData);

  // Send data to server
  const auto transmitStart = frc::Timer::GetTimestamp();
  std::lock_guard<std::mutex> socketLock(socketAccessMutex);
  if (clientSocketFd != -1) {
    ssize_t bytesSent =
        sendto(clientSocketFd, serializedInputData.data(),
               serializedInputData.length(), 0,
               (struct sockaddr*)&serverAddress, sizeof(serverAddress));
    if (bytesSent < 0) {
      perror("[IdunClient] Error sending packet");
    }
  }

  // Log connection status
  idun::IdunLogger::logConnected(isConnected());

  // Log roboRIO UIDs
  idun::IdunLogger::logRoboRioUIDs(buildUID, launchUID);

  // Update tracer times
  activeTracerTimes.builtInInputs = serializeStart - builtInInputsStart;
  activeTracerTimes.serialize = transmitStart - serializeStart;
  activeTracerTimes.transmit = frc::Timer::GetTimestamp() - transmitStart;
}

}  // namespace idun
