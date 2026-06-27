// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "idun/IdunLogManager.h"

// Copyright (c) 2009-2026 FIRST and other WPILib contributors
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// - Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
// - Neither the name of FIRST, WPILib, nor the names of other WPILib
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY FIRST AND OTHER WPILIB CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY NONINFRINGEMENT AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL FIRST OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <fmt/chrono.h>
#include <frc/Errors.h>
#include <hal/FRCUsageReporting.h>
#include <networktables/NetworkTableInstance.h>
#include <wpi/DataLog.h>
#include <wpi/DataLogBackgroundWriter.h>
#include <wpi/FileLogger.h>
#include <wpi/SafeThread.h>
#include <wpi/StringExtras.h>
#include <wpi/fs.h>
#include <wpi/print.h>
#include <wpi/timestamp.h>

#include <algorithm>
#include <cctype>
#include <ctime>
#include <random>
#include <string>
#include <vector>

#include "frc/DriverStation.h"
#include "frc/Filesystem.h"
#include "frc/RobotBase.h"
#include "frc/RobotController.h"

using namespace frc;
using namespace idun;

namespace {

struct Thread final : public wpi::SafeThread {
  Thread(std::string_view dir, std::string_view filename, double period);
  ~Thread() override;

  void Main() final;

  std::string logDir;
  bool filenameOverride;
  wpi::log::DataLogBackgroundWriter log;
};

struct Instance {
  Instance(std::string_view dir, std::string_view filename, double period);
  wpi::SafeThreadOwner<Thread> owner;
};

}  // namespace

// if less than this much free space, delete log files until there is this much
// free space OR there are this many files remaining.
static constexpr uintmax_t kFreeSpaceThreshold = 50000000;
static constexpr int kFileCountThreshold = 10;

static std::string makeLogDir(std::string_view dir) {
  if (!dir.empty()) {
    return std::string{dir};
  }
#ifdef __FRC_ROBORIO__
  // prefer a mounted USB drive if one is accessible
  std::error_code ec;
  auto s = fs::status("/u", ec);
  if (!ec && fs::is_directory(s) &&
      (s.permissions() & fs::perms::others_write) != fs::perms::none) {
    fs::create_directory("/u/logs", ec);
    return "/u/logs";
  }
  if (RobotBase::GetRuntimeType() == kRoboRIO) {
    FRC_ReportWarning(
        "IdunLogManager: Logging to RoboRIO 1 internal storage is "
        "not recommended! Plug in a FAT32 formatted flash drive!");
  }
  fs::create_directory("/home/lvuser/logs", ec);
  return "/home/lvuser/logs";
#else
  std::string logDir = filesystem::GetOperatingDirectory() + "/logs";
  std::error_code ec;
  fs::create_directory(logDir, ec);
  return logDir;
#endif
}

static std::string makeLogFilename(std::string_view filenameOverride) {
  if (!filenameOverride.empty()) {
    return std::string{filenameOverride};
  }
  static std::random_device dev;
  static std::mt19937 rng(dev());
  std::uniform_int_distribution<int> dist(0, 15);
  const char* v = "0123456789abcdef";
  std::string filename = "idun_";
  for (int i = 0; i < 16; i++) {
    filename += v[dist(rng)];
  }
  filename += ".wpilog";
  return filename;
}

Thread::Thread(std::string_view dir, std::string_view filename, double period)
    : logDir{dir},
      filenameOverride{!filename.empty()},
      log{dir, makeLogFilename(filename), period} {}

Thread::~Thread() {}

void Thread::Main() {
  int dsAttachCount = 0;
  int fmsAttachCount = 0;
  bool dsRenamed = filenameOverride;
  bool fmsRenamed = filenameOverride;
  int sysTimeCount = 0;
  wpi::log::IntegerLogEntry sysTimeEntry{
      log, "systemTime",
      "{\"source\":\"IdunLogManager\",\"format\":\"time_t_us\"}"};

  wpi::Event newDataEvent;
  DriverStation::ProvideRefreshedDataEventHandle(newDataEvent.GetHandle());

  for (;;) {
    bool timedOut = false;
    wpi::WaitForObject(newDataEvent.GetHandle(), 0.25, &timedOut);
    if (!m_active) {
      break;
    }

    if (!dsRenamed) {
      // track DS attach
      if (DriverStation::IsDSAttached()) {
        ++dsAttachCount;
      } else {
        dsAttachCount = 0;
      }
      if (dsAttachCount > 50) {  // 1 second
        if (RobotController::IsSystemTimeValid()) {
          std::time_t now = std::time(nullptr);
          auto tm = std::localtime(&now);
          log.SetFilename(fmt::format("idun_{:%y-%m-%d_%H-%M-%S}.wpilog", *tm));
          dsRenamed = true;
        } else {
          dsAttachCount = 0;  // wait a bit and try again
        }
      }
    }

    if (!fmsRenamed) {
      // track FMS attach
      if (DriverStation::IsFMSAttached()) {
        ++fmsAttachCount;
      } else {
        fmsAttachCount = 0;
      }
      if (fmsAttachCount > 250) {  // 5 seconds
        // match info comes through TCP, so we need to double-check we've
        // actually received it
        auto matchType = DriverStation::GetMatchType();
        if (matchType != DriverStation::kNone) {
          // rename per match info
          char matchTypeChar;
          switch (matchType) {
            case DriverStation::kPractice:
              matchTypeChar = 'p';
              break;
            case DriverStation::kQualification:
              matchTypeChar = 'q';
              break;
            case DriverStation::kElimination:
              matchTypeChar = 'e';
              break;
            default:
              matchTypeChar = '_';
              break;
          }
          std::time_t now = std::time(nullptr);
          std::string eventName = DriverStation::GetEventName();
          std::transform(eventName.begin(), eventName.end(), eventName.begin(),
                         [](unsigned char c) { return std::tolower(c); });
          log.SetFilename(fmt::format(
              "idun_{:%y-%m-%d_%H-%M-%S}_{}_{}{}.wpilog", *std::localtime(&now),
              eventName, matchTypeChar, DriverStation::GetMatchNumber()));
          fmsRenamed = true;
          dsRenamed = true;  // don't override FMS rename
        }
      }
    }

    // Write system time every ~5 seconds
    ++sysTimeCount;
    if (sysTimeCount >= 250) {
      sysTimeCount = 0;
      if (RobotController::IsSystemTimeValid()) {
        sysTimeEntry.Append(wpi::GetSystemTime(), wpi::Now());
      }
    }
  }
  DriverStation::RemoveRefreshedDataEventHandle(newDataEvent.GetHandle());
}

Instance::Instance(std::string_view dir, std::string_view filename,
                   double period) {
  owner.Start(makeLogDir(dir), filename, period);
}

static Instance& GetInstance(std::string_view dir = "",
                             std::string_view filename = "",
                             double period = 0.25) {
  static Instance instance(dir, filename, period);
  if (!instance.owner) {
    instance.owner.Start(makeLogDir(dir), filename, period);
  }
  return instance;
}

void IdunLogManager::start(std::string_view dir, std::string_view filename,
                           double period) {
  GetInstance(dir, filename, period);
}

void IdunLogManager::stop() {
  auto& inst = GetInstance();
  inst.owner.GetThread()->log.Stop();
  inst.owner.Join();
}

wpi::log::DataLog& IdunLogManager::getLog() {
  return GetInstance().owner.GetThread()->log;
}

std::string IdunLogManager::getLogDir() {
  return GetInstance().owner.GetThread()->logDir;
}
