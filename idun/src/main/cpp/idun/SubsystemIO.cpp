// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include <Robot.h>
#include <idun/IdunClient.h>
#include <idun/SubsystemIO.h>

namespace idun {

SubsystemIO::SubsystemIO(std::string_view name) : name(name) {
  Robot::registerSubsystem(this);
}

void SubsystemIO::inputPeriodic() {
  updateInputsProto(inputs);
  IdunClient::setSubsystemInputs(name, inputs);
}

void SubsystemIO::outputPeriodic() {
  if (IdunClient::isConnected()) {
    bool success;
    const auto outputs = IdunClient::getSubsystemOutputs(name, success);
    if (success) {
      applyOutputsProto(outputs);
    }
  } else {
    stop();
  }
}

};  // namespace idun
