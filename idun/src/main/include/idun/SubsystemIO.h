// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <IdunComms.pb.h>

namespace idun {

class SubsystemIO {
 public:
  SubsystemIO(std::string_view name);

  /** Update inputs and send the new state to the Idun client. */
  void inputPeriodic();

  /** Read outputs from the Idun client and apply to the hardware. If
   * disconnected from the server, bring the hardware to a safe state. */
  void outputPeriodic();

  /** Set the hardware to a safe state. */
  virtual void stop() = 0;

 protected:
  /** Pass through to the "updateInputs" method, translating the provided data
   * to a Protobuf representation. */
  virtual void updateInputsProto(idun::proto::IOData& inputs) = 0;

  /** Pass through to the "applyOutputs" method, translating the provided data
   * from a Protobuf representation. */
  virtual void applyOutputsProto(const idun::proto::IOData& outputs) = 0;

  /** The name of the subsystem. */
  std::string name;

 private:
  idun::proto::IOData inputs;
};

};  // namespace idun
