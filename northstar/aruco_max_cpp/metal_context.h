// Copyright (c) 2022-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <cstdint>
#include <vector>

class ArucoMaxMetalContext {
public:
  ArucoMaxMetalContext();
  ~ArucoMaxMetalContext();

  // Perform the adaptive threshold. Returns a pointer to the output binary
  // image. The output buffer is owned by ArucoMaxMetalContext and is valid
  // until the next call.
  uint8_t *adaptiveThreshold(const uint8_t *inImage, int width, int height,
                             int window_size, int threshold_val);

private:
  class Impl;
  Impl *pImpl;
};
