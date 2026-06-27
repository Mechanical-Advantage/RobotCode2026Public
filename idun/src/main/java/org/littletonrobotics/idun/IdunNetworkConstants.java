// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.idun;

@IdunConstants
public class IdunNetworkConstants {
  public static final int serverPort = 6328;
  public static final String serverAddressReal = "10.63.28.10";
  public static final String serverAddressSim = "127.0.0.1";
  public static final int timeoutMs = 200;
  public static final int maxPacketSizeBytes = 16384;
}
