// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026;

import org.littletonrobotics.idun.IdunConstants;
import org.littletonrobotics.idun.IdunPlatform;

@IdunConstants
public final class Constants {
  private static final RobotType robot = RobotType.DARWIN;
  public static final boolean tuningMode = false;

  public static final double loopPeriodSecs = 0.005;
  public static final double loopPeriodWatchdogSecs = 0.1;

  public static Mode getMode() {
    return switch (getRobot()) {
      case DARWIN, ALPHABOT -> IdunPlatform.isRobot ? Mode.REAL : Mode.REPLAY;
      case SIMBOT -> Mode.SIM;
    };
  }

  public static RobotType getRobot() {
    boolean isDesktop = Constants.disableHAL || IdunPlatform.isDesktop;
    return isDesktop && System.getenv("SIMBOT") != null ? RobotType.SIMBOT : robot;
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum RobotType {
    DARWIN,
    ALPHABOT,
    SIMBOT
  }

  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  /** Checks whether the correct robot is selected when deploying. */
  public static class CheckDeploy {
    public static void main(String... args) {
      if (robot == RobotType.SIMBOT) {
        System.err.println("Cannot deploy, invalid robot selected: " + robot);
        System.exit(1);
      }
    }
  }

  /** Checks that the default robot is selected and tuning mode is disabled. */
  public static class CheckPullRequest {
    public static void main(String... args) {
      if (robot != RobotType.DARWIN || tuningMode) {
        System.err.println("Do not merge, non-default constants are configured.");
        System.exit(1);
      }
    }
  }
}
