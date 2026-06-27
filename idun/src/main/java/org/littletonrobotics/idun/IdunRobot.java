// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.idun;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.NotifierJNI;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotController;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.util.List;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.ConsoleSource;
import org.littletonrobotics.junction.Logger;

/**
 * IdunRobot implements the IterativeRobotBase robot program framework. This class is based on
 * AdvantageKit's LoggedRobot class, with additional capabilities like Idun client loop cycle
 * synchronization and improved exception logging (since AdvantageKit provides all persistent
 * storage of console logs for the Mac Mini robot program).
 *
 * <p>The IdunRobot class is intended to be subclassed by a user creating a robot program.
 */
public class IdunRobot extends IterativeRobotBase {
  public static final double defaultPeriodSecs = 0.02;
  private final int notifier = NotifierJNI.initializeNotifier();
  private final long periodUs;
  private long nextCycleUs = 0;
  private final GcStatsCollector gcStatsCollector = new GcStatsCollector();

  private TimingMode timingMode = TimingMode.FIXED;

  /** Constructor for LoggedRobot. */
  protected IdunRobot() {
    this(defaultPeriodSecs);
  }

  /**
   * Constructor for LoggedRobot.
   *
   * @param period Period in seconds.
   */
  protected IdunRobot(double period) {
    super(period);
    this.periodUs = (long) (period * 1000000);
    NotifierJNI.setNotifierName(notifier, "IdunRobot");
    Logger.AdvancedHooks.disableRobotBaseCheck();

    // Configure Mac Mini console logging
    if (IdunPlatform.isRobot) {
      Logger.AdvancedHooks.setConsoleSource(
          new ConsoleSource.RoboRIO() {
            @Override
            public String getFilePath() {
              return "console.log";
            }
          });
    }

    // Usage reporting (mostly pointless)
    HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_AdvantageKit);
    HAL.report(
        tResourceType.kResourceType_LoggingFramework, tInstances.kLoggingFramework_AdvantageKit);
  }

  @Override
  public void close() {
    NotifierJNI.stopNotifier(notifier);
    NotifierJNI.cleanNotifier(notifier);
    super.close();
  }

  /** Provide an alternate "main loop" via startCompetition(). */
  @Override
  public void startCompetition() {
    try {
      // Robot init methods
      robotInit();
      if (IdunPlatform.isDesktop) {
        simulationInit();
      }
      long initEnd = RobotController.getFPGATime(); // Includes Robot constructor and robotInit

      // Register auto logged outputs
      AutoLogOutputManager.addObject(this);

      // Save data from init cycle
      Logger.AdvancedHooks.invokePeriodicAfterUser(initEnd, 0);

      // Tell the DS that the robot is ready to be enabled
      System.out.println("********** Robot program startup complete **********");
      DriverStationJNI.observeUserProgramStarting();

      // Loop forever, calling the appropriate mode-dependent function
      while (true) {
        // Fallback from sync to fixed if Idun is disconnected
        TimingMode appliedTimingMode = timingMode;
        if (timingMode == TimingMode.SYNC && !IdunServer.isConnected()) {
          appliedTimingMode = TimingMode.FIXED;
        }

        // Delay until next cycle
        switch (appliedTimingMode) {
          case FIXED -> {
            long currentTimeUs = RobotController.getFPGATime();
            if (nextCycleUs < currentTimeUs) {
              // Loop overrun, start next cycle immediately
              nextCycleUs = currentTimeUs;
            } else {
              // Wait before next cycle
              NotifierJNI.updateNotifierAlarm(notifier, nextCycleUs);
              if (NotifierJNI.waitForNotifierAlarm(notifier) == 0L) {
                // Break the loop if the notifier was stopped
                Logger.end();
                break;
              }
            }
            nextCycleUs += periodUs;
          }

          case SYNC -> {
            // Timeout is the full period, but starting after the last loop cycle ended. This
            // makes it slightly slower than the intended frequency in practice, which ensures
            // that we usually stay in sync with new inputs rather than timing out.
            IdunServer.waitForInputs(periodUs);
          }

          case FAST -> {
            // No delay, run as fast as possible
          }
        }

        long periodicBeforeStart = RobotController.getFPGATime();
        Logger.AdvancedHooks.invokePeriodicBeforeUser();
        long userCodeStart = RobotController.getFPGATime();
        loopFunc();
        long userCodeEnd = RobotController.getFPGATime();

        gcStatsCollector.update();
        Logger.AdvancedHooks.invokePeriodicAfterUser(
            userCodeEnd - userCodeStart, userCodeStart - periodicBeforeStart);
      }
    } catch (Exception exception) {
      StringWriter stringWriter = new StringWriter();
      exception.printStackTrace(new PrintWriter(stringWriter));
      Logger.AdvancedHooks.invokePeriodicAfterUser(0, 0, stringWriter.toString());
      Logger.end();
      throw exception;
    }
  }

  /** Ends the main loop in startCompetition(). */
  @Override
  public void endCompetition() {
    NotifierJNI.stopNotifier(notifier);
  }

  /** Sets the timing mode. */
  public void setTimingMode(TimingMode timingMode) {
    this.timingMode = timingMode;
  }

  public static enum TimingMode {
    /** Run at a fixed period, similar to standard TimedRobot. */
    FIXED,

    /**
     * Synchronize loop cycles with the Idun client (roboRIO). Falls back to {@link
     * org.littletonrobotics.idun.IdunRobot.TimingMode#FIXED fixed} when disconnected.
     */
    SYNC,

    /** Run loop cycles as quickly as possible. */
    FAST
  }

  private static final class GcStatsCollector {
    private List<GarbageCollectorMXBean> gcBeans = ManagementFactory.getGarbageCollectorMXBeans();
    private final long[] lastTimes = new long[gcBeans.size()];
    private final long[] lastCounts = new long[gcBeans.size()];

    public void update() {
      long accumTime = 0;
      long accumCounts = 0;
      for (int i = 0; i < gcBeans.size(); i++) {
        long gcTime = gcBeans.get(i).getCollectionTime();
        long gcCount = gcBeans.get(i).getCollectionCount();
        accumTime += gcTime - lastTimes[i];
        accumCounts += gcCount - lastCounts[i];

        lastTimes[i] = gcTime;
        lastCounts[i] = gcCount;
      }

      Logger.recordOutput("LoggedRobot/GCTimeMS", (double) accumTime);
      Logger.recordOutput("LoggedRobot/GCCounts", (double) accumCounts);
    }
  }
}
