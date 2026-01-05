// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.util.controllers;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TriggerUtil {
  private TriggerUtil() {}

  /**
   * Constantly starts the given command while the button is held.
   *
   * <p>{@link Command#schedule()} will be called repeatedly while the trigger is active, and will
   * be canceled when the trigger becomes inactive.
   *
   * @param command The command to start.
   * @return This trigger, so calls can be chained.
   */
  public static void whileTrueContinuous(Trigger trigger, final Command command) {
    CommandScheduler.getInstance()
        .getDefaultButtonLoop()
        .bind(
            new Runnable() {
              private boolean m_pressedLast = trigger.getAsBoolean();

              @Override
              public void run() {
                boolean pressed = trigger.getAsBoolean();

                if (pressed) {
                  CommandScheduler.getInstance().schedule(command);
                } else if (m_pressedLast) {
                  CommandScheduler.getInstance().cancel(command);
                }

                m_pressedLast = pressed;
              }
            });
  }

  /**
   * Tracker that activates only when a button is pressed twice quickly.
   *
   * @param baseTrigger The trigger to wrap.
   * @return The new trigger that activates on double press.
   */
  public static Trigger doublePress(Trigger baseTrigger) {
    var tracker = new DoublePressTracker(baseTrigger);
    return new Trigger(tracker::get);
  }

  /** Tracker that activates only when a button is pressed twice quickly. */
  private static class DoublePressTracker {
    // How long after the first press does the second need to occur?
    public static final double maxLengthSecs = 0.4;

    private final Trigger trigger;
    private final Timer resetTimer = new Timer();
    private DoublePressState state = DoublePressState.IDLE;

    private DoublePressTracker(Trigger baseTrigger) {
      trigger = baseTrigger;
    }

    private boolean get() {
      boolean pressed = trigger.getAsBoolean();
      switch (state) {
        case IDLE:
          if (pressed) {
            state = DoublePressState.FIRST_PRESS;
            resetTimer.reset();
            resetTimer.start();
          }
          break;
        case FIRST_PRESS:
          if (!pressed) {
            if (resetTimer.hasElapsed(maxLengthSecs)) {
              reset();
            } else {
              state = DoublePressState.FIRST_RELEASE;
            }
          }
          break;
        case FIRST_RELEASE:
          if (pressed) {
            state = DoublePressState.SECOND_PRESS;
          } else if (resetTimer.hasElapsed(maxLengthSecs)) {
            reset();
          }
          break;
        case SECOND_PRESS:
          if (!pressed) {
            reset();
          }
      }
      return state == DoublePressState.SECOND_PRESS;
    }

    private void reset() {
      state = DoublePressState.IDLE;
      resetTimer.stop();
    }

    private enum DoublePressState {
      IDLE,
      FIRST_PRESS,
      FIRST_RELEASE,
      SECOND_PRESS
    }
  }
}
