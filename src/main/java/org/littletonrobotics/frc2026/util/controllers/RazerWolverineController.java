// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.util.controllers;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Interface for 6328's custom bindings on a Razer Wolverine controller. */
public class RazerWolverineController extends CommandXboxController {
  public RazerWolverineController(int port) {
    super(port);
  }

  /**
   * Constructs a Trigger instance around the left claw button's digital signal.
   *
   * @return a Trigger instance representing the left claw button's digital signal attached to the
   *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #back(EventLoop)
   */
  public Trigger leftClaw() {
    return back();
  }

  /**
   * Constructs a Trigger instance around the right claw button's digital signal.
   *
   * @return a Trigger instance representing the right claw button's digital signal attached to the
   *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #start(EventLoop)
   */
  public Trigger rightClaw() {
    return start();
  }

  /**
   * Constructs a Trigger instance around the upper left paddle button's digital signal.
   *
   * @return a Trigger instance representing the upper left paddle button's digital signal attached
   *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #start(EventLoop)
   */
  public Trigger upperLeftPaddle() {
    return povUp().or(povUpLeft()).or(povUpRight());
  }

  /**
   * Constructs a Trigger instance around the lower left paddle button's digital signal.
   *
   * @return a Trigger instance representing the lower left paddle button's digital signal attached
   *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #start(EventLoop)
   */
  public Trigger lowerLeftPaddle() {
    return povDown().or(povDownLeft()).or(povDownRight());
  }

  /**
   * Constructs a Trigger instance around the upper right paddle button's digital signal.
   *
   * @return a Trigger instance representing the upper right paddle button's digital signal attached
   *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #start(EventLoop)
   */
  public Trigger upperRightPaddle() {
    return povLeft().or(povUpLeft()).or(povDownLeft());
  }

  /**
   * Constructs a Trigger instance around the lower right paddle button's digital signal.
   *
   * @return a Trigger instance representing the lower right paddle button's digital signal attached
   *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #start(EventLoop)
   */
  public Trigger lowerRightPaddle() {
    return povRight().or(povUpRight()).or(povDownRight());
  }
}
