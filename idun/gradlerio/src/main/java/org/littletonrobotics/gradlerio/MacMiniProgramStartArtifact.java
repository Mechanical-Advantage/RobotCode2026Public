// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.gradlerio;

import edu.wpi.first.deployutils.deploy.artifact.AbstractArtifact;
import edu.wpi.first.deployutils.deploy.context.DeployContext;
import edu.wpi.first.gradlerio.deploy.DeployStage;
import edu.wpi.first.gradlerio.deploy.StagedDeployTarget;
import edu.wpi.first.gradlerio.deploy.roborio.*;
import javax.inject.Inject;

public class MacMiniProgramStartArtifact extends AbstractArtifact {
  @Inject
  public MacMiniProgramStartArtifact(String name, StagedDeployTarget target) {
    super(name, target);
    target.setDeployStage(this, DeployStage.ProgramStart);
  }

  @Override
  public void deploy(DeployContext ctx) {
    ctx.execute("sync");
    ctx.execute("sudo launchctl load /Library/LaunchDaemons/org.littletonrobotics.robot.plist");
  }
}
