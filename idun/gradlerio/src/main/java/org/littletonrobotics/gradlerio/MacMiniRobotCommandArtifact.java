// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.gradlerio;

import edu.wpi.first.deployutils.deploy.artifact.FileArtifact;
import edu.wpi.first.deployutils.deploy.context.DeployContext;
import edu.wpi.first.gradlerio.deploy.DeployStage;
import edu.wpi.first.gradlerio.deploy.StagedDeployTarget;
import edu.wpi.first.gradlerio.deploy.roborio.*;
import java.util.function.Function;
import javax.inject.Inject;

public class MacMiniRobotCommandArtifact extends FileArtifact {

  private Function<DeployContext, String> startCommandFunc;

  @Inject
  public MacMiniRobotCommandArtifact(String name, StagedDeployTarget target) {
    super(name, target);

    target.setDeployStage(this, DeployStage.FileDeploy);
  }

  public Function<DeployContext, String> getStartCommandFunc() {
    return startCommandFunc;
  }

  public void setStartCommandFunc(Function<DeployContext, String> startCommandFunc) {
    this.startCommandFunc = startCommandFunc;
  }

  @Override
  public void deploy(DeployContext ctx) {
    String content = startCommandFunc.apply(ctx);

    ctx.execute("echo '" + content + "' > robotCommand");
    ctx.execute("chmod +x robotCommand");
  }
}
