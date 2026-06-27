// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.gradlerio;

import edu.wpi.first.deployutils.PathUtils;
import edu.wpi.first.deployutils.deploy.context.DeployContext;
import edu.wpi.first.gradlerio.deploy.DebuggableJavaArtifact;
import edu.wpi.first.gradlerio.deploy.DeployStage;
import edu.wpi.first.gradlerio.deploy.roborio.*;
import edu.wpi.first.gradlerio.wpi.WPIExtension;
import java.util.ArrayList;
import java.util.List;
import javax.inject.Inject;
import org.gradle.api.tasks.TaskProvider;
import org.gradle.api.tasks.bundling.Jar;

public class MacMiniJavaArtifact extends DebuggableJavaArtifact {
  private final MacMiniProgramStartArtifact programStartArtifact;
  private final MacMiniRobotCommandArtifact robotCommandArtifact;
  private final MacMiniJNILibraryArtifact nativeZipArtifact;

  private final List<String> jvmArgs = new ArrayList<>();
  private final List<String> arguments = new ArrayList<>();

  private final MacMini macMini;

  private GarbageCollectorType gcType = GarbageCollectorType.Other;

  private String javaCommand =
      "cd /Users/frc6328/robot && echo \"\" > console.log && sudo nice -n -20 sudo -u frc6328 /usr/bin/java";

  @Inject
  public MacMiniJavaArtifact(String name, MacMini target) {
    super(name, target);
    macMini = target;

    jvmArgs.add("--enable-native-access=ALL-UNNAMED");
    jvmArgs.add("--add-opens java.base/java.io=ALL-UNNAMED");
    jvmArgs.add("--sun-misc-unsafe-memory-access=allow");
    jvmArgs.add("-Djava.lang.invoke.stringConcat=BC_SB");
    jvmArgs.add("-Djava.library.path=" + MacMiniJNILibraryArtifact.libDeployDir);

    var debugConfiguration = target.getProject().getConfigurations().create("macMiniDebug");
    var releaseConfiguration = target.getProject().getConfigurations().create("macMiniRelease");

    programStartArtifact =
        target
            .getArtifacts()
            .create("programStart" + name, MacMiniProgramStartArtifact.class, art -> {});

    robotCommandArtifact =
        target
            .getArtifacts()
            .create(
                "robotCommand" + name,
                MacMiniRobotCommandArtifact.class,
                art -> {
                  art.setStartCommandFunc(this::generateStartCommand);
                  art.dependsOn(getJarProvider());
                });

    nativeZipArtifact =
        target
            .getArtifacts()
            .create(
                "nativeZips" + name,
                MacMiniJNILibraryArtifact.class,
                artifact -> {
                  target.setDeployStage(artifact, DeployStage.FileDeploy);

                  var cbl =
                      target
                          .getProject()
                          .getProviders()
                          .provider(
                              () -> {
                                boolean debug =
                                    target
                                        .getProject()
                                        .getExtensions()
                                        .getByType(WPIExtension.class)
                                        .getJava()
                                        .getDebugJni()
                                        .get();
                                if (debug) {
                                  return debugConfiguration;
                                } else {
                                  return releaseConfiguration;
                                }
                              });

                  artifact.getConfiguration().set(cbl);
                  artifact.setZipped(true);
                  artifact.getFilter().include("**/*.dylib*");
                  artifact.getFilter().include("**/*.dylib");
                  artifact.getFilter().getExcludes().add("**/*.dylib.debug");
                  artifact.getFilter().getExcludes().add("**/*.dylib.*.debug");
                });

    programStartArtifact.getPostdeploy().add(this::postStart);

    getPostdeploy()
        .add(
            ctx -> {
              String binFile = getBinFile(ctx);
              ctx.execute("chmod +x \"" + binFile + "\"; chown frc6328 \"" + binFile + "\"");
            });

    target.setDeployStage(this, DeployStage.FileDeploy);
  }

  public String getJavaCommand() {
    return javaCommand;
  }

  public void setJavaCommand(String javaCommand) {
    this.javaCommand = javaCommand;
  }

  public GarbageCollectorType getGcType() {
    return gcType;
  }

  public void setGcType(GarbageCollectorType gcType) {
    this.gcType = gcType;
  }

  private String getBinFile(DeployContext ctx) {
    return PathUtils.combine(
        ctx.getWorkingDir(), getFilename().getOrElse(getFile().get().getName()));
  }

  @Override
  public void setJarTask(Jar jarTask) {
    robotCommandArtifact.getDeployTask().configure(x -> x.dependsOn(jarTask));
    super.setJarTask(jarTask);
  }

  @Override
  public void setJarTask(TaskProvider<Jar> jarTask) {
    robotCommandArtifact.getDeployTask().configure(x -> x.dependsOn(jarTask));
    super.setJarTask(jarTask);
  }

  public MacMiniProgramStartArtifact getProgramStartArtifact() {
    return programStartArtifact;
  }

  public MacMiniRobotCommandArtifact getRobotCommandArtifact() {
    return robotCommandArtifact;
  }

  public MacMiniJNILibraryArtifact getNativeZipArtifact() {
    return nativeZipArtifact;
  }

  public List<String> getJvmArgs() {
    return jvmArgs;
  }

  public List<String> getArguments() {
    return arguments;
  }

  private String generateStartCommand(DeployContext ctx) {
    StringBuilder builder = new StringBuilder();
    builder.append(javaCommand);
    builder.append(" ");
    builder.append(String.join(" ", gcType.getGcArguments()));
    builder.append(" ");
    builder.append(String.join(" ", jvmArgs));
    builder.append(" ");

    // Debug stuff
    boolean debug = macMini.getDebug().get();
    if (debug) {
      builder.append("-XX:+UsePerfData -agentlib:jdwp=transport=dt_socket,address=0.0.0.0:");
      builder.append(getDebugPort());
      builder.append(",server=y,suspend=y ");
    }

    String binFile = getBinFile(ctx);

    builder.append("-jar \"");
    builder.append(binFile);
    builder.append("\" ");
    builder.append(String.join(" ", arguments));

    return builder.toString();
  }

  private void postStart(DeployContext ctx) {
    boolean debug = macMini.getDebug().get();
    if (debug) {
      ctx.getLogger()
          .withLock(
              x -> {
                x.log("====================================================================");
                x.log("DEBUGGING ACTIVE ON PORT " + getDebugPort() + "!");
                x.log("====================================================================");
              });
    }
  }
}
