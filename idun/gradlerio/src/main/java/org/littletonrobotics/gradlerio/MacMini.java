// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.gradlerio;

import edu.wpi.first.deployutils.deploy.DeployExtension;
import edu.wpi.first.deployutils.deploy.target.location.SshDeployLocation;
import edu.wpi.first.gradlerio.deploy.FRCExtension;
import edu.wpi.first.gradlerio.deploy.WPIRemoteTarget;
import edu.wpi.first.gradlerio.deploy.roborio.*;
import javax.inject.Inject;
import org.gradle.api.Project;
import org.gradle.api.logging.Logger;
import org.gradle.api.logging.Logging;

public class MacMini extends WPIRemoteTarget {
  private final Logger log;
  private int team;

  private final MacMiniProgramKillArtifact programKillArtifact;

  @Inject
  public MacMini(String name, Project project, DeployExtension de, FRCExtension frcExtension) {
    super(name, project, de, frcExtension);
    log = Logging.getLogger(this.toString());

    setDirectory("/Users/frc6328/robot");

    setMaxChannels(4);

    // Increase timeout. The only time this is really used is if the host is resolved,
    // but takes forever to connect, which can happen if the CPU is loaded.
    setTimeout(7);

    programKillArtifact =
        project
            .getObjects()
            .newInstance(MacMiniProgramKillArtifact.class, "programKill" + name, this);

    getTargetPlatform().set("osxuniversal");

    getArtifacts().add(programKillArtifact);
  }

  public MacMiniProgramKillArtifact getProgramKillArtifact() {
    return programKillArtifact;
  }

  public int getTeam() {
    return team;
  }

  public void setTeam(int team) {
    this.team = team;
    setAddresses("10." + (team / 100) + "." + (team % 100) + ".10");
  }

  public void setAddresses(String... addresses) {
    this.getLocations().clear();

    for (String addr : addresses) {
      this.addAddress(addr);
    }

    getLocations()
        .create(
            "ds",
            DSDeployLocation.class,
            ds -> {
              ds.setUser("frc" + Integer.toString(getTeam()));
              ds.setPassword("password");
              ds.setIpv6(false);
            });
  }

  public void addAddress(String address) {
    getLocations()
        .create(
            address,
            SshDeployLocation.class,
            loc -> {
              loc.setAddress(address);
              loc.setIpv6(false);
              loc.setUser("frc" + Integer.toString(getTeam()));
              loc.setPassword("password");
            });
  }

  @Override
  public String toString() {
    return "MacMini[" + getName() + "]";
  }
}
