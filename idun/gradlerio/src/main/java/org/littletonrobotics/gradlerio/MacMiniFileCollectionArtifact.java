// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.gradlerio;

import edu.wpi.first.deployutils.deploy.artifact.*;
import edu.wpi.first.deployutils.deploy.cache.CacheMethod;
import edu.wpi.first.deployutils.deploy.context.DeployContext;
import edu.wpi.first.deployutils.deploy.target.RemoteTarget;
import edu.wpi.first.deployutils.log.ETLogger;
import javax.inject.Inject;
import org.gradle.api.file.FileCollection;
import org.gradle.api.provider.Property;

public class MacMiniFileCollectionArtifact extends AbstractArtifact implements CacheableArtifact {

  private final Property<CacheMethod> cacheMethod;

  @Inject
  public MacMiniFileCollectionArtifact(String name, RemoteTarget target) {
    super(name, target);
    files = target.getProject().getObjects().property(FileCollection.class);
    cacheMethod = target.getProject().getObjects().property(CacheMethod.class);

    cacheMethod.set(new MacMiniMd5SumCacheMethod(name)); // Force macOS-compatible cache method
  }

  private final Property<FileCollection> files;

  public Property<FileCollection> getFiles() {
    return files;
  }

  @Override
  public Property<CacheMethod> getCacheMethod() {
    return cacheMethod;
  }

  @Override
  public void deploy(DeployContext context) {
    if (files.isPresent()) context.put(files.get().getFiles(), cacheMethod.getOrElse(null));
    else {
      ETLogger logger = context.getLogger();
      if (logger != null) {
        logger.log("No file(s) provided for " + toString());
      }
    }
  }
}
